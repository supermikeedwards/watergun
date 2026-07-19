/* Birdblast SPA (worklog §18). Pure static — no backend.
 *
 * Auth:   Cognito User Pool SRP login -> id token.
 * Creds:  Cognito Identity Pool -> scoped AWS creds (IoT + S3 read).
 * Live:   MQTT over WSS (SigV4-presigned) to AWS IoT -> Device Shadow + topics.
 * Images: S3 list via SDK, displayed through CloudFront (same domain).
 *
 * The Images + Config tabs work with the Pi OFFLINE (data lives in AWS: S3 + the
 * durable Device Shadow). Calibration + live video require the Pi online.
 *
 * NOTE: this talks to real AWS APIs that can't be exercised until the infra is
 * deployed + a user exists. Treat the first load as a debugging pass (per the
 * project's hard-won lesson about blind-written API calls).
 */
const CFG = window.BIRDBLAST || {};
const SHADOW = `$aws/things/${CFG.thingName}/shadow`;
const OFFLINE_AFTER_MS = 35000; // no heartbeat within this window => offline

const BB = {
  mqtt: null,
  reported: {},          // last reported shadow (status/config/telemetry/heartbeat/online)
  editedConfig: null,    // working copy for the Config tab
  lastHeartbeat: 0,
  tab: 'images',
  calX: 92, calY: 48,
  viewerTimer: null,
};

// ------------------------------------------------------------------ auth
BB.userPool = () => new AmazonCognitoIdentity.CognitoUserPool({
  UserPoolId: CFG.userPoolId, ClientId: CFG.userPoolClientId,
});

BB.login = function () {
  const email = document.getElementById('email').value.trim();
  const password = document.getElementById('password').value;
  const errEl = document.getElementById('loginErr');
  errEl.textContent = '';
  const user = new AmazonCognitoIdentity.CognitoUser({ Username: email, Pool: BB.userPool() });
  const details = new AmazonCognitoIdentity.AuthenticationDetails({ Username: email, Password: password });
  user.authenticateUser(details, {
    onSuccess: (session) => BB.onAuth(session.getIdToken().getJwtToken()),
    onFailure: (err) => { errEl.textContent = err.message || String(err); },
    newPasswordRequired: () => { errEl.textContent = 'Password change required — set it in the Cognito console first.'; },
  });
};

BB.tryResume = function () {
  const user = BB.userPool().getCurrentUser();
  if (!user) return BB.showLogin();
  user.getSession((err, session) => {
    if (err || !session.isValid()) return BB.showLogin();
    BB.onAuth(session.getIdToken().getJwtToken());
  });
};

BB.onAuth = function (idToken) {
  const key = `cognito-idp.${CFG.region}.amazonaws.com/${CFG.userPoolId}`;
  AWS.config.region = CFG.region;
  AWS.config.credentials = new AWS.CognitoIdentityCredentials({
    IdentityPoolId: CFG.identityPoolId,
    Logins: { [key]: idToken },
  });
  AWS.config.credentials.get((err) => {
    if (err) { document.getElementById('loginErr').textContent = 'Identity Pool error: ' + err.message; return; }
    document.getElementById('login').style.display = 'none';
    document.getElementById('app').style.display = '';
    BB.connectIot();
    BB.loadImages();
  });
};

BB.showLogin = function () {
  document.getElementById('login').style.display = '';
  document.getElementById('app').style.display = 'none';
};

// ------------------------------------------------------------------ IoT (MQTT/WSS)
// SigV4-presigned wss URL for AWS IoT (standard pattern, CryptoJS).
BB.presignIotUrl = function () {
  const c = AWS.config.credentials;
  const region = CFG.region, host = CFG.iotEndpoint;
  const now = new Date();
  const ymd = now.toISOString().slice(0, 10).replace(/-/g, '');
  const amz = ymd + 'T' + now.toISOString().slice(11, 19).replace(/:/g, '') + 'Z';
  const service = 'iotdevicegateway';
  const scope = `${ymd}/${region}/${service}/aws4_request`;
  const ak = c.accessKeyId, sk = c.secretAccessKey, st = c.sessionToken;
  let qs = 'X-Amz-Algorithm=AWS4-HMAC-SHA256'
    + '&X-Amz-Credential=' + encodeURIComponent(ak + '/' + scope)
    + '&X-Amz-Date=' + amz + '&X-Amz-SignedHeaders=host';
  const canonical = `GET\n/mqtt\n${qs}\nhost:${host}\n\nhost\n` + CryptoJS.SHA256('').toString();
  const toSign = `AWS4-HMAC-SHA256\n${amz}\n${scope}\n` + CryptoJS.SHA256(canonical).toString();
  const kDate = CryptoJS.HmacSHA256(ymd, 'AWS4' + sk);
  const kRegion = CryptoJS.HmacSHA256(region, kDate);
  const kService = CryptoJS.HmacSHA256(service, kRegion);
  const kSigning = CryptoJS.HmacSHA256('aws4_request', kService);
  const sig = CryptoJS.HmacSHA256(toSign, kSigning).toString();
  let url = `wss://${host}/mqtt?${qs}&X-Amz-Signature=${sig}`;
  if (st) url += '&X-Amz-Security-Token=' + encodeURIComponent(st);
  return url;
};

BB.connectIot = function () {
  const clientId = 'birdblast-web-' + Math.random().toString(16).slice(2, 10);
  const client = mqtt.connect(BB.presignIotUrl(), { clientId, reconnectPeriod: 0, keepalive: 30 });
  BB.mqtt = client;
  client.on('connect', () => {
    client.subscribe(`${SHADOW}/update/accepted`);
    client.subscribe(`${SHADOW}/get/accepted`);
    client.subscribe(`${SHADOW}/update/documents`);
    client.subscribe('watergun/cmd/resp');
    client.subscribe('watergun/stream');
    client.publish(`${SHADOW}/get`, '');   // pull last-known (works Pi-offline)
  });
  client.on('message', (t, payload) => BB.onIot(t, payload));
  // Reconnect with a FRESH presigned URL (they expire); mqtt.js reconnectPeriod=0 disables auto.
  client.on('close', () => setTimeout(() => { if (AWS.config.credentials) BB.reconnectIot(); }, 3000));
  client.on('error', (e) => console.warn('mqtt error', e && e.message));
  BB.tickStatus();
  setInterval(() => BB.tickStatus(), 5000);
};

BB.reconnectIot = function () {
  AWS.config.credentials.refresh(() => { try { BB.mqtt.end(true); } catch (e) {} BB.connectIot(); });
};

BB.onIot = function (topic, payload) {
  let msg = {};
  try { msg = JSON.parse(payload.toString()); } catch (e) { return; }
  if (topic === 'watergun/stream') { BB.onStreamFrame(msg); return; }
  if (topic === 'watergun/cmd/resp') { console.log('cmd resp', msg); return; }
  // Shadow docs
  const state = msg.state || {};
  const rep = state.reported || state;   // update/documents nests under .current.state; get/accepted under .state
  const merged = (msg.current && msg.current.state && msg.current.state.reported) || rep;
  Object.assign(BB.reported, merged || {});
  if (BB.reported.heartbeat) BB.lastHeartbeat = BB.reported.heartbeat * 1000;
  if (BB.reported.online) BB.lastHeartbeat = Math.max(BB.lastHeartbeat, Date.now() - 1);
  BB.render();
};

BB.online = function () { return (Date.now() - BB.lastHeartbeat) < OFFLINE_AFTER_MS; };

BB.publishDesired = function (partial) {
  if (!BB.mqtt || !BB.mqtt.connected) { alert('Not connected to the cloud yet.'); return; }
  BB.mqtt.publish(`${SHADOW}/update`, JSON.stringify({ state: { desired: partial } }));
};

BB.sendCmd = function (action, params) {
  if (!BB.online()) { alert('The watergun is offline.'); return; }
  BB.mqtt.publish('watergun/cmd', JSON.stringify({ req_id: Date.now(), action, params: params || {} }));
};

// ------------------------------------------------------------------ rendering
BB.show = function (tab) {
  BB.tab = tab;
  for (const s of ['images', 'config', 'status', 'cal'])
    document.getElementById(s).style.display = s === tab ? '' : 'none';
  const map = { images: 'tImages', config: 'tConfig', status: 'tStatus', cal: 'tCal' };
  for (const k in map) document.getElementById(map[k]).classList.toggle('active', k === tab);
  if (tab === 'config') BB.renderConfig();
  if (tab === 'cal') BB.enterViewer(); else BB.leaveViewer();
};

BB.tickStatus = function () {
  const on = BB.online();
  document.getElementById('dot').className = 'dot ' + (on ? 'online' : 'offline');
  let txt = on ? 'Online' : 'Offline';
  if (!on && BB.lastHeartbeat) {
    const mins = Math.round((Date.now() - BB.lastHeartbeat) / 60000);
    txt = mins < 120 ? `Last seen ${mins} min ago` : `Last seen ${Math.round(mins / 60)} h ago`;
  }
  document.getElementById('statusText').textContent = txt;
  document.getElementById('tCal').disabled = !on;
  BB.render();
};

BB.render = function () {
  const on = BB.online();
  const st = BB.reported.status || {};
  // Status tab pills + toggles
  const pills = document.getElementById('statusPills');
  if (pills) {
    pills.innerHTML =
      `<span class="pill ${st.armed ? 'on' : 'off'}">${st.armed ? 'ARMED' : 'NOT ARMED'}</span>` +
      `<span class="pill ${st.mode === 'active' ? 'on' : ''}">${(st.mode || '—').toUpperCase()}</span>` +
      `<span class="pill ${st.kids_mode ? 'info' : ''}">${st.kids_mode ? 'KIDS MODE' : 'BIRD MODE'}</span>` +
      (st.calibrating ? '<span class="pill info">CALIBRATING</span>' : '');
    const w = document.getElementById('tgWater'); if (w) w.checked = !!st.water_enabled;
    const k = document.getElementById('tgKids'); if (k) k.checked = !!st.kids_mode;
    const ob = document.getElementById('offlineBanner');
    ob.style.display = on ? 'none' : '';
    ob.textContent = 'Watergun offline — changes below are queued and applied when it reconnects.';
    const tel = BB.reported.telemetry || {};
    document.getElementById('telemetry').textContent =
      `last detection: ${st.last_detection || '—'}\nlast spray: ${st.last_spray || '—'}` +
      (tel.temp ? `\ntemp: ${tel.temp}  throttled: ${tel.throttled || '—'}` : '');
  }
  // Calibrate availability
  document.getElementById('calOffline').style.display = on ? 'none' : '';
  document.getElementById('calControls').style.display = on ? '' : 'none';
  if (on) {
    const cal = document.getElementById('calToggle');
    cal.textContent = st.calibrating ? 'Exit calibration mode' : 'Enter calibration mode';
    cal.className = 'btn ' + (st.calibrating ? 'red' : 'amber');
    document.getElementById('calLive').style.display = st.calibrating ? '' : 'none';
  }
};

// ------------------------------------------------------------------ Config tab
BB.renderConfig = function () {
  const cfg = BB.reported.config;
  const host = document.getElementById('cfgForm');
  if (!cfg) { host.innerHTML = '<span class="muted">No config reported yet.</span>'; return; }
  BB.editedConfig = JSON.parse(JSON.stringify(cfg));
  // Only expose the safe "tuning" sections — never certs/endpoints (cloud) etc.
  const SECTIONS = ['opening_hours', 'oak', 'detection', 'aim', 'spray', 'images', 'kids_mode'];
  let h = '';
  for (const sec of SECTIONS) {
    if (!(sec in cfg)) continue;
    h += `<h3 style="font-size:15px;margin:16px 0 4px">${sec.replace(/_/g, ' ')}</h3>`;
    h += BB.renderObj(sec, cfg[sec]);
  }
  host.innerHTML = h;
};

BB.renderObj = function (prefix, obj) {
  let h = '';
  for (const [k, v] of Object.entries(obj)) {
    const path = `${prefix}.${k}`;
    if (v !== null && typeof v === 'object' && !Array.isArray(v)) { h += BB.renderObj(path, v); continue; }
    const t = v === null ? 'null' : Array.isArray(v) ? 'array' : typeof v;
    const disp = v === null ? '' : (t === 'string' ? v : JSON.stringify(v));
    h += `<div class="field"><label>${path}</label>` +
         `<input data-path="${path}" data-type="${t}" value="${String(disp).replace(/"/g, '&quot;')}"></div>`;
  }
  return h;
};

BB.saveConfig = function () {
  const patch = {};
  for (const el of document.querySelectorAll('#cfgForm input[data-path]')) {
    const t = el.dataset.type, raw = el.value; let v;
    try {
      if (t === 'number') { v = Number(raw); if (Number.isNaN(v)) throw 0; }
      else if (t === 'boolean') { if (raw !== 'true' && raw !== 'false') throw 0; v = raw === 'true'; }
      else if (t === 'array') { v = JSON.parse(raw); if (!Array.isArray(v)) throw 0; }
      else if (t === 'null') { v = raw === '' ? null : (isNaN(Number(raw)) ? raw : Number(raw)); }
      else v = raw;
    } catch (e) { alert('Invalid value at ' + el.dataset.path); return; }
    BB.setPath(patch, el.dataset.path, v);
  }
  BB.publishDesired({ config: patch });
  alert(BB.online() ? 'Saved — applying now.' : 'Saved — will apply when the watergun reconnects.');
};

BB.setPath = function (o, path, v) {
  const p = path.split('.');
  for (let i = 0; i < p.length - 1; i++) o = (o[p[i]] = o[p[i]] || {});
  o[p[p.length - 1]] = v;
};

// ------------------------------------------------------------------ toggles
BB.toggleWater = function () { BB.publishDesired({ status: { water_enabled: document.getElementById('tgWater').checked } }); };
BB.toggleKids = function () { BB.publishDesired({ status: { kids_mode: document.getElementById('tgKids').checked } }); };

// ------------------------------------------------------------------ calibration
BB.toggleCal = function () {
  const calibrating = (BB.reported.status || {}).calibrating;
  BB.sendCmd(calibrating ? 'exit' : 'enter', {});
};
BB.step = function (axis, d) {
  if (axis === 'x') BB.calX = Math.max(0, Math.min(180, BB.calX + d));
  else BB.calY = Math.max(0, Math.min(180, BB.calY + d));
  BB.sendCmd('jog', { x: BB.calX, y: BB.calY });
};
BB.fire = function () { BB.sendCmd('fire', { duration_ms: 300 }); };

BB.onStreamFrame = function (msg) {
  if (!msg.jpeg_b64) return;
  const img = document.getElementById('stream');
  img.src = 'data:image/jpeg;base64,' + msg.jpeg_b64;
  const ph = document.getElementById('streamPh'); if (ph) ph.style.display = 'none';
};
// Signal viewer presence so the Pi streams frames only while the tab is open.
BB.enterViewer = function () {
  if (BB.viewerTimer) return;
  const ping = () => { if (BB.mqtt && BB.mqtt.connected) BB.mqtt.publish('watergun/viewer', JSON.stringify({ present: true, ts: Date.now() })); };
  ping(); BB.viewerTimer = setInterval(ping, 4000);
};
BB.leaveViewer = function () { if (BB.viewerTimer) { clearInterval(BB.viewerTimer); BB.viewerTimer = null; } };

// tap-to-aim
document.addEventListener('click', (e) => {
  const img = document.getElementById('stream');
  if (!img || e.target !== img) return;
  const r = img.getBoundingClientRect();
  const nx = (e.clientX - r.left) / r.width, ny = (e.clientY - r.top) / r.height;
  const ch = document.getElementById('crosshair');
  ch.style.left = (nx * r.width) + 'px'; ch.style.top = (ny * r.height) + 'px'; ch.style.opacity = 1;
  setTimeout(() => { ch.style.opacity = 0; }, 800);
  BB.sendCmd('aim', { nx, ny, fire: true, duration_ms: 300 });
});

// ------------------------------------------------------------------ Images (S3)
BB.loadImages = function () {
  const s3 = new AWS.S3({ region: CFG.region });
  s3.listObjectsV2({ Bucket: CFG.imagesBucket, Prefix: 'images/', MaxKeys: 60 }, (err, data) => {
    const g = document.getElementById('gallery');
    if (err) { g.innerHTML = `<span class="muted">Could not load images: ${err.message}</span>`; return; }
    const items = (data.Contents || []).filter(o => o.Key.endsWith('.jpg'))
      .sort((a, b) => new Date(b.LastModified) - new Date(a.LastModified));
    if (!items.length) { g.innerHTML = '<span class="muted">No detections yet.</span>'; return; }
    g.innerHTML = items.map(o => {
      const url = `${CFG.domain}/${o.Key}`;
      const when = new Date(o.LastModified).toLocaleString();
      return `<figure><img loading="lazy" src="${url}" alt=""><figcaption>${when}</figcaption></figure>`;
    }).join('');
  });
};

// ------------------------------------------------------------------ boot
BB.tryResume();
window.BB = BB;
