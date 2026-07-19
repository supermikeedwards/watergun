"""Mobile-friendly web UI. Tabs: Logs / Control / Config / Calibrate. No auth (on trusted Wi-Fi)."""
import json
import logging
import time
from flask import Flask, jsonify, request, Response, abort

from . import config, logging_setup, calibration, hardware, calibrate_ops
from .state import state

log = logging.getLogger(__name__)
app = Flask(__name__)

INDEX_HTML = """<!doctype html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pigeon Watergun</title>
<style>
 *{box-sizing:border-box}
 body{font-family:-apple-system,system-ui,sans-serif;margin:0;background:#111;color:#eee}
 header{background:#222;padding:10px 14px;font-size:18px;border-bottom:1px solid #333}
 nav{display:flex;background:#1a1a1a;border-bottom:1px solid #333}
 nav button{flex:1;background:none;border:none;color:#aaa;padding:14px;font-size:14px}
 nav button.active{color:#fff;border-bottom:2px solid #4aa3ff}
 main{padding:14px}
 .card{background:#1c1c1c;border:1px solid #2a2a2a;border-radius:8px;padding:14px;margin-bottom:12px}
 pre{background:#000;color:#0f0;padding:10px;border-radius:6px;max-height:60vh;overflow:auto;font-size:12px;white-space:pre-wrap;word-break:break-all}
 button.big{width:100%;padding:18px;font-size:18px;border:none;border-radius:8px;color:#fff;margin-top:8px}
 button.on{background:#2a8f2a}
 button.off{background:#c23}
 button.warn{background:#e08000}
 button.neutral{background:#444}
 label{display:block;margin-top:10px;font-size:13px;color:#aaa}
 input,select{width:100%;padding:10px;background:#000;border:1px solid #333;color:#eee;border-radius:6px;font-size:15px}
 .status{font-size:14px;margin-bottom:10px}
 .pill{display:inline-block;padding:2px 8px;border-radius:10px;font-size:12px;margin-left:6px}
 .pill.on{background:#2a8f2a}.pill.off{background:#c23}.pill.sleep{background:#555}.pill.cal{background:#4aa3ff}
 .row{display:flex;gap:8px}.row>*{flex:1}
 .jog{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:8px}
 .jog button{padding:10px;background:#333;border:1px solid #444;color:#eee;border-radius:6px;font-size:14px}
 .stream-wrap{position:relative;width:100%;background:#000;border-radius:6px;overflow:hidden}
 .stream-wrap img{display:block;width:100%;height:auto;touch-action:manipulation}
 .stream-wrap .crosshair{position:absolute;pointer-events:none;width:20px;height:20px;margin:-10px 0 0 -10px;border:2px solid #4aa3ff;border-radius:50%;opacity:0}
 .slider-row{display:flex;align-items:center;gap:8px;margin-top:6px}
 .slider-row input[type=range]{flex:1}
 .slider-row .val{min-width:50px;text-align:right;font-family:monospace;color:#4aa3ff}
 .small{font-size:12px;color:#888}
</style></head><body>
<header>💦 Pigeon Watergun</header>
<nav>
 <button id="tLogs" class="active" onclick="show('logs')">Logs</button>
 <button id="tCtrl" onclick="show('ctrl')">Control</button>
 <button id="tCfg" onclick="show('cfg')">Config</button>
 <button id="tCal" onclick="show('cal')">Calibrate</button>
</nav>
<main>
 <section id="logs">
  <div class="card"><div class="status" id="stat">…</div>
  <pre id="log">loading…</pre></div>
 </section>
 <section id="ctrl" style="display:none">
  <div class="card">
   <div class="status" id="stat2">…</div>
   <button id="toggle" class="big on" onclick="toggle()">…</button>
   <p style="color:#888;font-size:12px;margin-top:12px">AND-gate: web UI and physical switch must <b>both</b> be ON for water to spray. Detection keeps running regardless.</p>
  </div>
 </section>
 <section id="cfg" style="display:none">
  <div class="card" id="cfgForm">loading…</div>
 </section>
 <section id="cal" style="display:none">
  <div class="card">
   <div class="status" id="calStat">…</div>
   <button id="calToggle" class="big warn" onclick="toggleCal()">Enter calibration mode</button>
   <p class="small">In calibration mode, detection + auto-spray are paused. The "Fire test shot" button below bypasses the armed AND-gate, so it will fire water even if the main controls are OFF — only while this tab is active.</p>
  </div>
  <div class="card" id="calControls" style="display:none">
   <div class="stream-wrap" id="streamWrap">
    <img id="stream" alt="camera stream">
    <div class="crosshair" id="crosshair"></div>
   </div>
   <p class="small">Tap the image to aim the gun at that point and fire a short test burst.</p>

   <h3 style="margin-top:14px">Jog servos</h3>
   <div class="slider-row"><span>X</span><input type="range" id="jogX" min="0" max="180" step="0.5" oninput="jogX(this.value)"><span class="val" id="jogXv">—</span></div>
   <div class="slider-row"><span>Y</span><input type="range" id="jogY" min="0" max="180" step="0.5" oninput="jogY(this.value)"><span class="val" id="jogYv">—</span></div>
   <div class="jog">
    <button onclick="step('x',-1)">X −1°</button>
    <button onclick="step('x',+1)">X +1°</button>
    <button onclick="step('x',-5)">X −5°</button>
    <button onclick="step('x',+5)">X +5°</button>
    <button onclick="step('y',-1)">Y −1°</button>
    <button onclick="step('y',+1)">Y +1°</button>
    <button onclick="step('y',-5)">Y −5°</button>
    <button onclick="step('y',+5)">Y +5°</button>
   </div>
   <button class="big neutral" style="margin-top:8px" onclick="recenter()">Move to current Center</button>

   <h3 style="margin-top:14px">Save current angles as…</h3>
   <div class="row">
    <button onclick="setCal('SERVO_X_CENTER','SERVO_Y_CENTER')" class="big neutral">Center (X & Y)</button>
   </div>
   <div class="row">
    <button onclick="setCal('SERVO_X_MIN_ANGLE',null)" class="big neutral">X min</button>
    <button onclick="setCal('SERVO_X_MAX_ANGLE',null)" class="big neutral">X max</button>
   </div>
   <div class="row">
    <button onclick="setCal(null,'SERVO_Y_MIN_ANGLE')" class="big neutral">Y min</button>
    <button onclick="setCal(null,'SERVO_Y_MAX_ANGLE')" class="big neutral">Y max</button>
   </div>
   <pre id="calVals" style="margin-top:10px;font-size:11px;max-height:14vh">loading…</pre>

   <h3 style="margin-top:14px">Fire test shot</h3>
   <div class="slider-row"><span>Burst</span><input type="range" id="burstMs" min="100" max="1500" step="50" value="300" oninput="document.getElementById('burstMsV').textContent=this.value+'ms'"><span class="val" id="burstMsV">300ms</span></div>
   <button class="big off" onclick="fireBurst()">Fire burst at current aim</button>
  </div>
 </section>
</main>
<script>
const S={x:90,y:90};
function show(t){for(const s of ['logs','ctrl','cfg','cal'])document.getElementById(s).style.display=s===t?'':'none';
 for(const b of ['tLogs','tCtrl','tCfg','tCal'])document.getElementById(b).classList.remove('active');
 document.getElementById({logs:'tLogs',ctrl:'tCtrl',cfg:'tCfg',cal:'tCal'}[t]).classList.add('active');
 if(t==='cal')loadCal();}
async function refresh(){
 const s=await (await fetch('/api/status')).json();
 const badge=`<span class="pill ${s.armed?'on':'off'}">${s.armed?'ARMED':'NOT ARMED'}</span>
  <span class="pill ${s.water_enabled?'on':'off'}">WEB ${s.water_enabled?'ON':'OFF'}</span>
  <span class="pill ${s.switch_enabled?'on':'off'}">SWITCH ${s.switch_enabled?'ON':'OFF'}</span>
  <span class="pill ${s.mode==='active'?'on':'sleep'}">${s.mode.toUpperCase()}</span>
  ${s.calibrating?'<span class="pill cal">CALIBRATING</span>':''}`;
 document.getElementById('stat').innerHTML=badge;
 document.getElementById('stat2').innerHTML=badge+`<br><small>last detection: ${s.last_detection||'—'} • last spray: ${s.last_spray||'—'}</small>`;
 document.getElementById('calStat').innerHTML=badge;
 const t=document.getElementById('toggle');
 t.textContent=s.water_enabled?'STOP WATER':'START WATER';
 t.className='big '+(s.water_enabled?'off':'on');
 const ct=document.getElementById('calToggle');
 ct.textContent=s.calibrating?'Exit calibration mode':'Enter calibration mode';
 ct.className='big '+(s.calibrating?'off':'warn');
 document.getElementById('calControls').style.display=s.calibrating?'':'none';
 const l=await (await fetch('/api/logs')).text();
 const pre=document.getElementById('log');pre.textContent=l;pre.scrollTop=pre.scrollHeight;
}
async function toggle(){await fetch('/api/toggle',{method:'POST'});refresh();}
// Render a leaf as an input. Tag with data-type so saveCfg can coerce correctly.
function leafInput(path,v){
 const t=v===null?'null':Array.isArray(v)?'array':typeof v;
 const display=v===null?'null':(t==='string'?v:JSON.stringify(v));
 const esc=String(display).replace(/&/g,'&amp;').replace(/"/g,'&quot;');
 return `<label>${path}</label><input data-path="${path}" data-type="${t}" value="${esc}">`;
}
function renderObj(prefix,obj){
 let h='';
 for(const [k,v] of Object.entries(obj)){
  const path=prefix?`${prefix}.${k}`:k;
  if(v!==null&&typeof v==='object'&&!Array.isArray(v)){
   h+=renderObj(path,v);
  }else{
   h+=leafInput(path,v);
  }
 }
 return h;
}
async function loadCfg(){
 const c=await (await fetch('/api/config')).json();
 let h='';
 for(const [sec,obj] of Object.entries(c)){
  h+=`<h3 style="margin-top:14px">${sec}</h3>`;
  if(obj!==null&&typeof obj==='object'&&!Array.isArray(obj)){
   h+=renderObj(sec,obj);
  }else{
   h+=leafInput(sec,obj);
  }
 }
 h+='<button class="big on" style="margin-top:16px" onclick="saveCfg()">Save (restart service to apply hardware/camera changes)</button>';
 document.getElementById('cfgForm').innerHTML=h;
}
function setByPath(o,path,v){
 const p=path.split('.');
 for(let i=0;i<p.length-1;i++)o=o[p[i]];
 o[p[p.length-1]]=v;
}
async function saveCfg(){
 const c=await (await fetch('/api/config')).json();
 for(const el of document.querySelectorAll('#cfgForm input[data-path]')){
  const path=el.dataset.path;const t=el.dataset.type;const raw=el.value;
  let v;
  try{
   if(t==='number'){v=Number(raw);if(Number.isNaN(v))throw new Error('not a number');}
   else if(t==='boolean'){if(raw==='true'||raw==='false')v=(raw==='true');else throw new Error('expected true/false');}
   else if(t==='null'){if(raw===''||raw==='null')v=null;else if(!Number.isNaN(Number(raw)))v=Number(raw);else v=raw;}
   else if(t==='array'){v=JSON.parse(raw);if(!Array.isArray(v))throw new Error('expected array');}
   else{v=raw;}
  }catch(e){alert(`Invalid value at ${path}: ${e.message}`);return;}
  setByPath(c,path,v);
 }
 const r=await fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(c)});
 if(r.ok){alert('Saved. Detection params apply immediately. Restart service for camera/hardware changes.');}
 else{const j=await r.json().catch(()=>({}));alert('Save rejected: '+(j.error||r.status));}
}

// --- Calibration tab ---
async function toggleCal(){
 const s=await (await fetch('/api/status')).json();
 const url=s.calibrating?'/api/calibrate/exit':'/api/calibrate/enter';
 await fetch(url,{method:'POST'});
 refresh();
 if(!s.calibrating){setTimeout(loadCal,200);}
}
async function loadCal(){
 // Start/refresh the stream (cache-bust)
 document.getElementById('stream').src='/stream.mjpg?t='+Date.now();
 const c=await (await fetch('/api/calibration')).json();
 S.x=c.SERVO_X_CENTER;S.y=c.SERVO_Y_CENTER;
 document.getElementById('jogX').value=S.x;document.getElementById('jogXv').textContent=S.x.toFixed(1);
 document.getElementById('jogY').value=S.y;document.getElementById('jogYv').textContent=S.y.toFixed(1);
 document.getElementById('calVals').textContent=Object.entries(c).map(([k,v])=>`${k}=${v}`).join('\\n');
}
async function postJog(){
 await fetch('/api/calibrate/jog',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify({x:S.x,y:S.y})});
}
function jogX(v){S.x=parseFloat(v);document.getElementById('jogXv').textContent=S.x.toFixed(1);postJog();}
function jogY(v){S.y=parseFloat(v);document.getElementById('jogYv').textContent=S.y.toFixed(1);postJog();}
function step(axis,delta){
 if(axis==='x'){S.x=Math.max(0,Math.min(180,S.x+delta));document.getElementById('jogX').value=S.x;document.getElementById('jogXv').textContent=S.x.toFixed(1);}
 else{S.y=Math.max(0,Math.min(180,S.y+delta));document.getElementById('jogY').value=S.y;document.getElementById('jogYv').textContent=S.y.toFixed(1);}
 postJog();
}
async function recenter(){
 const c=await (await fetch('/api/calibration')).json();
 S.x=c.SERVO_X_CENTER;S.y=c.SERVO_Y_CENTER;
 document.getElementById('jogX').value=S.x;document.getElementById('jogY').value=S.y;
 document.getElementById('jogXv').textContent=S.x.toFixed(1);document.getElementById('jogYv').textContent=S.y.toFixed(1);
 postJog();
}
async function setCal(xKey,yKey){
 const body={};
 if(xKey)body[xKey]=S.x;
 if(yKey)body[yKey]=S.y;
 const r=await fetch('/api/calibrate/set',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
 if(r.ok){loadCal();}else{alert('Save failed');}
}
async function fireBurst(){
 const ms=parseInt(document.getElementById('burstMs').value,10);
 const r=await fetch('/api/calibrate/fire',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({duration_ms:ms})});
 if(!r.ok){const j=await r.json().catch(()=>({}));alert('Fire rejected: '+(j.error||r.status));}
}

// Click-to-aim: tap the image, compute pixel -> angle, move servos, fire short burst
document.addEventListener('click',async(e)=>{
 const img=document.getElementById('stream');
 if(!img||e.target!==img)return;
 const r=img.getBoundingClientRect();
 const nx=(e.clientX-r.left)/r.width;
 const ny=(e.clientY-r.top)/r.height;
 // show crosshair
 const ch=document.getElementById('crosshair');
 ch.style.left=(nx*r.width)+'px';ch.style.top=(ny*r.height)+'px';ch.style.opacity=1;
 setTimeout(()=>{ch.style.opacity=0;},800);
 const ms=parseInt(document.getElementById('burstMs').value,10);
 const resp=await fetch('/api/calibrate/aim',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify({nx,ny,fire:true,duration_ms:ms})});
 const j=await resp.json().catch(()=>({}));
 if(!resp.ok){alert('Aim rejected: '+(j.error||resp.status));return;}
 // reflect new angles in sliders
 if(j.x!=null){S.x=j.x;document.getElementById('jogX').value=S.x;document.getElementById('jogXv').textContent=S.x.toFixed(1);}
 if(j.y!=null){S.y=j.y;document.getElementById('jogY').value=S.y;document.getElementById('jogYv').textContent=S.y.toFixed(1);}
});
loadCfg();refresh();setInterval(refresh,2000);
</script></body></html>
"""


@app.route("/")
def index():
    return Response(INDEX_HTML, mimetype="text/html")


@app.route("/api/status")
def api_status():
    return jsonify({
        "water_enabled": state.water_enabled,
        "switch_enabled": state.switch_enabled,
        "armed": state.armed,
        "mode": state.mode,
        "kids_mode": state.kids_mode,
        "calibrating": state.calibrating,
        "last_detection": state.last_detection,
        "last_spray": state.last_spray,
    })


@app.route("/api/toggle", methods=["POST"])
def api_toggle():
    with state.lock:
        state.water_enabled = not state.water_enabled
    log.info("Web UI toggle: water %s", "ENABLED" if state.water_enabled else "DISABLED")
    return jsonify({"water_enabled": state.water_enabled})


@app.route("/api/logs")
def api_logs():
    return "\n".join(logging_setup.recent(300))


@app.route("/api/config", methods=["GET", "POST"])
def api_config():
    if request.method == "GET":
        return jsonify(config.get())
    new_cfg = request.get_json(force=True)
    try:
        config.validate_against(new_cfg, config.get())
    except config.ConfigShapeError as e:
        log.warning("Web UI config save rejected: %s", e)
        return jsonify({"error": str(e)}), 400
    config.save(new_cfg)
    log.info("Web UI config saved")
    return jsonify({"ok": True})


# ------------------------- Calibration endpoints -------------------------
# All delegate to calibrate_ops (shared with the cloud command handler) so the
# aim formula + safety rules live in exactly one place.

def _cal_response(fn):
    """Run a calibrate_ops call, mapping CalibrationError to 409 (not in cal mode)
    or 400 (bad params)."""
    try:
        return jsonify({"ok": True, **(fn() or {})})
    except calibrate_ops.CalibrationError as e:
        code = 409 if "calibration mode" in str(e) else 400
        return jsonify({"error": str(e)}), code
    except Exception as e:
        log.warning("Calibration op failed: %s", e)
        return jsonify({"error": str(e)}), 500


@app.route("/api/toggle_mode", methods=["POST"])
def api_toggle_mode():
    with state.lock:
        state.kids_mode = not state.kids_mode
    log.info("Web UI mode toggle: kids_mode=%s", state.kids_mode)
    return jsonify({"kids_mode": state.kids_mode})


@app.route("/api/calibrate/enter", methods=["POST"])
def api_cal_enter():
    return _cal_response(calibrate_ops.enter)


@app.route("/api/calibrate/exit", methods=["POST"])
def api_cal_exit():
    return _cal_response(calibrate_ops.exit_)


@app.route("/api/calibration", methods=["GET"])
def api_cal_values():
    return jsonify(calibration.load())


@app.route("/api/calibrate/jog", methods=["POST"])
def api_cal_jog():
    body = request.get_json(force=True, silent=True) or {}
    return _cal_response(lambda: calibrate_ops.jog(body.get("x"), body.get("y")))


@app.route("/api/calibrate/set", methods=["POST"])
def api_cal_set():
    body = request.get_json(force=True, silent=True) or {}
    return _cal_response(lambda: calibrate_ops.set_cal(body))


@app.route("/api/calibrate/aim", methods=["POST"])
def api_cal_aim():
    body = request.get_json(force=True, silent=True) or {}
    return _cal_response(lambda: calibrate_ops.aim(
        body.get("nx"), body.get("ny"),
        fire=bool(body.get("fire", False)),
        duration_ms=int(body.get("duration_ms", 300))))


@app.route("/api/calibrate/fire", methods=["POST"])
def api_cal_fire():
    body = request.get_json(force=True, silent=True) or {}
    return _cal_response(lambda: calibrate_ops.fire(int(body.get("duration_ms", 300))))


@app.route("/stream.mjpg")
def stream_mjpg():
    """MJPEG multipart stream fed by state.latest_jpeg (updated by controller)."""
    boundary = "frame"

    def gen():
        last_ts = 0.0
        idle_start = time.time()
        while not state.exit_flag:
            jpeg = None
            ts = 0.0
            with state.lock:
                if state.latest_jpeg is not None:
                    jpeg = state.latest_jpeg
                    ts = state.latest_jpeg_ts
            if jpeg is not None and ts != last_ts:
                last_ts = ts
                idle_start = time.time()
                yield (b"--" + boundary.encode() + b"\r\n"
                       b"Content-Type: image/jpeg\r\n"
                       b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                       + jpeg + b"\r\n")
            else:
                # If no frame for >10s, drop the connection so the browser reconnects
                if time.time() - idle_start > 10:
                    break
            time.sleep(0.05)

    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=" + boundary)


def run(host, port):
    # Disable Flask's own access log for efficiency on the Pi
    logging.getLogger("werkzeug").setLevel(logging.WARNING)
    app.run(host=host, port=port, threaded=True, use_reloader=False)
