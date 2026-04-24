"""Mobile-friendly web UI. Tabs: Logs / Control / Config. No auth (on trusted Wi-Fi)."""
import json
import logging
from flask import Flask, jsonify, request, Response

from . import config, logging_setup
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
 nav button{flex:1;background:none;border:none;color:#aaa;padding:14px;font-size:15px}
 nav button.active{color:#fff;border-bottom:2px solid #4aa3ff}
 main{padding:14px}
 .card{background:#1c1c1c;border:1px solid #2a2a2a;border-radius:8px;padding:14px;margin-bottom:12px}
 pre{background:#000;color:#0f0;padding:10px;border-radius:6px;max-height:60vh;overflow:auto;font-size:12px;white-space:pre-wrap;word-break:break-all}
 button.big{width:100%;padding:18px;font-size:18px;border:none;border-radius:8px;color:#fff;margin-top:8px}
 button.on{background:#2a8f2a}
 button.off{background:#c23}
 label{display:block;margin-top:10px;font-size:13px;color:#aaa}
 input,select{width:100%;padding:10px;background:#000;border:1px solid #333;color:#eee;border-radius:6px;font-size:15px}
 .status{font-size:14px;margin-bottom:10px}
 .pill{display:inline-block;padding:2px 8px;border-radius:10px;font-size:12px;margin-left:6px}
 .pill.on{background:#2a8f2a}.pill.off{background:#c23}.pill.sleep{background:#555}
 .row{display:flex;gap:8px}.row>*{flex:1}
</style></head><body>
<header>💦 Pigeon Watergun</header>
<nav>
 <button id="tLogs" class="active" onclick="show('logs')">Logs</button>
 <button id="tCtrl" onclick="show('ctrl')">Control</button>
 <button id="tCfg" onclick="show('cfg')">Config</button>
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
   <p style="color:#888;font-size:12px;margin-top:12px">Stop = water off (detection keeps running), same as the physical switch.</p>
  </div>
 </section>
 <section id="cfg" style="display:none">
  <div class="card" id="cfgForm">loading…</div>
 </section>
</main>
<script>
function show(t){for(const s of ['logs','ctrl','cfg'])document.getElementById(s).style.display=s===t?'':'none';
 for(const b of ['tLogs','tCtrl','tCfg'])document.getElementById(b).classList.remove('active');
 document.getElementById({logs:'tLogs',ctrl:'tCtrl',cfg:'tCfg'}[t]).classList.add('active');}
async function refresh(){
 const s=await (await fetch('/api/status')).json();
 const badge=`<span class="pill ${s.water_enabled?'on':'off'}">${s.water_enabled?'WATER ON':'WATER OFF'}</span>
  <span class="pill ${s.mode==='active'?'on':'sleep'}">${s.mode.toUpperCase()}</span>`;
 document.getElementById('stat').innerHTML=badge;
 document.getElementById('stat2').innerHTML=badge+`<br><small>last detection: ${s.last_detection||'—'} • last spray: ${s.last_spray||'—'}</small>`;
 const t=document.getElementById('toggle');
 t.textContent=s.water_enabled?'STOP WATER':'START WATER';
 t.className='big '+(s.water_enabled?'off':'on');
 const l=await (await fetch('/api/logs')).text();
 const pre=document.getElementById('log');pre.textContent=l;pre.scrollTop=pre.scrollHeight;
}
async function toggle(){await fetch('/api/toggle',{method:'POST'});refresh();}
async function loadCfg(){
 const c=await (await fetch('/api/config')).json();
 let h='';
 for(const [sec,obj] of Object.entries(c)){
  h+=`<h3 style="margin-top:14px">${sec}</h3>`;
  for(const [k,v] of Object.entries(obj)){
   const id=`${sec}__${k}`;
   h+=`<label>${k}</label><input id="${id}" value="${typeof v==='object'?JSON.stringify(v):v}">`;
  }
 }
 h+='<button class="big on" style="margin-top:16px" onclick="saveCfg()">Save (restart service to apply hardware/camera changes)</button>';
 document.getElementById('cfgForm').innerHTML=h;
}
async function saveCfg(){
 const c=await (await fetch('/api/config')).json();
 for(const [sec,obj] of Object.entries(c))for(const k of Object.keys(obj)){
  const el=document.getElementById(`${sec}__${k}`);if(!el)continue;
  let v=el.value;
  if(typeof obj[k]==='number')v=Number(v);
  else if(typeof obj[k]==='boolean')v=(v==='true'||v===true);
  c[sec][k]=v;
 }
 const r=await fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(c)});
 alert(r.ok?'Saved. Detection params apply immediately. Restart service for camera/hardware changes.':'Save failed');
}
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
        "mode": state.mode,
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
    config.save(new_cfg)
    log.info("Web UI config saved")
    return jsonify({"ok": True})


def run(host, port):
    # Disable Flask's own access log for efficiency on the Pi
    logging.getLogger("werkzeug").setLevel(logging.WARNING)
    app.run(host=host, port=port, threaded=True, use_reloader=False)
