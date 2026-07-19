"""AWS IoT cloud client — makes the Pi a thin pub/sub client (worklog §18).

The Pi holds ONE outbound MQTT/TLS connection to AWS IoT Core. AWS owns the whole
web interface (static SPA in S3/CloudFront, Cognito auth, browser talks IoT-over-WSS).
This module:

  * connects to IoT Core with the device X.509 cert (no inbound ports on the Pi),
  * mirrors state + config + telemetry into the classic Device Shadow `reported`,
  * applies `desired` deltas (config edits + water/kids toggles) pushed from the SPA,
  * handles real-time calibration commands on watergun/cmd (online-only),
  * publishes live JPEG frames on watergun/stream while a viewer is present,
  * uploads detection images to S3 for the offline-browsable gallery,
  * emits a heartbeat so the SPA can show an online/offline status pill.

Deliberate dependency choice: **paho-mqtt + boto3 + requests** — all pure-Python,
ARM-safe. This avoids the awscrt/native-wheel pain that plagued tflite/cv2 on the
Pi 3 (see the May worklog entries). paho speaks plain MQTT; the Device Shadow is
just reserved MQTT topics, so no AWS-specific native lib is needed.

Everything here is best-effort and non-fatal: if the cloud is unreachable the
controller keeps running locally (LAN Flask UI still works). `cloud.enabled=false`
in config.json disables this module entirely.
"""
import json
import logging
import os
import ssl
import threading
import time

from . import config, calibration, calibrate_ops
from .state import state

log = logging.getLogger(__name__)


def _now():
    return int(time.time())


def _deep_merge(base, patch):
    """Recursively merge patch into a copy of base (dicts only). Used to fold a
    partial `desired.config` delta into the full current config before validating."""
    out = dict(base)
    for k, v in (patch or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


class CloudClient:
    def __init__(self, cfg):
        c = cfg["cloud"]
        self.cfg = cfg
        self.enabled = bool(c.get("enabled", False))
        self.thing = c["thing_name"]
        self.endpoint = c["endpoint"]
        self.port = int(c.get("port", 8883))
        self.region = c.get("region", "us-east-1")
        self.ca_path = _abspath(c["ca_path"])
        self.cert_path = _abspath(c["cert_path"])
        self.key_path = _abspath(c["key_path"])
        self.images_bucket = c.get("images_bucket", "")
        self.role_alias = c.get("role_alias", "")
        self.creds_endpoint = c.get("credentials_endpoint", "")
        self.heartbeat_s = int(c.get("heartbeat_seconds", 15))
        self.cmd_topic = c.get("cmd_topic", "watergun/cmd")
        self.resp_topic = self.cmd_topic + "/resp"
        self.stream_topic = c.get("stream_topic", "watergun/stream")
        self.viewer_topic = c.get("viewer_topic", "watergun/viewer")
        self.viewer_timeout = int(c.get("viewer_timeout_seconds", 10))

        self._shadow = f"$aws/things/{self.thing}/shadow"
        self._client = None
        self._connected = False
        self._last_viewer = 0.0
        self._s3 = None
        self._creds = None
        self._creds_exp = 0.0
        self._lock = threading.Lock()

    # ---------------------------------------------------------------- lifecycle
    def start(self):
        if not self.enabled:
            log.info("Cloud client disabled (cloud.enabled=false)")
            return
        for p in (self.ca_path, self.cert_path, self.key_path):
            if not os.path.exists(p):
                log.error("Cloud disabled: missing cert file %s", p)
                self.enabled = False
                return
        try:
            import paho.mqtt.client as mqtt
        except Exception as e:
            log.error("Cloud disabled: paho-mqtt not installed (%s)", e)
            self.enabled = False
            return

        # paho-mqtt 2.x requires an explicit callback API version; 1.x has no such arg.
        # Pin to VERSION1 so the (client, userdata, flags, rc) callbacks below work on both.
        try:
            cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1,
                             client_id=self.thing, clean_session=True)
        except (AttributeError, TypeError):
            cl = mqtt.Client(client_id=self.thing, clean_session=True)
        cl.tls_set(ca_certs=self.ca_path, certfile=self.cert_path,
                   keyfile=self.key_path, tls_version=ssl.PROTOCOL_TLSv1_2)
        cl.on_connect = self._on_connect
        cl.on_disconnect = self._on_disconnect
        cl.on_message = self._on_message
        # Last Will: if the Pi drops, AWS publishes offline so the SPA pill flips fast.
        cl.will_set(f"{self._shadow}/update",
                    json.dumps({"state": {"reported": {"heartbeat": 0, "online": False}}}),
                    qos=1, retain=False)
        self._client = cl
        try:
            cl.connect(self.endpoint, self.port, keepalive=max(self.heartbeat_s * 2, 30))
        except Exception as e:
            log.error("Cloud connect failed (%s); will retry in background", e)
        cl.loop_start()
        threading.Thread(target=self._heartbeat_loop, daemon=True).start()
        log.info("Cloud client starting: endpoint=%s thing=%s", self.endpoint, self.thing)

    def close(self):
        if self._client:
            try:
                self._publish_reported({"online": False, "heartbeat": 0})
                self._client.loop_stop()
                self._client.disconnect()
            except Exception:
                pass

    @property
    def connected(self):
        return self._connected

    def viewer_active(self):
        return (time.time() - self._last_viewer) < self.viewer_timeout

    # ------------------------------------------------------------------- MQTT
    def _on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            log.error("Cloud MQTT connect rc=%s", rc)
            return
        self._connected = True
        client.subscribe(f"{self._shadow}/update/delta", qos=1)
        client.subscribe(f"{self._shadow}/get/accepted", qos=1)
        client.subscribe(self.cmd_topic, qos=1)
        client.subscribe(self.viewer_topic, qos=0)
        # Sync: ask for current shadow (to pick up any desired set while offline),
        # then report our full current state + config.
        client.publish(f"{self._shadow}/get", "", qos=1)
        self.report_config(config.get())
        self.report_status()
        log.info("Cloud connected + subscribed (shadow delta/get, cmd, viewer)")

    def _on_disconnect(self, client, userdata, rc):
        self._connected = False
        log.warning("Cloud MQTT disconnected rc=%s (paho auto-reconnects)", rc)

    def _on_message(self, client, userdata, msg):
        try:
            topic = msg.topic
            payload = msg.payload.decode("utf-8") if msg.payload else ""
            if topic == self.viewer_topic:
                self._last_viewer = time.time()
                return
            if topic == self.cmd_topic:
                self._handle_command(json.loads(payload) if payload else {})
                return
            data = json.loads(payload) if payload else {}
            if topic.endswith("/update/delta"):
                self._handle_delta(data.get("state", {}))
            elif topic.endswith("/get/accepted"):
                desired = data.get("state", {}).get("desired", {})
                if desired:
                    self._handle_delta(desired)
        except Exception as e:
            log.warning("Cloud message error on %s: %s", getattr(msg, "topic", "?"), e)

    # --------------------------------------------------------------- shadow in
    def _handle_delta(self, delta):
        """Apply a `desired` delta from the SPA. Two channels: status toggles and
        config edits. After applying, report back so the delta clears."""
        reported_back = {}

        st = delta.get("status") or {}
        if "water_enabled" in st:
            with state.lock:
                state.water_enabled = bool(st["water_enabled"])
            log.info("Shadow desired -> water_enabled=%s", state.water_enabled)
        if "kids_mode" in st:
            with state.lock:
                state.kids_mode = bool(st["kids_mode"])
            log.info("Shadow desired -> kids_mode=%s", state.kids_mode)

        cfg_patch = delta.get("config")
        if cfg_patch:
            merged = _deep_merge(config.get(), cfg_patch)
            try:
                config.validate_against(merged, config.get())
            except config.ConfigShapeError as e:
                log.warning("Rejected desired.config: %s", e)
                self._publish_topic(self.resp_topic,
                                    {"type": "config_rejected", "error": str(e)})
            else:
                config.save(merged)  # sets dirty flag -> controller hot-reloads
                log.info("Shadow desired.config applied (%d top-level keys)", len(cfg_patch))
                reported_back["config"] = merged

        # Always echo status so desired.status clears in the shadow.
        self.report_status()
        if reported_back:
            self._publish_reported(reported_back)

    def _handle_command(self, msg):
        """Real-time calibration command (online-only). Ack on resp_topic."""
        req_id = msg.get("req_id")
        action = msg.get("action")
        params = msg.get("params", msg)
        try:
            result = calibrate_ops.dispatch(action, params)
            self._publish_topic(self.resp_topic,
                                {"req_id": req_id, "action": action, "ok": True, "result": result})
        except calibrate_ops.CalibrationError as e:
            self._publish_topic(self.resp_topic,
                                {"req_id": req_id, "action": action, "ok": False, "error": str(e)})
        except Exception as e:
            log.warning("Command %s failed: %s", action, e)
            self._publish_topic(self.resp_topic,
                                {"req_id": req_id, "action": action, "ok": False, "error": str(e)})
        # calibration flag may have changed — reflect it.
        self.report_status()

    # -------------------------------------------------------------- shadow out
    def _status_snapshot(self):
        return {
            "water_enabled": state.water_enabled,
            "switch_enabled": state.switch_enabled,
            "armed": state.armed,
            "mode": state.mode,
            "kids_mode": state.kids_mode,
            "calibrating": state.calibrating,
            "last_detection": state.last_detection,
            "last_spray": state.last_spray,
        }

    def report_status(self):
        self._publish_reported({"status": self._status_snapshot()})

    def report_config(self, cfg):
        self._publish_reported({"config": cfg, "calibration": calibration.load()})

    def report_telemetry(self, telemetry):
        self._publish_reported({"telemetry": {**telemetry, "ts": _now()}})

    def _publish_reported(self, partial):
        self._publish_topic(f"{self._shadow}/update", {"state": {"reported": partial}})

    def _publish_topic(self, topic, obj, qos=0):
        if not self._client:
            return
        try:
            self._client.publish(topic, json.dumps(obj, default=str), qos=qos)
        except Exception as e:
            log.debug("publish to %s failed: %s", topic, e)

    def _heartbeat_loop(self):
        while not state.exit_flag:
            if self._connected:
                self._publish_reported({"online": True, "heartbeat": _now(),
                                        "status": self._status_snapshot()})
            time.sleep(self.heartbeat_s)

    # ---------------------------------------------------------------- live stream
    def publish_stream_frame(self, jpeg_bytes):
        """Publish a JPEG frame to the stream topic (base64) while a viewer is present.
        Reuses the existing MQTT connection — lower latency than S3 round-trips and no
        S3 write churn. AWS IoT max payload is 128KB; a q50 640x480 frame is ~30-50KB."""
        if not (self._connected and jpeg_bytes):
            return
        if len(jpeg_bytes) > 120_000:
            return  # too big for one IoT message; SPA can fall back to S3 snapshot
        import base64
        self._publish_topic(self.stream_topic,
                            {"ts": time.time(),
                             "jpeg_b64": base64.b64encode(jpeg_bytes).decode("ascii")})

    # ------------------------------------------------------------------- S3 upload
    def _get_s3(self):
        """Return a boto3 S3 client authenticated via the IoT credentials provider
        (role alias) so the Pi uses its X.509 cert for S3 too — no static IAM keys.
        Falls back to the default boto3 credential chain if no creds endpoint set."""
        try:
            import boto3
        except Exception as e:
            log.debug("boto3 unavailable: %s", e)
            return None

        if not self.creds_endpoint or not self.role_alias:
            # Fallback: default chain (e.g. ~/.aws profile). Cached.
            if self._s3 is None:
                self._s3 = boto3.client("s3", region_name=self.region)
            return self._s3

        # IoT credentials provider: GET temp creds with the client cert.
        if self._creds and time.time() < self._creds_exp - 120:
            return self._s3
        try:
            import requests
            url = f"https://{self.creds_endpoint}/role-aliases/{self.role_alias}/credentials"
            r = requests.get(url, cert=(self.cert_path, self.key_path),
                             verify=self.ca_path,
                             headers={"x-amzn-iot-thingname": self.thing}, timeout=10)
            r.raise_for_status()
            c = r.json()["credentials"]
            self._creds = c
            # expiration is ISO8601; refresh a bit early regardless.
            self._creds_exp = time.time() + 1800
            self._s3 = boto3.client(
                "s3", region_name=self.region,
                aws_access_key_id=c["accessKeyId"],
                aws_secret_access_key=c["secretAccessKey"],
                aws_session_token=c["sessionToken"])
            return self._s3
        except Exception as e:
            log.warning("IoT credentials provider failed: %s", e)
            return None

    def upload_image(self, local_path, meta=None):
        """Upload a detection image to the S3 gallery bucket. Best-effort."""
        if not self.images_bucket or not os.path.exists(local_path):
            return None
        s3 = self._get_s3()
        if s3 is None:
            return None
        ts = time.gmtime()
        key = "images/%04d/%02d/%02d/%s" % (ts.tm_year, ts.tm_mon, ts.tm_mday,
                                            os.path.basename(local_path))
        try:
            extra = {"ContentType": "image/jpeg"}
            if meta:
                extra["Metadata"] = {k: str(v) for k, v in meta.items()}
            s3.upload_file(local_path, self.images_bucket, key, ExtraArgs=extra)
            log.info("Uploaded detection image to s3://%s/%s", self.images_bucket, key)
            return key
        except Exception as e:
            log.warning("S3 image upload failed: %s", e)
            return None


def _abspath(p):
    if os.path.isabs(p):
        return p
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base, p)
