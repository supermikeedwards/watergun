# Fresh Pi setup — Pigeon Watergun (OAK-D-POE + AWS IoT)

Bring a **brand-new, blank Raspberry Pi** up to a working watergun that appears on
**https://birdblast.the-edwards.fr**. This reflects the current architecture (worklog §18):
the Pi is a thin controller — it drives the servos + solenoid, talks to the **OAK-D-POE**
for camera/detection, and connects **outbound** to **AWS IoT** (no inbound ports). The
birdblast web UI is served from AWS, so it works even when the Pi is off.

> The old PiCamera / motion-detection / port-forwarding instructions are gone — superseded
> by the OAK (§12) and the AWS-offload (§18).

---

## 0. What you need
- Raspberry Pi (Pi 3 or newer) + **Raspberry Pi OS** (Lite is fine — headless).
- The **OAK-D-POE** + TP-Link PoE injector (see §12/§17) on a dedicated Ethernet link to the Pi.
- Servos (PCA9685), solenoid relay (GPIO17), arming switch (GPIO27) wired as before.
- The **AWS device certs** (produced once on your laptop — see step 3).

## 1. OS + network basics
1. Flash Raspberry Pi OS, enable SSH, join Wi-Fi (for internet/AWS).
2. `sudo raspi-config` → set timezone **Europe/Paris**, enable **I2C** (for the PCA9685 servo board).
3. Confirm NTP: `timedatectl status` → `System clock synchronized: yes`.

## 2. Get the code
```bash
git clone https://github.com/supermikeedwards/watergun.git ~/watergun
cd ~/watergun
```

## 3. AWS device certificate (once, from your laptop — NOT on the Pi)
The cert is the Pi's identity for AWS IoT. Mint it on a machine with the `skimr` AWS profile:
```bash
cd infra && ./provision-device-cert.sh      # writes certs/ (gitignored)
```
Then copy the whole `certs/` folder onto the Pi:
```bash
scp -r certs pi@<pi-ip>:~/watergun/          # from the laptop
```
(The non-secret endpoints/bucket/role are already baked into `config.json`; the certs are
the only per-device secret.)

## 4. Run the setup script (on the Pi)
```bash
cd ~/watergun
bash scripts/setup_pi.sh
```
It installs apt libs + `pip install -r requirements.txt`, adds the depthai udev rule, checks
the certs, flips `cloud.enabled=true` (once certs are present), checks the model blob, and
installs + enables the `watergun` systemd service.

## 5. OAK network link (manual — OS-version specific)
The OAK is on a dedicated Ethernet/PoE link. Give the Pi a static address on that link so
`depthai` can reach the OAK at `192.168.10.2` (matches `config.json` → `oak.device_ip`):

- **Bullseye (dhcpcd):** add to `/etc/dhcpcd.conf`:
  ```
  interface eth0
  static ip_address=192.168.10.1/24
  ```
- **Bookworm (NetworkManager):**
  ```bash
  sudo nmcli con add type ethernet ifname eth0 con-name oak ipv4.method manual ipv4.addresses 192.168.10.1/24
  ```
Reboot the link / Pi. The OAK self-assigns on this link; if discovery is flaky, flash a static
IP onto the OAK (see worklog §17 backlog) or set `oak.device_ip` to what it reports.

## 6. Model blob (OAK detector) — ACTION NEEDED
`config.json` → `oak.model_blob` points at `models/yolov8n_coco_640x640.blob`, which is **not
yet in the repo**. Fetch it once (any machine with internet):
```bash
python3 tools/fetch_yolov8n_blob.py
```
> ⚠️ Known issue (2026-07-19): the Luxonis zoo artifact 404s under the old name. Verify the
> current YOLOv8n entry at https://github.com/luxonis/depthai-model-zoo (or convert an ONNX
> via `blobconverter.from_onnx`) and update `tools/fetch_yolov8n_blob.py` / `oak.model_blob`.
> **Without the blob, detection won't run — but the cloud client, web UI, servos, and
> calibration all still work**, so you can bring the Pi online first and sort the blob after.

## 7. Start it
```bash
sudo systemctl start watergun
journalctl -u watergun -f
```
Watch for:
- `Cloud client starting: endpoint=... thing=watergun-pi` then `Cloud connected + subscribed`.
- Within ~20s the status pill on **https://birdblast.the-edwards.fr** flips to **Online** and
  the **Calibrate** tab activates.
- `OakDetector ready ...` (only if the OAK + blob are present).

## 8. Verify end-to-end
- Log in to birdblast → **Status** pill shows **Online**.
- **Config** tab → change a value → Save → journal shows `Shadow desired.config applied` +
  `Config reloaded`.
- **Calibrate** tab → Enter calibration → tap the live view → the gun aims + fires.
- A detection saves a JPEG that appears in the **Images** tab (served from S3).

---

## Fallback / notes
- **LAN-only mode:** set `cloud.enabled=false` in `config.json` and the old Flask UI still
  serves on `http://<pi-ip>:8080` (Logs/Control/Config/Calibrate). Useful for on-site debug.
- **Debugging:** see `docs/PI_DEBUG.md` and `tools/pi_diagnose.sh`.
- **Cloud contract:** thing `watergun-pi`, classic Device Shadow (`status`/`config`/
  `telemetry`/`heartbeat`), topics `watergun/{cmd,cmd/resp,stream,viewer}`. Details in worklog §18.
