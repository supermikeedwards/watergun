#!/usr/bin/env bash
# Fresh Raspberry Pi setup for the Pigeon Watergun (current architecture:
# OAK-D-POE camera/inference + AWS IoT cloud client + LAN Flask fallback).
#
# Run ON the Pi, from the repo root, AFTER:
#   git clone https://github.com/supermikeedwards/watergun.git ~/watergun && cd ~/watergun
#   (and after copying the AWS device certs into ./certs/  -- see docs/PI_SETUP.md)
#
# Idempotent-ish: safe to re-run. Does NOT configure the eth0<->OAK static IP
# (that's OS-version specific -- see docs/PI_SETUP.md).
set -euo pipefail
cd "$(cd "$(dirname "$0")/.." && pwd)"   # repo root
REPO="$(pwd)"
echo "== Pigeon Watergun Pi setup =="
echo "repo: $REPO"

# 1. System libraries that the pip wheels commonly need on Raspberry Pi OS.
echo "== apt dependencies =="
sudo apt-get update
sudo apt-get install -y python3-pip git \
  libatlas-base-dev libopenblas0 libopenjp2-7 libtiff6 2>/dev/null \
  || sudo apt-get install -y python3-pip git libatlas-base-dev libopenblas0 libopenjp2-7 libtiff5

# 2. Python deps (piwheels provides prebuilt ARM wheels on Raspberry Pi OS).
echo "== pip install -r requirements.txt (--user) =="
pip3 install --user --upgrade -r requirements.txt

# 3. depthai USB/PoE device rules (harmless if already present).
echo "== depthai udev rules =="
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/80-movidius.rules >/dev/null
sudo udevadm control --reload-rules && sudo udevadm trigger || true

# 4. Certs check.
echo "== AWS device certs =="
missing=0
for f in certs/AmazonRootCA1.pem certs/watergun-pi.cert.pem certs/watergun-pi.private.key; do
  if [ ! -f "$f" ]; then echo "  MISSING: $f"; missing=1; else echo "  ok: $f"; fi
done
if [ "$missing" = "1" ]; then
  echo "  -> Copy the certs/ dir from your laptop (produced by infra/provision-device-cert.sh)."
  echo "     Cloud will stay disabled until the certs are present."
fi

# 5. Enable the cloud client in config.json (endpoints are pre-filled in the repo).
if [ "$missing" = "0" ]; then
  echo "== enabling cloud in config.json =="
  python3 - <<'PY'
import json
c = json.load(open("config.json"))
c["cloud"]["enabled"] = True
json.dump(c, open("config.json", "w"), indent=2)
print("  cloud.enabled = True")
PY
fi

# 6. Model blob check (OAK detector needs it).
echo "== OAK model blob =="
BLOB=$(python3 -c "import json;print(json.load(open('config.json'))['oak']['model_blob'])")
if [ -f "$BLOB" ]; then echo "  ok: $BLOB"; else
  echo "  MISSING: $BLOB"
  echo "  -> fetch it (see docs/PI_SETUP.md 'Model blob'); detection won't run without it,"
  echo "     but the cloud client + web UI + servos will still work."
fi

# 7. systemd service.
echo "== systemd service =="
UNIT=/etc/systemd/system/watergun.service
sudo tee "$UNIT" >/dev/null <<EOF
[Unit]
Description=Pigeon Watergun
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$REPO
ExecStart=/usr/bin/python3 $REPO/run.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable watergun
echo ""
echo "== Done =="
echo "Start it with:   sudo systemctl start watergun"
echo "Watch logs:      journalctl -u watergun -f"
echo "The Pi should appear ONLINE at https://birdblast.the-edwards.fr within ~20s."
