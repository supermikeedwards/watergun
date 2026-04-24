# Raspberry Pi deployment

## 1. Static IP (recommended method: router DHCP reservation)
Easiest: log into your router and assign a reserved IP to the Pi's MAC address. No changes on the Pi needed.

On the Pi, get the MAC with:
```bash
ip link show wlan0 | awk '/ether/ {print $2}'
```

Alternative (on the Pi, classic Raspberry Pi OS using `dhcpcd`):
```bash
sudo nano /etc/dhcpcd.conf
```
Append:
```
interface wlan0
static ip_address=192.168.1.50/24
static routers=192.168.1.1
static domain_name_servers=192.168.1.1 1.1.1.1
```
Then `sudo systemctl restart dhcpcd`.

## 2. Pull the code
First time:
```bash
cd ~
git clone https://github.com/supermikeedwards/watergun.git
mv watergun-repo watergun 2>/dev/null || true   # folder name for systemd unit
cd watergun
```
*(If you cloned into a different directory name, adjust `WorkingDirectory=` in `systemd/watergun.service`.)*

Subsequent updates:
```bash
cd ~/watergun
git pull
sudo systemctl restart watergun
```

## 3. Install dependencies
```bash
sudo apt update
sudo apt install -y python3-pip python3-opencv tzdata
pip3 install -r requirements.txt
```

## 4. Enable at boot (systemd)
```bash
sudo cp systemd/watergun.service /etc/systemd/system/watergun.service
sudo systemctl daemon-reload
sudo systemctl enable watergun
sudo systemctl start watergun
```

Check status / logs:
```bash
sudo systemctl status watergun
journalctl -u watergun -f
```

## 5. Confirm NTP (time sync)
Raspberry Pi OS runs `systemd-timesyncd` by default — nothing to install.
```bash
timedatectl status        # should show "System clock synchronized: yes" and "NTP service: active"
sudo timedatectl set-timezone Europe/Paris
```

## 6. Open the web UI
On any phone/laptop on the same Wi-Fi: `http://<pi-ip>:8080`

Tabs:
- **Logs** — live tail, auto-refreshes every 2 s
- **Control** — Start/Stop button (water only; detection continues). Matches the physical switch.
- **Config** — edit any value in `config.json`. Save to persist; most values take effect immediately. Camera/hardware/port changes require `sudo systemctl restart watergun`.

## 7. Physical switch
Wire between GPIO27 and GND (internal pull-up enabled). Press toggles water on/off, exactly like the web UI Stop button.

## 8. Calibrating servos
The `servo_calibration.txt` file is created on first run with defaults. Edit directly on the Pi if needed — the file format is `KEY=value` per line (e.g. `SERVO_X_CENTER=90.0`).

## 9. Power / energy notes
- Outside opening hours the process sleeps in 5-minute chunks (configurable). Camera frames aren't captured during sleep.
- JPEG quality is 80 to keep image writes fast and small (~30–60 KB).
- Old images are auto-deleted after 15 days (configurable).
- Running headless (no X / no `cv2.imshow`) saves noticeable CPU on a Pi 3.
