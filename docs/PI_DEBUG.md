# Pi deployment debugging guide

Use this when `journalctl -u watergun -f` shows the service crashing on startup,
or `python3 run.py` crashes with a Python traceback.

This guide is structured around the specific failure modes we have already hit
on the Pi 3 / Bullseye build, so it's biased toward those — start at §1 and
follow the branches.

---

## §1. Run the one-shot diagnostic

```bash
cd /home/pi/water_gun
bash tools/pi_diagnose.sh
```

This produces `pi_diagnose.out` in the current directory and also prints to
the screen. Read the output, find the section(s) with red flags, then jump to
the matching fix below.

If you can't read the verdict from the screen, the file is the source of
truth — paste it back in chat next session.

What "red flags" look like, by section:

| § | Red flag                                                            | Jump to        |
|---|---------------------------------------------------------------------|----------------|
| 3 | numpy path is under `/home/pi/.local/...`                           | §2 numpy       |
| 3 | numpy import fails with `libopenblas.so.0: cannot open …`           | §3 openblas    |
| 5 | cv2 path is under `/home/pi/.local/...`                             | §4 cv2         |
| 5 | cv2 import fails or crashes Python                                  | §4 cv2         |
| 6 | tflite_runtime import fails                                         | §5 tflite      |
| 7 | `~/.local/.../site-packages/` lists `numpy`, `cv2`, `opencv*`,      | §6 nuke .local |
|   | `picamera`, `RPi`, etc. (any C-extension package)                   |                |
| 8 | `SHA256 MISMATCH` or `MODEL FILE MISSING`                           | §7 model       |
| 9 | Step A (import tflite_runtime) fails                                | §5 tflite      |
| 9 | Step B fails (numpy)                                                | §2 numpy       |
| 9 | Step C fails (`Interpreter(...)`) with `result with an error set`   | §8 ABI clash   |
| 9 | Step C fails with `could not open file` / file-system error         | §7 model       |
| 9 | All four steps PASS                                                 | §9 elsewhere   |

---

## §2. numpy is in `~/.local` instead of `/usr/lib/python3/dist-packages`

Cause: `pip3 install --user -r requirements.txt` installed a pip-version of
numpy that shadows the apt-installed `python3-numpy`. The pip version is
either broken (partial install / wrong arch) or ABI-incompatible with
`python3-opencv` and `tflite_runtime`.

Fix:
```bash
pip3 uninstall -y numpy
# If pip says "not installed" but the directory still exists, force-remove:
rm -rf ~/.local/lib/python3*/site-packages/numpy*
# Verify:
python3 -c "import numpy; print(numpy.__version__, numpy.__file__)"
# expected: 1.19.x or similar, path under /usr/lib/python3/dist-packages
```

Then re-run the diagnostic and continue down the list.

---

## §3. `libopenblas.so.0: cannot open shared object file`

System numpy is dynamically linked to libopenblas. On a fresh Bullseye, or
after `apt autoremove`, the library may be missing.

Fix:
```bash
sudo apt update
sudo apt install -y libopenblas0-pthread
# fallback if the above package isn't found on this Pi:
sudo apt install -y libopenblas-base
# Verify numpy imports:
python3 -c "import numpy; print(numpy.__version__)"
```

---

## §4. cv2 in `~/.local` (or import crash)

Same problem as numpy, different package. `requirements.txt` lists
`opencv-python-headless`, which is fine on a laptop but on the Pi 3 it
shadows the apt `python3-opencv` we already installed in §3 of the original
deploy.

Fix:
```bash
pip3 uninstall -y opencv-python opencv-python-headless opencv-contrib-python
rm -rf ~/.local/lib/python3*/site-packages/cv2*
rm -rf ~/.local/lib/python3*/site-packages/opencv*
# Verify the system one is back:
python3 -c "import cv2; print(cv2.__version__, cv2.__file__)"
# expected path under /usr/lib/python3/dist-packages
```

---

## §5. tflite_runtime fails to import

If `pi_diagnose.sh` §6 shows tflite_runtime missing, or the import raises:

a) Check `pip3 show tflite_runtime` lists a version. If not installed:
```bash
pip3 install --user tflite-runtime
```

b) If installed but version is **newer than 2.13** AND `uname -m` is
   `armv7l`: pip may have grabbed an aarch64-only wheel. Pin to a known-good
   armv7l version:
```bash
pip3 uninstall -y tflite-runtime
pip3 install --user "tflite-runtime==2.13.0"
```

c) If pip can't find an armv7l wheel for `tflite-runtime` at all (rare),
   fall back to the legacy mode while we figure out a better path:

   In the web UI Config tab → set `bird_detection.legacy_motion_only` to
   `true` → Save → `sudo systemctl restart watergun`.

   This restores the old motion-only detector. We then look at
   alternatives next session (Path B, Coral USB, or a TF Lite from source
   build).

---

## §6. Nuclear option — wipe `~/.local` Python packages

If §7 of the diagnostic shows multiple shadowed system packages
(numpy, cv2, picamera, RPi.GPIO, etc.), the cleanest fix is to wipe
`~/.local` entirely and re-install only the things the system genuinely
doesn't provide.

```bash
# nukes everything pip --user has ever installed; safe — you can always reinstall.
rm -rf ~/.local/lib/python3*/site-packages
rm -rf ~/.local/lib/python3*/dist-packages 2>/dev/null

# now install ONLY the things that aren't in apt:
pip3 install --user adafruit-circuitpython-servokit imutils Flask "tflite-runtime==2.13.0"

# verify everything imports:
python3 -c "import cv2, flask, numpy, picamera, RPi.GPIO; print('system imports OK')"
python3 -c "import tflite_runtime.interpreter as t; print('tflite OK')"
python3 -c "from adafruit_servokit import ServoKit; print('servokit OK')"
```

Note: this skips re-installing numpy + opencv via pip — they come from apt.

---

## §7. Model file missing or SHA256 mismatch

Expected SHA256: `32c486140391eb4dc43fca7113ad392be632dc5366687f2731f73d740678693f`
Expected size: ~4.3 MB (4,374,888 bytes ± a few hundred).

Quick fix — re-download just the model file directly from the upstream
source (same URL we used to seed the repo):

```bash
cd /home/pi/water_gun
curl -L --fail -o models/ssdlite_mobiledet_coco_qat_postprocess.tflite \
  https://github.com/google-coral/test_data/raw/master/ssdlite_mobiledet_coco_qat_postprocess.tflite
sha256sum models/ssdlite_mobiledet_coco_qat_postprocess.tflite
# should match the expected SHA above.
```

If that succeeds but `git status` now shows the file as modified — that's
fine, leave it; the file matches origin. Or alternatively reset:
```bash
git checkout -- models/ssdlite_mobiledet_coco_qat_postprocess.tflite
sha256sum models/ssdlite_mobiledet_coco_qat_postprocess.tflite
```

If the SHA still mismatches even straight out of git, the file in the
repo has a problem on our side — let me know and I'll re-commit it.

---

## §8. tflite Interpreter() fails with "result with an error set"

This is the symptom we hit on 2026-05-17. The model file loads but the
TFLite C extension and numpy are at incompatible ABIs.

Most likely scenario: tflite-runtime (compiled against numpy 1.21+) is
calling into our system numpy 1.19.x. The C call returns a numpy array
that the wrapper doesn't recognise → undefined Python error state.

Try in this order:

a) **Make sure §2 + §6 are clean first** (no user-pip numpy or cv2 left
   over). 90% of the time this fixes step C.

b) Pin tflite-runtime to a version known to work with numpy 1.19:
```bash
pip3 install --user "tflite-runtime==2.13.0"
```
   2.13 was the last release with full armv7l + numpy 1.19 support.

c) If that still fails, install a *newer* numpy via pip into `--user`
   (yes, contradicting §2) — but constrained to a version range that
   keeps cv2 working. Bullseye `python3-opencv` 4.5.x is built against
   numpy 1.19 but tolerates 1.21 at runtime in our experience:
```bash
pip3 install --user "numpy>=1.21,<1.24"
python3 -c "import cv2, numpy as np; print('cv2 still loads with numpy', np.__version__)"
```
   If cv2 import now crashes, undo with `pip3 uninstall -y numpy`.

d) Last resort — switch the detector to `legacy_motion_only=true` (web
   UI Config tab) and we re-evaluate next session. The motion-only
   pipeline doesn't load TFLite at all, so it sidesteps this problem
   entirely while we sort it out.

---

## §9. All four diagnostic steps pass but the service still crashes

Less likely, but possible. The crash is somewhere outside the
detector init — maybe camera, GPIO, or the web server.

```bash
cd /home/pi/water_gun
python3 run.py
```

This runs the whole thing in the foreground. The traceback will name a
specific file and line. Common culprits in this case:

- **Camera in use by another process.** Reboot the Pi, try again.
- **GPIO already exported.** `sudo systemctl stop watergun` then run
  `python3 run.py` directly. If still failing: reboot.
- **Permission errors on `motion_images/` or `logs/`.** `chown -R pi:pi
  /home/pi/water_gun`.

If the traceback mentions a watergun source file we wrote — paste it
back next session and we'll fix it.

---

## After any fix — re-run

1. `bash tools/pi_diagnose.sh` — confirm the failing section now passes.
2. `sudo systemctl restart watergun`
3. `journalctl -u watergun -n 50 --no-pager` — confirm no crash on startup.
4. `journalctl -u watergun -f` — watch live for a minute.

Look for these "service is healthy" lines:
```
BirdDetector ready: backend=tflite_runtime input=320x320 dtype=...
Switch watcher started: initial switch_enabled=...
Camera AE re-locked: {shutter_us: ..., ...}
Telemetry: {'temp': "...'C", ...}    (every 60s)
```

---

## Need help? Send these back next session

If nothing in this guide worked, share next session:

1. `pi_diagnose.out` (the full file written by the script).
2. The output of `journalctl -u watergun -n 100 --no-pager`.
3. The output of `python3 run.py` running directly (foreground crash —
   redirect with `python3 run.py 2>&1 | tee run_crash.out`).

That's enough for me to diagnose just about any remaining failure.
