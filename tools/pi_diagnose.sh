#!/usr/bin/env bash
# pi_diagnose.sh — one-shot diagnostic for the watergun Pi deployment.
#
# Run this from the watergun checkout directory on the Pi:
#     cd /home/pi/water_gun
#     bash tools/pi_diagnose.sh
#
# Output goes to BOTH the screen and `pi_diagnose.out` in the current directory.
# Paste `pi_diagnose.out` back to me next session.
#
# Safe to run repeatedly. Read-only — does NOT modify anything on the system.

set -u  # error on undefined vars; do NOT set -e (we want to keep going past failures)

OUT_FILE="pi_diagnose.out"
exec > >(tee "$OUT_FILE") 2>&1

section() { printf "\n=== %s ===\n" "$1"; }
run() {     printf "\n$ %s\n" "$*"; "$@" 2>&1 || true; }

printf "watergun Pi diagnostic — %s\n" "$(date -Is)"
printf "Working directory: %s\n" "$(pwd)"

section "1. System info"
run uname -a
run cat /etc/os-release
run python3 --version
run pip3 --version

section "2. Repo state"
run git -C . log --oneline -3
run git -C . status -sb
run git -C . rev-parse HEAD

section "3. numpy: which one is Python actually using?"
# Note: stdout of python3 -c is captured even when it raises.
run python3 - <<'PY'
import sys, traceback
try:
    import numpy
    print("numpy version :", numpy.__version__)
    print("numpy path    :", numpy.__file__)
    print("(should be under /usr/lib/python3/dist-packages NOT /home/pi/.local)")
except Exception:
    print("numpy IMPORT FAILED:")
    traceback.print_exc()
PY

section "4. libopenblas check (system numpy depends on it)"
run dpkg -l libopenblas0-pthread libopenblas-base 2>/dev/null
run ldconfig -p | grep -i openblas

section "5. opencv: which one?"
run python3 - <<'PY'
import sys, traceback
try:
    import cv2
    print("cv2 version :", cv2.__version__)
    print("cv2 path    :", cv2.__file__)
    print("(should be under /usr/lib/python3/dist-packages — apt-installed python3-opencv)")
except Exception:
    print("cv2 IMPORT FAILED:")
    traceback.print_exc()
PY

section "6. tflite_runtime version + path"
run python3 - <<'PY'
import sys, traceback
try:
    import tflite_runtime
    import tflite_runtime.interpreter as tflite
    print("tflite_runtime version :", tflite_runtime.__version__)
    print("tflite_runtime path    :", tflite_runtime.__file__)
except Exception:
    print("tflite_runtime IMPORT FAILED:")
    traceback.print_exc()
PY
run pip3 show tflite_runtime

section "7. ~/.local/lib/python3.*/site-packages — what did pip --user install?"
# These are the packages that can shadow system ones. Anything dangerous here
# (numpy, cv2/opencv, picamera) means we have a conflict.
run sh -c 'ls ~/.local/lib/python3*/site-packages/ 2>/dev/null'

section "8. Model file integrity"
MODEL=models/ssdlite_mobiledet_coco_qat_postprocess.tflite
EXPECTED_SHA256=32c486140391eb4dc43fca7113ad392be632dc5366687f2731f73d740678693f
EXPECTED_SIZE_APPROX_BYTES=4374888  # ~4.3 MB; allow ±5%
run ls -la "$MODEL"
run file "$MODEL"
if [ -f "$MODEL" ]; then
    ACTUAL_SHA=$(sha256sum "$MODEL" | awk '{print $1}')
    ACTUAL_SIZE=$(stat -c%s "$MODEL")
    printf "\nactual   sha256 : %s\n" "$ACTUAL_SHA"
    printf "expected sha256 : %s\n" "$EXPECTED_SHA256"
    if [ "$ACTUAL_SHA" = "$EXPECTED_SHA256" ]; then
        printf "VERDICT         : SHA256 MATCH ✓\n"
    else
        printf "VERDICT         : SHA256 MISMATCH ✗ — file is corrupt or modified.\n"
    fi
    printf "actual   size   : %s bytes\n" "$ACTUAL_SIZE"
    printf "expected size   : ~%s bytes\n" "$EXPECTED_SIZE_APPROX_BYTES"
    # Magic bytes at offset 4 should be TFL3
    run python3 -c "import sys; d=open('$MODEL','rb').read(); print('magic@4:', d[4:8])"
else
    printf "MODEL FILE MISSING — git pull did not produce the file.\n"
fi

section "9. End-to-end TFLite load test (the actual failing step)"
run python3 - <<'PY'
import traceback
print("Step A: import tflite_runtime")
try:
    import tflite_runtime.interpreter as tflite
    print("  OK")
except Exception:
    traceback.print_exc(); raise SystemExit(0)

print("Step B: import numpy")
try:
    import numpy as np
    print("  OK — version:", np.__version__)
except Exception:
    traceback.print_exc(); raise SystemExit(0)

print("Step C: instantiate Interpreter from model file")
try:
    i = tflite.Interpreter("models/ssdlite_mobiledet_coco_qat_postprocess.tflite")
    print("  OK")
except Exception:
    traceback.print_exc(); raise SystemExit(0)

print("Step D: allocate_tensors()")
try:
    i.allocate_tensors()
    print("  OK")
    print("  input details :", i.get_input_details()[0]['shape'], i.get_input_details()[0]['dtype'])
    print("  output count  :", len(i.get_output_details()))
except Exception:
    traceback.print_exc(); raise SystemExit(0)

print("ALL FOUR STEPS PASSED — tflite + model file are healthy.")
PY

section "10. Last 60 lines of journal for the watergun service"
run sudo -n journalctl -u watergun -n 60 --no-pager 2>/dev/null

section "Done"
printf "\nFull output saved to: %s\n" "$OUT_FILE"
printf "Paste that file back next session.\n"
