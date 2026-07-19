#!/usr/bin/env python3
"""OAK-D-POE bring-up / hello-world test.

Purpose: verify a (second-hand) Luxonis OAK-D-POE actually works, BEFORE we depend
on the YOLO .blob or the full watergun pipeline. This is step A1 in worklog §14.

It does three escalating things, each printing PASS/FAIL:

  1. DISCOVER  - list every OAK depthai can see on the network/USB. Confirms the
                 PoE link is up and the device booted.
  2. CONNECT   - open a device session and read its identity: MXID, product/device
                 name, connection protocol+speed, bootloader/firmware, and the list
                 of CONNECTED CAMERA SENSORS (the key check for a used unit - all
                 three sensors should enumerate: RGB IMX378 + 2x stereo mono).
  3. STREAM    - build a minimal ColorCamera->XLinkOut pipeline, pull ~30 RGB
                 frames, report the measured FPS + frame shape, and save one frame
                 to /tmp/oak_test.jpg so you can eyeball the actual image.

No .blob, no neural network, no servos - just "is this camera alive and sane".

Usage on the Pi (OAK on the dedicated PoE link):
    python3 tools/oak_hello.py                 # uses --ip default 192.168.10.2
    python3 tools/oak_hello.py --ip 192.168.10.2
    python3 tools/oak_hello.py --discover-only  # just step 1
    python3 tools/oak_hello.py --no-ip          # auto-discovery instead of fixed IP

Exit code 0 = all attempted stages passed; non-zero = something failed.
"""
import argparse
import os
import sys
import tempfile
import time

OUT_JPG = os.path.join(tempfile.gettempdir(), "oak_test.jpg")


def _hr(title):
    print("\n" + "=" * 60)
    print(title)
    print("=" * 60)


def discover():
    import depthai as dai
    _hr("STAGE 1: DISCOVER")
    devices = dai.Device.getAllAvailableDevices()
    if not devices:
        print("FAIL: no OAK devices found by depthai.")
        print("  - PoE: is the injector powered? Pi eth0 up with 192.168.10.1/24?")
        print("  - Try: ping 192.168.10.2   (the OAK's static IP)")
        return False, None
    print(f"PASS: {len(devices)} device(s) found:")
    first_ip = None
    for d in devices:
        # DeviceInfo fields vary across depthai versions; print defensively.
        name = getattr(d, "name", "?")
        mxid = getattr(d, "mxid", getattr(d, "getMxId", lambda: "?"))
        if callable(mxid):
            try:
                mxid = mxid()
            except Exception:
                mxid = "?"
        state = getattr(d, "state", "?")
        print(f"  - name/ip={name}  mxid={mxid}  state={state}")
        if first_ip is None and isinstance(name, str) and name.count(".") == 3:
            first_ip = name
    return True, first_ip


def connect_and_stream(ip, use_ip, n_frames):
    """Open ONE device session (with the streaming pipeline), read identity, then
    pull frames. Doing it in a single session avoids the PoE reboot-on-close that
    breaks an immediate reconnect - especially on old/flaky bootloaders."""
    import depthai as dai
    import cv2
    _hr("STAGE 2+3: CONNECT + IDENTITY + STREAM")

    pipeline = dai.Pipeline()
    cam = pipeline.create(dai.node.ColorCamera)
    cam.setPreviewSize(640, 480)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam.setFps(30)
    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("rgb")
    cam.preview.link(xout.input)

    try:
        if use_ip and ip:
            print(f"Connecting by IP: {ip}")
            dev = dai.Device(pipeline, dai.DeviceInfo(ip))
        else:
            print("Connecting by auto-discovery")
            dev = dai.Device(pipeline)
    except Exception as e:
        print(f"FAIL: could not open device session: {e}")
        return False, False

    ok_connect = False
    ok_stream = False
    with dev:
        # --- identity ---
        try:
            print(f"  MXID:            {dev.getMxId()}")
        except Exception as e:
            print(f"  MXID:            (unavailable: {e})")
        for attr in ("getDeviceName", "getProductName"):
            fn = getattr(dev, attr, None)
            if fn:
                try:
                    print(f"  {attr[3:]:15} {fn()}")
                except Exception:
                    pass
        try:
            cams = dev.getConnectedCameraFeatures()
            print(f"  Connected camera sensors: {len(cams)}")
            for c in cams:
                socket = getattr(c, "socket", "?")
                sensor = getattr(c, "sensorName", "?")
                w = getattr(c, "width", "?")
                h = getattr(c, "height", "?")
                print(f"    - socket={socket} sensor={sensor} {w}x{h}")
        except Exception as e:
            print(f"  (could not list cameras: {e})")
        ok_connect = True
        print("  -> identity read OK")

        # --- stream ---
        q = dev.getOutputQueue("rgb", maxSize=4, blocking=False)
        got = 0
        last_shape = None
        t0 = time.time()
        deadline = t0 + 15
        saved = False
        while got < n_frames and time.time() < deadline:
            pkt = q.tryGet()
            if pkt is None:
                time.sleep(0.005)
                continue
            frame = pkt.getCvFrame()
            last_shape = frame.shape
            got += 1
            if not saved:
                cv2.imwrite(OUT_JPG, frame)
                saved = True
        dt = time.time() - t0
        if got == 0:
            print("  FAIL: no frames arrived within timeout.")
        else:
            fps = got / dt if dt > 0 else 0
            print(f"  PASS: received {got} frames in {dt:.1f}s (~{fps:.1f} FPS), shape={last_shape}")
            print(f"  Saved first frame to {OUT_JPG}")
            print("  Open that file to confirm the image looks sane.")
            ok_stream = True
    return ok_connect, ok_stream


def main():
    ap = argparse.ArgumentParser(description="OAK-D-POE bring-up test")
    ap.add_argument("--ip", default="192.168.10.2", help="OAK static IP on the PoE link")
    ap.add_argument("--no-ip", action="store_true", help="use auto-discovery instead of --ip")
    ap.add_argument("--discover-only", action="store_true", help="only run stage 1")
    ap.add_argument("--frames", type=int, default=30, help="frames to grab in stage 3")
    args = ap.parse_args()

    try:
        import depthai  # noqa: F401
    except Exception as e:
        print(f"FATAL: cannot import depthai: {e}")
        print("Install on the Pi with:  pip3 install --user depthai")
        return 2

    use_ip = not args.no_ip
    ok_discover, found_ip = discover()
    if args.discover_only:
        return 0 if ok_discover else 1

    # If discovery found a POE IP and the user didn't override, prefer it.
    ip = args.ip
    if use_ip and found_ip and args.ip == "192.168.10.2" and found_ip != "192.168.10.2":
        print(f"\n(Note: discovery saw {found_ip}; using that instead of default.)")
        ip = found_ip

    ok_connect, ok_stream = connect_and_stream(ip, use_ip, args.frames)

    _hr("SUMMARY")
    print(f"  discover: {'PASS' if ok_discover else 'FAIL'}")
    print(f"  connect:  {'PASS' if ok_connect else 'FAIL'}")
    print(f"  stream:   {'PASS' if ok_stream else 'FAIL'}")
    all_ok = ok_discover and ok_connect and ok_stream
    print("\nRESULT:", "ALL PASS - the OAK is alive and producing images." if all_ok
          else "SOMETHING FAILED - see stages above.")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
