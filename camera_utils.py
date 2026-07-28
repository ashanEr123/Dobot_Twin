"""Open webcam with a reliable backend (DirectShow on Windows) and warm-up frames."""

import platform
import time
import cv2
import config


def open_camera(index=None, width=None, height=None, warmup_frames=15):
    """Returns (cap, backend_name). Check cap.isOpened() before use."""
    index = config.CAMERA_INDEX if index is None else index
    width = width or getattr(config, "CAMERA_WIDTH", 1920)
    height = height or getattr(config, "CAMERA_HEIGHT", 1080)
    target_fps = getattr(config, "CAMERA_FPS", 30)

    cap = None
    backend = "default"

    if platform.system() == "Windows":
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if cap.isOpened():
            backend = "DSHOW"
        else:
            cap.release()
            cap = None

    if cap is None:
        cap = cv2.VideoCapture(index)

    if not cap.isOpened():
        return cap, backend

    ok, _ = cap.read()
    if not ok:
        return cap, backend

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, target_fps)
    ok, _ = cap.read()
    if not ok:
        print(f"[camera_utils][warn] {width}x{height}@{target_fps}fps failed; "
              f"falling back to native settings.")
        cap.release()
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW) if backend == "DSHOW" \
            else cv2.VideoCapture(index)

    for _ in range(warmup_frames):
        cap.read()
        time.sleep(0.02)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[camera_utils] Camera {index}: backend={backend}  "
          f"{actual_w}x{actual_h}  fps={actual_fps:.1f}")

    return cap, backend
