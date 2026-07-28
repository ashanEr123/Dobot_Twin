"""ArUco tip tracking for placement error and path smoothness."""

import os
import sys
import json
import time
import numpy as np
import config
import camera_utils


def _get_aruco_dict():
    import cv2
    name = config.ARUCO_DICT
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def load_lens_calibration():
    """Load camera_matrix/dist_coeffs, or (None, None) if missing."""
    path = config.CALIBRATION_FILE
    if not os.path.exists(path):
        print(f"[camera_tracking][warn] No lens calibration at '{path}'.")
        return None, None

    with open(path) as f:
        data = json.load(f)

    camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.array(data["dist_coeffs"], dtype=np.float64)
    rms = data.get("rms_reprojection_error")
    if rms is not None and rms > config.RMS_REPROJECTION_WARNING_THRESHOLD:
        print(f"[camera_tracking][warn] RMS {rms:.3f}px -- consider recalibrating.")

    return camera_matrix, dist_coeffs


def _undistort(frame, camera_matrix, dist_coeffs):
    if camera_matrix is None:
        return frame
    import cv2
    return cv2.undistort(frame, camera_matrix, dist_coeffs)


def calibrate():
    """Click 4 grid corners, enter real-world mm, save homography."""
    import cv2

    camera_matrix, dist_coeffs = load_lens_calibration()

    cap, backend = camera_utils.open_camera()
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera {config.CAMERA_INDEX}.")
    print(f"[camera_tracking] Camera {config.CAMERA_INDEX} via {backend}.")
    clicked_px = []

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicked_px) < 4:
            clicked_px.append((x, y))
            print(f"Point {len(clicked_px)}: ({x}, {y})")

    cv2.namedWindow("calibrate")
    cv2.setMouseCallback("calibrate", on_click)

    print("Click 4 corners: TL, TR, BR, BL. Press 'q' when done.")

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        frame = _undistort(frame, camera_matrix, dist_coeffs)
        for pt in clicked_px:
            cv2.circle(frame, pt, 6, (0, 0, 255), -1)
        cv2.imshow("calibrate", frame)
        if cv2.waitKey(1) & 0xFF == ord("q") or len(clicked_px) == 4:
            break

    cap.release()
    cv2.destroyAllWindows()

    if len(clicked_px) != 4:
        print("Need exactly 4 points.")
        return

    print("\nEnter real-world (x, y) mm for each point, same order.")
    real_pts = []
    for i in range(4):
        x = float(input(f"Point {i+1} x (mm): "))
        y = float(input(f"Point {i+1} y (mm): "))
        real_pts.append((x, y))

    src = np.array(clicked_px, dtype=np.float32)
    dst = np.array(real_pts, dtype=np.float32)
    H, _ = cv2.findHomography(src, dst)

    np.save(config.WORKSPACE_CALIBRATION_FILE, H)
    print(f"Saved -> {config.WORKSPACE_CALIBRATION_FILE}")


def _load_homography():
    try:
        return np.load(config.WORKSPACE_CALIBRATION_FILE)
    except FileNotFoundError:
        raise RuntimeError(
            "No workspace calibration. Run: python camera_tracking.py calibrate"
        )


def pixel_to_mm(px, py, H):
    pt = np.array([[px, py]], dtype=np.float32).reshape(-1, 1, 2)
    import cv2
    out = cv2.perspectiveTransform(pt, H)
    return float(out[0, 0, 0]), float(out[0, 0, 1])


def _track_loop(cap, backend, camera_matrix, dist_coeffs, H, detector,
                 marker_id, should_continue, release_on_exit=True):
    path = []
    frames_read = 0
    frames_failed = 0
    frames_with_any_marker = 0
    frames_with_target_marker = 0
    ids_seen = set()
    t0 = time.time()

    while should_continue():
        ok, frame = cap.read()
        if not ok:
            frames_failed += 1
            continue
        frames_read += 1
        frame = _undistort(frame, camera_matrix, dist_coeffs)
        corners, ids, _ = detector.detectMarkers(frame)
        if ids is not None:
            frames_with_any_marker += 1
            ids_seen.update(int(i) for i in ids.flatten())
            for c, i in zip(corners, ids.flatten()):
                if i == marker_id:
                    frames_with_target_marker += 1
                    cx = float(c[0][:, 0].mean())
                    cy = float(c[0][:, 1].mean())
                    x_mm, y_mm = pixel_to_mm(cx, cy, H)
                    path.append((time.time() - t0, x_mm, y_mm))

    if release_on_exit:
        cap.release()

    print(f"[camera_tracking] backend={backend}  duration={time.time()-t0:.2f}s  "
          f"frames_read={frames_read}  failed={frames_failed}  "
          f"any_marker={frames_with_any_marker}  "
          f"target_id_{marker_id}={frames_with_target_marker}  "
          f"ids_seen={sorted(ids_seen) if ids_seen else 'none'}")

    if frames_read == 0:
        print("[camera_tracking][warn] No frames read -- check CAMERA_INDEX.")
    elif frames_with_any_marker == 0:
        print("[camera_tracking][warn] No ArUco detected. Check dict, lighting, focus.")
    elif frames_with_target_marker == 0:
        print(f"[camera_tracking][warn] Saw ids {sorted(ids_seen)}, "
              f"not END_EFFECTOR_MARKER_ID={marker_id}.")

    return path


def _setup_tracking(marker_id):
    import cv2
    marker_id = marker_id or config.END_EFFECTOR_MARKER_ID
    camera_matrix, dist_coeffs = load_lens_calibration()
    H = _load_homography()
    aruco_dict = _get_aruco_dict()
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    cap, backend = camera_utils.open_camera()
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera {config.CAMERA_INDEX}.")
    return cap, backend, camera_matrix, dist_coeffs, H, detector, marker_id


class TrackingSession:
    """Reuse one open camera across many trials."""

    def __init__(self, marker_id=None):
        (self.cap, self.backend, self.camera_matrix, self.dist_coeffs,
         self.H, self.detector, self.marker_id) = _setup_tracking(marker_id)
        print(f"[camera_tracking] TrackingSession open ({self.backend}).")

    def close(self):
        self.cap.release()
        print("[camera_tracking] TrackingSession closed.")


def track_marker_path(duration_s, marker_id=None):
    """Record (t, x_mm, y_mm) for a fixed duration. For tests only."""
    cap, backend, camera_matrix, dist_coeffs, H, detector, marker_id = \
        _setup_tracking(marker_id)
    t0 = time.time()
    return _track_loop(cap, backend, camera_matrix, dist_coeffs, H, detector,
                        marker_id, should_continue=lambda: time.time() - t0 < duration_s)


def track_marker_path_until(stop_event, marker_id=None, session=None):
    """Record path until stop_event is set. Use for real trials."""
    if session is not None:
        return _track_loop(session.cap, session.backend, session.camera_matrix,
                            session.dist_coeffs, session.H, session.detector,
                            marker_id or session.marker_id,
                            should_continue=lambda: not stop_event.is_set(),
                            release_on_exit=False)

    cap, backend, camera_matrix, dist_coeffs, H, detector, marker_id = \
        _setup_tracking(marker_id)
    return _track_loop(cap, backend, camera_matrix, dist_coeffs, H, detector,
                        marker_id, should_continue=lambda: not stop_event.is_set())


def compute_placement_error(final_xy_mm):
    """Distance (mm) from final tip position to target pose."""
    t = config.target_pose()
    return float(np.hypot(final_xy_mm[0] - t["x"], final_xy_mm[1] - t["y"]))


def compute_smoothness_index(path):
    """Higher is smoother: 1 / (1 + RMS jerk)."""
    if len(path) < 5:
        return float("nan")

    t = np.array([p[0] for p in path])
    x = np.array([p[1] for p in path])
    y = np.array([p[2] for p in path])

    vx, vy = np.gradient(x, t), np.gradient(y, t)
    ax, ay = np.gradient(vx, t), np.gradient(vy, t)
    jx, jy = np.gradient(ax, t), np.gradient(ay, t)

    jerk_mag = np.hypot(jx, jy)
    rms_jerk = float(np.sqrt(np.mean(jerk_mag ** 2)))

    return 1.0 / (1.0 + rms_jerk / 1000.0)


def preview_live(marker_id=None):
    """Live ArUco preview. ESC to quit."""
    import cv2

    marker_id = marker_id or config.END_EFFECTOR_MARKER_ID
    camera_matrix, dist_coeffs = load_lens_calibration()
    aruco_dict = _get_aruco_dict()
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    cap, backend = camera_utils.open_camera()
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera {config.CAMERA_INDEX}.")

    print(f"Preview -- looking for ID {marker_id} ({config.ARUCO_DICT}). ESC to quit.")

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        frame = _undistort(frame, camera_matrix, dist_coeffs)
        corners, ids, _ = detector.detectMarkers(frame)

        display = frame.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(display, corners, ids)
            for c, i in zip(corners, ids.flatten()):
                if i == marker_id:
                    cx = int(c[0][:, 0].mean())
                    cy = int(c[0][:, 1].mean())
                    cv2.putText(display, "TARGET", (cx - 40, cy - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.putText(display, f"ID {marker_id}  ESC=quit",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("preview", display)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python camera_tracking.py [calibrate|test|preview]")
        sys.exit(0)

    if sys.argv[1] == "calibrate":
        calibrate()
    elif sys.argv[1] == "preview":
        preview_live()
    elif sys.argv[1] == "test":
        print("Tracking 3s...")
        path = track_marker_path(3.0)
        print(f"Captured {len(path)} points.")
        if path:
            print("First:", path[0])
            print("Last:", path[-1])
            print("Smoothness:", compute_smoothness_index(path))
