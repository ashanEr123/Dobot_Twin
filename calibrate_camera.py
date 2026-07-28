"""One-time lens calibration with a printed checkerboard. Saves camera_calibration.json."""

import argparse
import json
import sys

import numpy as np
import config
import camera_utils


def parse_args():
    p = argparse.ArgumentParser(description="Webcam lens calibration")
    p.add_argument("--camera-index", type=int, default=config.CAMERA_INDEX)
    p.add_argument("--square-size-mm", type=float, required=True,
                    help="Measured size of one checkerboard square (mm)")
    p.add_argument("--corners", type=str, default=None,
                    help='Internal corners as "COLSxROWS", e.g. "9x6"')
    p.add_argument("--min-captures", type=int, default=15)
    return p.parse_args()


def collect_checkerboard_captures(cap, pattern_size, min_captures):
    """SPACE to capture, ESC to finish. Returns (imgpoints, frame_size)."""
    import cv2

    imgpoints = []
    frame_size = None

    print(f"Looking for {pattern_size[0]}x{pattern_size[1]} corners. "
          f"SPACE=capture, ESC=done.\nCaptured: 0", end="", flush=True)

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        frame_size = (frame.shape[1], frame.shape[0])
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        found, corners = cv2.findChessboardCorners(
            gray, pattern_size,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, pattern_size, corners, found)

        cv2.putText(display, f"Captures: {len(imgpoints)}  "
                              f"(SPACE=capture, ESC=finish)",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("calibrate_camera", display)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        if key == 32 and found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(refined)
            print(f"\rCaptured: {len(imgpoints)}", end="", flush=True)

    cap.release()
    cv2.destroyAllWindows()
    print()

    if len(imgpoints) < min_captures:
        print(f"[warn] Only {len(imgpoints)} captures (want {min_captures}).")

    return imgpoints, frame_size


def run_calibration(imgpoints, frame_size, pattern_size, square_size_mm):
    import cv2

    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size_mm
    objpoints = [objp for _ in imgpoints]

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, frame_size, None, None
    )
    return rms, camera_matrix, dist_coeffs


def save_calibration(path, rms, camera_matrix, dist_coeffs, frame_size):
    data = dict(
        rms_reprojection_error=float(rms),
        camera_matrix=camera_matrix.tolist(),
        dist_coeffs=dist_coeffs.flatten().tolist(),
        image_width=frame_size[0],
        image_height=frame_size[1],
    )
    with open(path, "w") as f:
        json.dump(data, f, indent=2)


def main():
    args = parse_args()

    if args.corners:
        cols, rows = (int(v) for v in args.corners.lower().split("x"))
        pattern_size = (cols, rows)
    else:
        pattern_size = tuple(config.CHECKERBOARD_INTERNAL_CORNERS)

    cap, backend = camera_utils.open_camera(index=args.camera_index)
    if not cap.isOpened():
        print(f"[ERROR] Could not open camera {args.camera_index}.")
        sys.exit(1)
    print(f"Opened camera {args.camera_index} via {backend}.")

    imgpoints, frame_size = collect_checkerboard_captures(
        cap, pattern_size, args.min_captures
    )

    if len(imgpoints) < 4:
        print("[ERROR] Need at least 4 captures.")
        sys.exit(1)

    print("Running calibration...")
    rms, camera_matrix, dist_coeffs = run_calibration(
        imgpoints, frame_size, pattern_size, args.square_size_mm
    )

    save_calibration(config.CALIBRATION_FILE, rms, camera_matrix, dist_coeffs, frame_size)

    print(f"\nRMS reprojection error: {rms:.4f} px")
    if rms > config.RMS_REPROJECTION_WARNING_THRESHOLD:
        print(f"[warn] Above {config.RMS_REPROJECTION_WARNING_THRESHOLD} px -- recalibrate.")
    print(f"Saved -> {config.CALIBRATION_FILE}")


if __name__ == "__main__":
    main()
