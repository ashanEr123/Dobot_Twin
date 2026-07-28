"""List camera indices so you can set CAMERA_INDEX in config.py."""

import cv2
import config
import camera_utils

MAX_INDEX_TO_CHECK = 5
PREVIEW_FRAMES = 30
READ_RETRIES = 10
DISPLAY_WIDTH = 960
DISPLAY_HEIGHT = 540


def main():
    found_any = False

    for index in range(MAX_INDEX_TO_CHECK + 1):
        cap, backend = camera_utils.open_camera(index=index, warmup_frames=5)
        if not cap.isOpened():
            print(f"Index {index}: no camera")
            cap.release()
            continue

        ok, frame = False, None
        for _ in range(READ_RETRIES):
            ok, frame = cap.read()
            if ok and frame is not None:
                break

        if not ok:
            print(f"Index {index} ({backend}): opened but no frames")
            cap.release()
            continue

        found_any = True
        h, w = frame.shape[:2]
        print(f"Index {index} ({backend}): {w}x{h} -- press any key for next")

        window_name = "list_cameras"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, DISPLAY_WIDTH, DISPLAY_HEIGHT)

        for _ in range(PREVIEW_FRAMES):
            ok, frame = cap.read()
            if not ok:
                break
            cv2.putText(frame, f"CAMERA_INDEX = {index} ({backend})",
                        (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.imshow(window_name, frame)
            if cv2.waitKey(100) != -1:
                break

        cap.release()
        cv2.destroyAllWindows()

    if not found_any:
        print("No cameras found.")
    else:
        print("Set CAMERA_INDEX in config.py to the index you want.")


if __name__ == "__main__":
    main()
