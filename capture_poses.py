"""Capture current robot poses and print config.py-ready dicts."""

from dobot_interface import DobotController

KNOWN_LABELS = {
    "PICK_POSE": ["x", "y", "z", "r"],
    "GRID_ORIGIN": ["x", "y", "z"],
}


def format_pose(label, pose):
    x, y, z, r, j1, j2, j3, j4 = pose
    fields = KNOWN_LABELS.get(label.upper(), ["x", "y", "z", "r"])
    values = dict(x=round(x, 2), y=round(y, 2), z=round(z, 2), r=round(r, 2))
    parts = ", ".join(f"{f}={values[f]}" for f in fields)
    return f"{label.upper()} = dict({parts})"


def main():
    print("Connecting...")
    ctrl = DobotController(verbose=False).connect()
    print("Connected.\n")
    print("Hold the forearm button to move by hand, then press Enter to capture.\n")

    captured = []
    try:
        while True:
            label = input("Label (e.g. PICK_POSE) or blank to finish: ").strip()
            if not label:
                break

            input(f"Move to '{label}', then press Enter...")
            pose = ctrl.get_pose()
            line = format_pose(label, pose)
            print(f"  Captured: {line}\n")
            captured.append((label, pose, line))
    finally:
        ctrl.disconnect()

    if not captured:
        print("No poses captured.")
        return

    print("\nPaste into config.py:")
    for label, pose, line in captured:
        print(line)
    print("\nRaw (x, y, z, r, j1, j2, j3, j4):")
    for label, pose, line in captured:
        print(f"  {label}: {pose}")


if __name__ == "__main__":
    main()
