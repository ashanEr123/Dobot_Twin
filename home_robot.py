"""Home the Dobot. Run once after each power-on, before other scripts."""

from dobot_interface import DobotController


def main():
    print("=== Homing the Dobot Magician ===")
    print("Clear the workspace -- the arm will move.\n")
    input("Press Enter when ready (Ctrl+C to cancel)... ")

    ctrl = DobotController(verbose=False).connect()
    try:
        ctrl.home()
        pose = ctrl.get_pose()
        print(f"\nPose after homing: {pose}")
    finally:
        ctrl.disconnect()


if __name__ == "__main__":
    main()
