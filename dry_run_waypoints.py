"""Slow, confirmed waypoint walkthrough before full-speed trials."""

import config
import safety
from dobot_interface import DobotController
from emergency_stop import EmergencyStop

DRY_RUN_VELOCITY = 15
DRY_RUN_ACCELERATION = 15


def main():
    print("=== Dry run: one waypoint at a time ===\n")

    print("--- Reachability check ---")
    ok = safety.check_all_waypoints()
    if not ok:
        proceed = input("\nSafety check failed. Type 'yes' to continue anyway: ")
        if proceed.strip().lower() != "yes":
            print("Aborted.")
            return

    p = config.PICK_POSE
    t = config.target_pose()
    hover_z = p["z"] + config.HOVER_Z_OFFSET

    if "r" not in p:
        print(f"[warn] PICK_POSE missing 'r' -- using 0. Current: {p}")
    pick_r = p.get("r", 0)

    waypoints = [
        ("hover above pick", p["x"], p["y"], hover_z, pick_r),
        ("pick (descend)", p["x"], p["y"], p["z"], pick_r),
        ("hover above pick (retreat)", p["x"], p["y"], hover_z, pick_r),
        ("hover above target", t["x"], t["y"], hover_z, t["r"]),
        ("target (descend)", t["x"], t["y"], t["z"], t["r"]),
        ("hover above target (retreat)", t["x"], t["y"], hover_z, t["r"]),
    ]

    print("\n--- Connecting ---")
    ctrl = DobotController(verbose=False).connect()
    print("Homing (~10-20s)...")
    ctrl.home()
    print()

    ctrl.set_motion_params(DRY_RUN_VELOCITY, DRY_RUN_ACCELERATION)
    print(f"Speed locked to v={DRY_RUN_VELOCITY} a={DRY_RUN_ACCELERATION}.")

    estop = EmergencyStop(ctrl)
    estop.arm()

    try:
        for i, (label, x, y, z, r) in enumerate(waypoints):
            print(f"\n[{i+1}/{len(waypoints)}] '{label}' "
                  f"-> x={x:.1f} y={y:.1f} z={z:.1f} r={r:.1f}")

            wp_ok, warnings = safety.check_pose(x, y, z, label=label)
            for w in warnings:
                print(f"  [safety] {w}")
            user_accepted_risk = False
            if not wp_ok:
                proceed = input("  Failed safety. Type 'yes' to move anyway: ")
                if proceed.strip().lower() != "yes":
                    print("  Stopped.")
                    break
                user_accepted_risk = True

            if estop.was_triggered():
                print("  E-stop was triggered earlier -- stopping.")
                break

            input("  Press Enter to move (Ctrl+C to abort)... ")

            ctrl.safe_move_to(x, y, z, r, max_step_mm=30,
                              velocity_ratio=DRY_RUN_VELOCITY,
                              acceleration_ratio=DRY_RUN_ACCELERATION,
                              context_label=label,
                              override_safety=user_accepted_risk)
            print(f"  Done. Pose: {ctrl.get_pose()}")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        estop.disarm()
        ctrl.disconnect()

    print("\nDry run finished.")


if __name__ == "__main__":
    main()
