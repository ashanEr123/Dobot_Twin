"""Reachability checks for Dobot Magician poses. Conservative heuristics, not exact kinematics."""

import math
import config

MAX_REACH_MM = 320.0
REACH_SAFETY_MARGIN_MM = 30.0
MAX_SAFE_RADIUS_MM = MAX_REACH_MM - REACH_SAFETY_MARGIN_MM  # 290mm

MIN_RADIUS_MM = 150.0  # near-base zone is awkward for this arm

MAX_BASE_ANGLE_DEG = 85.0  # published J1 is +/-90

# High radius + deep Z is often unreachable (dome-shaped workspace).
HIGH_RADIUS_THRESHOLD_MM = 250.0
MIN_Z_AT_HIGH_RADIUS_MM = -60.0


def check_pose(x, y, z, label=""):
    """Returns (ok, warnings)."""
    warnings = []
    ok = True
    radius = math.hypot(x, y)
    angle_deg = math.degrees(math.atan2(y, x))

    if radius > MAX_SAFE_RADIUS_MM:
        ok = False
        warnings.append(
            f"{label}: radius {radius:.1f}mm exceeds safe max "
            f"{MAX_SAFE_RADIUS_MM:.1f}mm (published max {MAX_REACH_MM}mm).")

    if radius < MIN_RADIUS_MM:
        ok = False
        warnings.append(
            f"{label}: radius {radius:.1f}mm below min {MIN_RADIUS_MM}mm.")

    if abs(angle_deg) > MAX_BASE_ANGLE_DEG:
        ok = False
        warnings.append(
            f"{label}: base angle {angle_deg:.1f} deg exceeds "
            f"+/-{MAX_BASE_ANGLE_DEG} deg.")

    if radius > HIGH_RADIUS_THRESHOLD_MM and z < MIN_Z_AT_HIGH_RADIUS_MM:
        ok = False
        warnings.append(
            f"{label}: high radius ({radius:.1f}mm) with low z ({z:.1f}mm).")

    return ok, warnings


def check_all_waypoints():
    """Check pick, target, and hover poses from config."""
    all_ok = True
    p = config.PICK_POSE
    t = config.target_pose()
    hover_z = p["z"] + config.HOVER_Z_OFFSET

    checks = [
        ("PICK_POSE", p["x"], p["y"], p["z"]),
        ("PICK_POSE hover", p["x"], p["y"], hover_z),
        ("target", t["x"], t["y"], t["z"]),
        ("target hover", t["x"], t["y"], hover_z),
    ]

    for label, x, y, z in checks:
        ok, warnings = check_pose(x, y, z, label=label)
        for w in warnings:
            print(f"[safety][{'BLOCKED' if not ok else 'warn'}] {w}")
        all_ok = all_ok and ok

    if all_ok:
        print("[safety] All waypoints passed. Still verify with dry_run_waypoints.py.")
    else:
        print("[safety] One or more waypoints failed. Adjust config or dry-run carefully.")

    return all_ok


if __name__ == "__main__":
    check_all_waypoints()
