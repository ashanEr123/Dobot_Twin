"""Lightweight twin: path time estimate and energy proxy. Not a full simulator."""

import math
import config


def path_length_mm():
    """Outbound pick-and-place path length (mm)."""
    p = config.PICK_POSE
    t = config.target_pose()
    hover_z = p["z"] + config.HOVER_Z_OFFSET

    seg1 = abs(hover_z - p["z"])
    seg2 = math.dist((p["x"], p["y"]), (t["x"], t["y"]))
    seg3 = abs(hover_z - t["z"])
    return seg1 + seg2 + seg3


def estimate_time_s(velocity_ratio, acceleration_ratio, path_len_mm=None,
                     v_max_mm_s=200.0, a_max_mm_s2=800.0):
    """Trapezoidal motion-profile time estimate (rough)."""
    if path_len_mm is None:
        path_len_mm = path_length_mm()

    v = max(1e-6, v_max_mm_s * velocity_ratio / 100.0)
    a = max(1e-6, a_max_mm_s2 * acceleration_ratio / 100.0)

    d_accel = v ** 2 / a

    if d_accel >= path_len_mm:
        t = 2 * math.sqrt(path_len_mm / a)
    else:
        t_accel = v / a
        d_cruise = path_len_mm - d_accel
        t_cruise = d_cruise / v
        t = 2 * t_accel + t_cruise

    return t


def estimate_energy_proxy(velocity_ratio, acceleration_ratio, time_s):
    """Same formula used with measured time_s."""
    w_v = config.ENERGY_VELOCITY_WEIGHT
    w_a = config.ENERGY_ACCEL_WEIGHT
    return (w_v * velocity_ratio ** 2 + w_a * acceleration_ratio ** 2) * time_s / 1e4


def sanity_check(velocity_ratio, acceleration_ratio, blend_ratio):
    """Returns (ok, reason)."""
    vr = config.VELOCITY_RATIO_RANGE
    ar = config.ACCELERATION_RATIO_RANGE
    br = config.BLEND_RATIO_RANGE

    if not (vr[0] <= velocity_ratio <= vr[1]):
        return False, f"velocity_ratio {velocity_ratio} outside {vr}"
    if not (ar[0] <= acceleration_ratio <= ar[1]):
        return False, f"acceleration_ratio {acceleration_ratio} outside {ar}"
    if not (br[0] <= blend_ratio <= br[1]):
        return False, f"blend_ratio {blend_ratio} outside {br}"

    return True, "ok"


def preview(velocity_ratio, acceleration_ratio, blend_ratio):
    """Print estimate for one parameter combo."""
    ok, reason = sanity_check(velocity_ratio, acceleration_ratio, blend_ratio)
    if not ok:
        print(f"[REJECTED] {reason}")
        return None

    t = estimate_time_s(velocity_ratio, acceleration_ratio)
    e = estimate_energy_proxy(velocity_ratio, acceleration_ratio, t)
    print(f"v={velocity_ratio} a={acceleration_ratio} blend={blend_ratio} "
          f"-> est. time={t:.2f}s, est. energy_proxy={e:.3f}")
    return dict(time_s=t, energy_proxy=e)


if __name__ == "__main__":
    print(f"Path length for current config: {path_length_mm():.1f} mm\n")
    for v, a, b in [(50, 50, 0), (100, 100, 100), (20, 20, 0)]:
        preview(v, a, b)
