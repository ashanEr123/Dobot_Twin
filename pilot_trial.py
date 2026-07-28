"""One real pick-and-place trial with diagnostics. Does not write CSV."""

import argparse
import math

import config
import digital_twin
import safety
from dobot_interface import DobotController
from data_collection import run_one_trial


def parse_args():
    p = argparse.ArgumentParser(description="Run one pilot pick-and-place trial")
    p.add_argument("--velocity", type=int,
                    default=config.BASELINE_PARAMS["velocity_ratio"])
    p.add_argument("--acceleration", type=int,
                    default=config.BASELINE_PARAMS["acceleration_ratio"])
    p.add_argument("--blend", type=int,
                    default=config.BASELINE_PARAMS["blend_ratio"])
    return p.parse_args()


def main():
    args = parse_args()
    v, a, b = args.velocity, args.acceleration, args.blend

    print("--- Reachability check ---")
    ok = safety.check_all_waypoints()
    if not ok:
        proceed = input("\nSafety failed. Type 'yes' to proceed: ")
        if proceed.strip().lower() != "yes":
            print("Aborted.")
            return
    print()

    print(f"=== Pilot: v={v} a={a} blend={b} ===\n")

    print("--- Twin estimate ---")
    predicted = digital_twin.preview(v, a, b)
    if predicted is None:
        print("Aborting -- parameters rejected.")
        return
    print()

    print("--- Connecting ---")
    ctrl = DobotController(verbose=False).connect()
    print("\nHoming (~10-20s)...")
    ctrl.home()
    print()

    try:
        print("--- Running trial ---")
        result = run_one_trial(ctrl, v, a, b)
    finally:
        ctrl.disconnect()

    if result is None:
        print("\nTrial did not run.")
        return

    print("\n--- Measured ---")
    for k, val in result.items():
        print(f"  {k}: {val:.4f}" if isinstance(val, float) else f"  {k}: {val}")

    print("\n--- Twin vs actual ---")
    pred_time = predicted["time_s"]
    pred_energy = predicted["energy_proxy"]
    act_time = result["time_s"]
    act_energy = result["energy_proxy"]

    def pct_diff(pred, act):
        if pred == 0:
            return float("nan")
        return 100.0 * (act - pred) / pred

    print(f"  time_s:       pred={pred_time:.2f}  act={act_time:.2f}  "
          f"diff={pct_diff(pred_time, act_time):+.1f}%")
    print(f"  energy_proxy: pred={pred_energy:.3f}  act={act_energy:.3f}  "
          f"diff={pct_diff(pred_energy, act_energy):+.1f}%")

    if math.isnan(result["position_error_mm"]):
        print("\n[warn] position_error_mm is NaN -- check camera tracking.")


if __name__ == "__main__":
    main()
