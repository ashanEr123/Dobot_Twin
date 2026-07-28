"""Run selected Pareto candidates + baseline on the robot; compare mean/std."""

import pandas as pd

import config
import safety
import camera_tracking as ct
from dobot_interface import DobotController
from data_collection import run_one_trial


def build_candidate_list():
    candidates = pd.read_csv(config.CANDIDATES_CSV)
    rows = [dict(label="baseline_manual", **config.BASELINE_PARAMS)]
    for _, r in candidates.iterrows():
        rows.append(dict(label=r["label"], velocity_ratio=int(r["velocity_ratio"]),
                          acceleration_ratio=int(r["acceleration_ratio"]),
                          blend_ratio=int(r["blend_ratio"])))
    return rows


def main():
    candidates = build_candidate_list()
    print(f"Validating {len(candidates)} configs ({config.N_REPEATS} repeats each):")
    for c in candidates:
        print(f"  {c}")

    print("\n--- Reachability check ---")
    ok = safety.check_all_waypoints()
    if not ok:
        proceed = input("\nSafety failed. Type 'yes' to proceed: ")
        if proceed.strip().lower() != "yes":
            print("Aborted.")
            return
    print()

    ctrl = DobotController(verbose=False).connect()
    print("Homing (~10-20s)...")
    ctrl.home()
    print()

    session = ct.TrackingSession()
    results = []
    try:
        for c in candidates:
            for repeat in range(config.N_REPEATS):
                print(f"\n[{c['label']}] repeat {repeat+1}/{config.N_REPEATS}")
                r = run_one_trial(ctrl, c["velocity_ratio"],
                                   c["acceleration_ratio"], c["blend_ratio"],
                                   session=session)
                if r is None:
                    continue
                row = dict(label=c["label"], repeat=repeat,
                           velocity_ratio=c["velocity_ratio"],
                           acceleration_ratio=c["acceleration_ratio"],
                           blend_ratio=c["blend_ratio"], **r)
                results.append(row)
                print(f"  -> time={r['time_s']:.2f}s "
                      f"error={r['position_error_mm']:.2f}mm "
                      f"energy={r['energy_proxy']:.3f} "
                      f"smoothness={r['smoothness_index']:.3f}")
    finally:
        session.close()
        ctrl.disconnect()

    df = pd.DataFrame(results)
    df.to_csv(config.VALIDATION_CSV, index=False)
    print(f"\nSaved -> {config.VALIDATION_CSV}")

    summary = df.groupby("label")[
        ["time_s", "position_error_mm", "energy_proxy", "smoothness_index"]
    ].agg(["mean", "std"])
    print("\n=== Summary (mean +/- std) ===")
    print(summary)


if __name__ == "__main__":
    main()
