"""Run LHS-sampled trials on the robot; append metrics to experiment_data.csv."""

import os
import csv
import time
import numpy as np
from scipy.stats import qmc

import config
import digital_twin
import safety
from dobot_interface import DobotController
import camera_tracking as ct

FIELDNAMES = [
    "run_index", "trial_id", "repeat_id",
    "velocity_ratio", "acceleration_ratio", "blend_ratio",
    "time_s", "position_error_mm", "energy_proxy", "smoothness_index",
]


def generate_trials():
    """Latin Hypercube sample, expanded by N_REPEATS, then shuffled."""
    sampler = qmc.LatinHypercube(d=3, seed=config.RANDOM_SEED)
    unit_samples = sampler.random(n=config.N_UNIQUE_TRIALS)

    lo = np.array([config.VELOCITY_RATIO_RANGE[0],
                    config.ACCELERATION_RATIO_RANGE[0],
                    config.BLEND_RATIO_RANGE[0]])
    hi = np.array([config.VELOCITY_RATIO_RANGE[1],
                    config.ACCELERATION_RATIO_RANGE[1],
                    config.BLEND_RATIO_RANGE[1]])
    scaled = qmc.scale(unit_samples, lo, hi)
    scaled = np.round(scaled).astype(int)

    trials = []
    for trial_id, (v, a, b) in enumerate(scaled):
        for repeat_id in range(config.N_REPEATS):
            trials.append(dict(trial_id=trial_id, repeat_id=repeat_id,
                                velocity_ratio=int(v), acceleration_ratio=int(a),
                                blend_ratio=int(b)))

    rng = np.random.default_rng(config.RANDOM_SEED)
    rng.shuffle(trials)
    for i, t in enumerate(trials):
        t["run_index"] = i

    return trials


def load_completed_run_indices():
    if not os.path.exists(config.DATA_CSV):
        return set()
    done = set()
    with open(config.DATA_CSV, newline="") as f:
        for row in csv.DictReader(f):
            done.add(int(row["run_index"]))
    return done


def ensure_csv_header():
    if not os.path.exists(config.DATA_CSV):
        with open(config.DATA_CSV, "w", newline="") as f:
            csv.DictWriter(f, fieldnames=FIELDNAMES).writeheader()


def append_row(row):
    with open(config.DATA_CSV, "a", newline="") as f:
        csv.DictWriter(f, fieldnames=FIELDNAMES).writerow(row)


def run_one_trial(ctrl, v, a, b, session=None):
    ok, reason = digital_twin.sanity_check(v, a, b)
    if not ok:
        print(f"[skip] {reason}")
        return None

    # Track until the move finishes (Event), not a guessed duration.
    import threading
    stop_event = threading.Event()
    path_holder = {}

    def _track():
        path_holder["path"] = ct.track_marker_path_until(stop_event, session=session)

    tracker_thread = threading.Thread(target=_track)
    tracker_thread.start()

    exec_time = ctrl.run_pick_and_place(v, a, b)

    stop_event.set()
    tracker_thread.join()
    path = path_holder.get("path", [])

    if path:
        final_xy = (path[-1][1], path[-1][2])
        position_error_mm = ct.compute_placement_error(final_xy)
        smoothness_index = ct.compute_smoothness_index(path)
    else:
        position_error_mm = float("nan")
        smoothness_index = float("nan")

    energy_proxy = digital_twin.estimate_energy_proxy(v, a, exec_time)

    return dict(time_s=exec_time, position_error_mm=position_error_mm,
                energy_proxy=energy_proxy, smoothness_index=smoothness_index)


def main():
    print("--- Reachability check ---")
    ok = safety.check_all_waypoints()
    if not ok:
        proceed = input("\nSafety failed. Type 'yes' to proceed: ")
        if proceed.strip().lower() != "yes":
            print("Aborted.")
            return
    print()

    ensure_csv_header()
    trials = generate_trials()
    done = load_completed_run_indices()

    remaining = [t for t in trials if t["run_index"] not in done]
    print(f"{len(trials)} total, {len(done)} done, {len(remaining)} remaining.")

    ctrl = DobotController(verbose=False).connect()
    print("\nHoming (~10-20s)...")
    ctrl.home()
    print()

    session = ct.TrackingSession()
    try:
        for i, t in enumerate(remaining):
            print(f"\n[{i+1}/{len(remaining)}] run_index={t['run_index']} "
                  f"trial_id={t['trial_id']} repeat={t['repeat_id']} "
                  f"v={t['velocity_ratio']} a={t['acceleration_ratio']} "
                  f"blend={t['blend_ratio']}")

            result = run_one_trial(ctrl, t["velocity_ratio"],
                                    t["acceleration_ratio"], t["blend_ratio"],
                                    session=session)
            if result is None:
                continue

            row = {**t, **result}
            append_row(row)
            print(f"  -> time={result['time_s']:.2f}s "
                  f"error={result['position_error_mm']:.2f}mm "
                  f"energy={result['energy_proxy']:.3f} "
                  f"smoothness={result['smoothness_index']:.3f}")

            time.sleep(0.5)
    finally:
        session.close()
        ctrl.disconnect()

    print(f"\nDone. Data saved to {config.DATA_CSV}")


if __name__ == "__main__":
    main()
