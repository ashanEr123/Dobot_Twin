# Lightweight Digital Twin

Trajectory optimization for the Dobot Magician: digital twin checks → real-robot experiments → Random Forest surrogates → NSGA-II → physical validation.

## Setup

```bash
pip install -r requirements.txt
```

Edit `config.py` (port, poses, camera index) for your hardware.

## Pipeline

1. Home / connect / capture poses as needed
2. Calibrate lens → workspace → test tracking
3. Dry-run waypoints, then pilot trial
4. `data_collection.py` → `train_surrogate.py` → `optimize_nsga2.py` → `validate_and_compare.py`

## File tree

```
Lightweight_Digital_Twin/
├── config.py                 # shared settings (edit this)
├── digital_twin.py           # time/energy estimates + param sanity check
├── safety.py                 # reachability heuristics
├── dobot_interface.py        # pydobot wrapper (connect, home, pick/place)
├── emergency_stop.py         # ESC software stop
│
├── home_robot.py             # home after power-on
├── capture_poses.py          # read poses into config format
├── check_grid_cells.py       # reachability for all grid cells
├── test_dobot_connection.py  # port smoke test
├── dry_run_waypoints.py      # slow confirmed moves
├── pilot_trial.py            # one real trial (no CSV)
├── data_collection.py        # LHS trials → experiment_data.csv
│
├── camera_utils.py           # open camera (DSHOW on Windows)
├── list_cameras.py           # find CAMERA_INDEX
├── calibrate_camera.py       # lens calibration
├── camera_tracking.py        # ArUco tracking + workspace calibrate
│
├── train_surrogate.py        # Random Forest models
├── optimize_nsga2.py         # Pareto front + candidates
├── validate_and_compare.py   # candidates vs baseline on robot
│
├── requirements.txt
├── camera_calibration.json   # lens cal (generated)
├── experiment_data.csv       # trial data (generated)
└── README.md
```

Generated later (not required in repo): `models/`, `pareto_front.csv`, `selected_candidates.csv`, `validation_results.csv`, `workspace_calibration.npy`, `pareto_front.png`.
