"""Shared settings used by every script. Edit poses and port for your setup."""

# --- Robot ---
# Windows: "COM3"  Linux: "/dev/ttyUSB0"  macOS: "/dev/tty.usbmodem..."
DOBOT_PORT = "COM7"

# --- Task geometry (mm, Dobot base frame) ---
# Measure with capture_poses.py or DobotStudio.
PICK_POSE = dict(x=271.34, y=-56.85, z=-94.97)

HOVER_Z_OFFSET = 5  # mm above pick/place when hovering

GRID_ORIGIN = dict(x=271.34, y=-56.85, z=-94.97)  # cell (0,0)
GRID_SPACING_MM = 20
GRID_ROWS = 3
GRID_COLS = 3

TARGET_CELL = (0, 1)

# If set, used instead of GRID_ORIGIN + row/col*spacing for the place pose.
TARGET_POSE_OVERRIDE = dict(x=252.1, y=-59.42, z=-93.28, r=0)

def target_pose():
    """Place pose for TARGET_CELL (mm)."""
    if TARGET_POSE_OVERRIDE is not None:
        return dict(TARGET_POSE_OVERRIDE)
    row, col = TARGET_CELL
    x = GRID_ORIGIN["x"] + row * GRID_SPACING_MM
    y = GRID_ORIGIN["y"] + col * GRID_SPACING_MM
    z = GRID_ORIGIN["z"]
    return dict(x=x, y=y, z=z, r=0)

# --- Decision variable ranges (Dobot ratios, 0-100) ---
VELOCITY_RATIO_RANGE = (20, 100)
ACCELERATION_RATIO_RANGE = (20, 100)
BLEND_RATIO_RANGE = (0, 100)

# --- Experiment ---
N_UNIQUE_TRIALS = 70
N_REPEATS = 3
RANDOM_SEED = 42

DATA_CSV = "experiment_data.csv"
MODEL_DIR = "models"
PARETO_CSV = "pareto_front.csv"
CANDIDATES_CSV = "selected_candidates.csv"
VALIDATION_CSV = "validation_results.csv"

BASELINE_PARAMS = dict(velocity_ratio=50, acceleration_ratio=50, blend_ratio=0)

# --- Energy proxy (no power sensor on Magician) ---
# energy_proxy = (W_V * v^2 + W_A * a^2) * time_s / 1e4
ENERGY_VELOCITY_WEIGHT = 1.0
ENERGY_ACCEL_WEIGHT = 0.5

# --- Camera / ArUco ---
CAMERA_INDEX = 1
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
CAMERA_FPS = 30
ARUCO_DICT = "DICT_4X4_50"
END_EFFECTOR_MARKER_ID = 0
TRACKING_FPS = 30

CALIBRATION_FILE = "camera_calibration.json"
CHECKERBOARD_INTERNAL_CORNERS = (9, 6)
RMS_REPROJECTION_WARNING_THRESHOLD = 1.0

WORKSPACE_CALIBRATION_FILE = "workspace_calibration.npy"
