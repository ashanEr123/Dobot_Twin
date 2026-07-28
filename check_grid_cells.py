"""Print reachability status for every grid cell in config."""

import math
import config
import safety


def main():
    print(f"Checking {config.GRID_ROWS}x{config.GRID_COLS} cells "
          f"({config.GRID_SPACING_MM}mm) from GRID_ORIGIN = {config.GRID_ORIGIN}\n")

    any_ok = False
    all_ok = True

    for row in range(config.GRID_ROWS):
        for col in range(config.GRID_COLS):
            x = config.GRID_ORIGIN["x"] + row * config.GRID_SPACING_MM
            y = config.GRID_ORIGIN["y"] + col * config.GRID_SPACING_MM
            z = config.GRID_ORIGIN["z"]

            radius = math.hypot(x, y)
            angle = math.degrees(math.atan2(y, x))
            ok, warnings = safety.check_pose(x, y, z, label=f"({row},{col})")

            status = "OK" if ok else "BLOCKED"
            marker = "  <-- TARGET_CELL" if (row, col) == tuple(config.TARGET_CELL) else ""
            print(f"  ({row},{col}): radius={radius:6.1f}mm  angle={angle:6.1f}deg  "
                  f"{status}{marker}")
            for w in warnings:
                print(f"       {w}")

            any_ok = any_ok or ok
            all_ok = all_ok and ok

    print()
    if all_ok:
        print("All cells OK.")
    elif any_ok:
        print("Some cells blocked. Pick an OK cell as TARGET_CELL, or move the grid.")
    else:
        print("No cells pass. Move the grid closer / in front of the robot.")


if __name__ == "__main__":
    main()
