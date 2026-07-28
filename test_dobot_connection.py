"""Minimal connection test: open the port and read pose."""

import argparse
import sys

import config
from dobot_interface import robust_connect


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=str, default=config.DOBOT_PORT)
    p.add_argument("--boot-wait", type=float, default=2.0,
                    help="Seconds to wait after opening the port (board boot)")
    return p.parse_args()


def main():
    args = parse_args()
    port = args.port

    print(f"Connecting on {port} ...")
    try:
        import pydobot  # noqa: F401
    except ImportError:
        print("[ERROR] pip install pydobot")
        sys.exit(1)

    try:
        device = robust_connect(port, verbose=True, boot_wait_s=args.boot_wait)
    except Exception as e:
        print(f"\n[FAILED] {type(e).__name__}: {e}")
        print("Check: COM port, DobotStudio closed, power on, USB drivers.")
        sys.exit(1)

    print(f"\n[SUCCESS] Connected on {port}.")
    try:
        pose = device.pose()
        print(f"Pose: {pose}")
    except Exception as e:
        print(f"[WARNING] pose() failed: {e}")
    finally:
        device.close()
        print("Disconnected.")


if __name__ == "__main__":
    main()
