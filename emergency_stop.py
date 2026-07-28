"""Software e-stop: press ESC to stop+clear the robot queue. Physical forearm button is faster."""

import argparse
import threading
import time

import config


class EmergencyStop:
    def __init__(self, controller, hotkey="esc"):
        self.controller = controller
        self.hotkey = hotkey
        self._armed = False
        self._triggered = threading.Event()

    def arm(self):
        import keyboard
        keyboard.add_hotkey(self.hotkey, self._on_trigger)
        self._armed = True
        print(f"[estop] Armed on '{self.hotkey}'. Physical button is still safer.")

    def disarm(self):
        if self._armed:
            import keyboard
            keyboard.remove_hotkey(self.hotkey)
            self._armed = False

    def _on_trigger(self):
        self._triggered.set()
        print("\n[estop][TRIGGERED] Stopping robot...")
        try:
            device = self.controller.device
            device._set_queued_cmd_stop_exec()
            device._set_queued_cmd_clear()
            print("[estop] Stop + clear sent.")
        except Exception as e:
            print(f"[estop][ERROR] {e}. Use the physical button.")

    def was_triggered(self):
        return self._triggered.is_set()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=str, default=config.DOBOT_PORT)
    args = p.parse_args()

    from dobot_interface import DobotController
    ctrl = DobotController(port=args.port, verbose=False).connect()

    estop = EmergencyStop(ctrl)
    estop.arm()

    print("Press ESC to test, Ctrl+C to exit.")
    try:
        while not estop.was_triggered():
            time.sleep(0.2)
        print("E-stop triggered.")
    except KeyboardInterrupt:
        pass
    finally:
        estop.disarm()
        ctrl.disconnect()
