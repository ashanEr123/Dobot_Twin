"""Thin wrapper around pydobot for the rest of the project."""

import time
import math
import config


def _patch_pydobot_read_timing():
    """Poll serial until a reply arrives (pydobot 1.3.2 can miss slow responses)."""
    import pydobot.dobot as _dobot_module
    from pydobot.message import Message

    if getattr(_dobot_module.Dobot, "_read_timing_patched", False):
        return

    def _read_message_patched(self, timeout=4.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            b = self.ser.read_all()
            if b and len(b) > 0:
                msg = Message(b)
                print(f'[dobot][diag] received {len(b)} bytes -> parsed OK')
                if self.verbose:
                    print('pydobot: <<', msg)
                return msg
            time.sleep(0.05)
        print(f'[dobot][diag] TIMED OUT after {timeout}s -- zero bytes '
              f'received from the robot for this command.')
        return None

    _dobot_module.Dobot._read_message = _read_message_patched
    _dobot_module.Dobot._read_timing_patched = True


def _add_homing_support():
    """Add SET_HOME_CMD (protocol ID 31); not in stock pydobot."""
    import struct
    from pydobot.dobot import Dobot
    from pydobot.message import Message
    from pydobot.enums.ControlValues import ControlValues

    if getattr(Dobot, "_homing_patched", False):
        return

    HOME_CMD_ID = 31

    def _set_home_cmd(self, wait=True):
        msg = Message()
        msg.id = HOME_CMD_ID
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray(struct.pack('I', 0))
        return self._send_command(msg, wait)

    Dobot._set_home_cmd = _set_home_cmd
    Dobot._homing_patched = True


def _add_cp_params_support():
    """Add SET_CP_PARAMS (protocol ID 90); not in stock pydobot."""
    import struct
    from pydobot.dobot import Dobot
    from pydobot.message import Message
    from pydobot.enums.ControlValues import ControlValues

    if getattr(Dobot, "_cp_params_patched", False):
        return

    CP_PARAMS_ID = 90

    def _set_cp_params(self, plan_acc, junction_vel, acc, real_time_track=0, wait=False):
        msg = Message()
        msg.id = CP_PARAMS_ID
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', plan_acc)))
        msg.params.extend(bytearray(struct.pack('f', junction_vel)))
        msg.params.extend(bytearray(struct.pack('f', acc)))
        msg.params.extend(bytearray(struct.pack('B', real_time_track)))
        return self._send_command(msg, wait)

    Dobot._set_cp_params = _set_cp_params
    Dobot._cp_params_patched = True


def robust_connect(port, verbose=False, retries=3, boot_wait_s=2.0):
    """Open serial briefly first (board reset), wait, then connect via pydobot."""
    import serial
    import pydobot

    _patch_pydobot_read_timing()

    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(boot_wait_s)
        try:
            ser.dtr = False
            ser.rts = False
        except Exception:
            pass
        time.sleep(0.2)
        ser.close()
        time.sleep(0.3)
    except Exception as e:
        print(f"[dobot][warn] Pre-connect port toggle failed ({e}) -- "
              f"continuing anyway.")

    last_exc_str = None
    last_exc_type = None
    for attempt in range(1, retries + 1):
        try:
            return pydobot.Dobot(port=port, verbose=verbose)
        except Exception as e:
            # Don't keep `e` alive across retries (holds the serial port open).
            last_exc_type = type(e).__name__
            last_exc_str = str(e)
            print(f"[dobot] Connect attempt {attempt}/{retries} failed "
                  f"({last_exc_type}: {last_exc_str}). Retrying...")
            del e
            import gc
            gc.collect()
            time.sleep(1.5)

    raise ConnectionError(
        f"Could not connect to Dobot on {port} after {retries} attempts. "
        f"Last error: {last_exc_type}: {last_exc_str}"
    )


class DobotController:
    def __init__(self, port=None, verbose=False):
        self.port = port or config.DOBOT_PORT
        self.verbose = verbose
        self.device = None

    def connect(self):
        self.device = robust_connect(self.port, verbose=self.verbose)
        print(f"[dobot] Connected on {self.port}")
        return self

    def disconnect(self):
        if self.device is not None:
            self.device.close()
            print("[dobot] Disconnected")

    def home(self):
        """Home the arm (moves physically). Run after each power-on."""
        _add_homing_support()
        print("[dobot] Homing -- clear the workspace...")
        self.device._set_home_cmd(wait=True)
        print("[dobot] Homing complete.")

    def set_motion_params(self, velocity_ratio, acceleration_ratio):
        """Set PTP velocity/acceleration ratios (0-100)."""
        self.device.speed(velocity=velocity_ratio, acceleration=acceleration_ratio)

    def set_cp_blend(self, blend_ratio):
        """Set CP blend ratio (0-100). Maps onto planAcc/junctionVel/acc."""
        try:
            _add_cp_params_support()
            self.device._set_cp_params(blend_ratio, blend_ratio, blend_ratio, 0)
        except Exception as e:
            print(f"[dobot][warn] Failed to set CP params ({type(e).__name__}: "
                  f"{e}) -- blend_ratio={blend_ratio} was NOT applied.")

    def move_to(self, x, y, z, r=0, wait=True):
        self.device.move_to(x, y, z, r, wait=wait)

    def safe_move_to(self, x, y, z, r=0, max_step_mm=30, velocity_ratio=15,
                      acceleration_ratio=15, verbose=True, safe_hover_z=None,
                      context_label=None, override_safety=False):
        """
        Move in small steps: rise, rotate wrist, move XY, then descend.
        Does not model self-collision; watch the arm.
        """
        import safety

        context_label = context_label or f"move to ({x:.1f},{y:.1f},{z:.1f})"

        current = self.get_pose()
        cx, cy, cz, cr = current[0], current[1], current[2], current[3]

        if safe_hover_z is None:
            safe_hover_z = max(cz, z) + config.HOVER_Z_OFFSET

        stages = []
        if cz < safe_hover_z:
            stages.append(("rise to safe height", cx, cy, safe_hover_z, cr))
        stages.append(("rotate wrist at safe height", cx, cy, safe_hover_z, r))
        stages.append(("move horizontally at safe height", x, y, safe_hover_z, r))
        stages.append(("descend to final z", x, y, z, r))

        if verbose:
            print(f"[dobot] safe_move_to -> '{context_label}' "
                  f"({len(stages)} stages)")

        self.set_motion_params(velocity_ratio, acceleration_ratio)

        for stage_label, tx, ty, tz, tr in stages:
            start = self.get_pose()
            sx, sy, sz = start[0], start[1], start[2]
            distance = math.dist((sx, sy, sz), (tx, ty, tz))
            n_steps = max(1, math.ceil(distance / max_step_mm))

            if verbose:
                print(f"  [{context_label} :: {stage_label}] "
                      f"{distance:.1f}mm, {n_steps} step(s)")

            for i in range(1, n_steps + 1):
                t = i / n_steps
                step_x = sx + (tx - sx) * t
                step_y = sy + (ty - sy) * t
                step_z = sz + (tz - sz) * t
                step_r = start[3] + (tr - start[3]) * t

                ok, warnings = safety.check_pose(
                    step_x, step_y, step_z,
                    label=f"{context_label} :: {stage_label} step {i}/{n_steps}")
                if not ok:
                    if override_safety:
                        for w in warnings:
                            print(f"[dobot][safety][OVERRIDDEN] {w}")
                    else:
                        for w in warnings:
                            print(f"[dobot][safety][BLOCKED] {w}")
                        raise RuntimeError(
                            f"safe_move_to aborted at '{context_label}' / "
                            f"'{stage_label}' {i}/{n_steps}: "
                            + " | ".join(warnings))

                if verbose:
                    print(f"    step {i}/{n_steps}: -> x={step_x:.1f} "
                          f"y={step_y:.1f} z={step_z:.1f} r={step_r:.1f}")
                self.move_to(step_x, step_y, step_z, step_r, wait=True)

    def get_pose(self):
        return self.device.pose()

    def suction_on(self):
        self.device.suck(True)

    def suction_off(self):
        self.device.suck(False)

    def run_pick_and_place(self, velocity_ratio, acceleration_ratio, blend_ratio):
        """One pick-and-place cycle. Returns working time in seconds."""
        p = config.PICK_POSE
        t = config.target_pose()
        hover_z = p["z"] + config.HOVER_Z_OFFSET

        if "r" not in p:
            print(f"[warn] PICK_POSE missing 'r' -- using 0. Current: {p}")
        pick_r = p.get("r", 0)

        self.set_motion_params(velocity_ratio, acceleration_ratio)
        self.set_cp_blend(blend_ratio)

        t_start = time.time()

        self.move_to(p["x"], p["y"], hover_z, pick_r)
        self.move_to(p["x"], p["y"], p["z"], pick_r)
        self.suction_on()
        time.sleep(0.3)
        self.move_to(p["x"], p["y"], hover_z, pick_r)
        self.move_to(t["x"], t["y"], hover_z, t["r"])
        self.move_to(t["x"], t["y"], t["z"], t["r"])
        self.suction_off()
        time.sleep(0.3)
        self.move_to(t["x"], t["y"], hover_z, t["r"])

        t_end = time.time()
        execution_time_s = (t_end - t_start) - 0.6  # subtract suction dwells

        return max(execution_time_s, 0.01)


if __name__ == "__main__":
    ctrl = DobotController(verbose=True).connect()
    try:
        t = ctrl.run_pick_and_place(velocity_ratio=50, acceleration_ratio=50, blend_ratio=0)
        print(f"Measured execution time: {t:.2f} s")
    finally:
        ctrl.disconnect()
