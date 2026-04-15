#!/usr/bin/env python3

# This code was made using the help of AI and is a custom script for gamepad teleop, specifically using PS4 DualShock wireless controller

import os
import struct
import time
import json 


from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

# -------------------------
# Fixed settings
# -------------------------
DEVICE = "/dev/input/js0"       # port assigned to gamepad controller
ROBOT_PORT = "/dev/ttyACM0"     # port to Serial Bus Driver

CALIBRATION_PATH = os.path.expanduser(
    "~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower_arm.json"
)

DEADZONE = 0.08  # we define a deadzone to account for 'stick drift' with controller joysticks
SPEED = 35.0
ROLL_SPEED = 20.0
GRIPPER_SPEED = 10.5
LOOP_HZ = 30.0
DT = 1.0 / LOOP_HZ

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
MAX_ABS = 32767.0



# -------------------------
# Controller mapping
# -------------------------
AXIS_LSX = 0
AXIS_LSY = 1
AXIS_L2 = 2
AXIS_RSX = 3
AXIS_RSY = 4
AXIS_R2 = 5

BTN_CROSS = 0
BTN_L1 = 4
BTN_R1 = 5
BTN_OPTIONS = 9

axis_state = {
    AXIS_LSX: 0,
    AXIS_LSY: 0,
    AXIS_L2: -32767,
    AXIS_RSX: 0,
    AXIS_RSY: 0,
    AXIS_R2: -32767,
}

button_state = {
    BTN_CROSS: 0,
    BTN_L1: 0,
    BTN_R1: 0,
    BTN_OPTIONS: 0,
}


def normalize_stick(raw: int) -> float:
    value = raw / MAX_ABS
    if abs(value) < DEADZONE:
        return 0.0
    if value > 1.0:
        return 1.0
    if value < -1.0:
        return -1.0
    return value


def normalize_trigger(raw: int) -> float:
    # Rest = -32767, full press = +32767
    value = (raw + 32767.0) / 65534.0
    if value < 0.0:
        return 0.0
    if value > 1.0:
        return 1.0
    return value


def read_js_event(js) -> tuple[int, int, int, int] | None:
    data = js.read(8)
    if data is None or len(data) != 8:
        return None
    return struct.unpack("IhBB", data)


def update_controller_state(js) -> None:
    while True:
        event = read_js_event(js)
        if event is None:
            break

        _, value, event_type, number = event
        event_type &= ~JS_EVENT_INIT

        if event_type == JS_EVENT_AXIS and number in axis_state:
            axis_state[number] = value
        elif event_type == JS_EVENT_BUTTON and number in button_state:
            button_state[number] = value



def load_calibration() -> dict[str, MotorCalibration]:
    with open(CALIBRATION_PATH, "r") as f:
        raw = json.load(f)

    calibration = {}
    for name, item in raw.items():
        calibration[name] = MotorCalibration(
            id=item["id"],
            drive_mode=item["drive_mode"],
            homing_offset=item["homing_offset"],
            range_min=item["range_min"],
            range_max=item["range_max"],
        )
    return calibration

def build_bus() -> FeetechMotorsBus:
    return FeetechMotorsBus(
        port=ROBOT_PORT,
        motors={
            "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
            "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
            "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
            "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
            "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
            "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        },
        calibration=load_calibration(),
    )


def read_present_positions(bus: FeetechMotorsBus) -> dict[str, float]:
    positions = {}
    for name in bus.motors:
        positions[name] = float(bus.read("Present_Position", name))
    return positions


def print_live_positions(bus: FeetechMotorsBus) -> None:
    pos = read_present_positions(bus)
    print("\nLIVE Present_Position:")
    for name, value in pos.items():
        print(f"  {name:14s} {value:8.3f}")


def main() -> None:
    if not os.path.exists(DEVICE):
        raise FileNotFoundError(f"Controller not found: {DEVICE}")

    bus = build_bus()
    print("Connecting to robot bus...")
    bus.connect()

    print("Reading startup positions...")
    target = read_present_positions(bus)

    print("Startup target positions:")
    for name, value in target.items():
        print(f"  {name:14s} {value:8.3f}")

    cross_prev = 0
    gripper_fault = False

    try:
        with open(DEVICE, "rb", buffering=0) as js:
            os.set_blocking(js.fileno(), False)
            print("\nTeleop running.")
            print("Controls:")
            print("  LSX -> shoulder_pan")
            print("  LSY -> shoulder_lift")
            print("  RSX -> elbow_flex")
            print("  RSY -> wrist_flex")
            print("  L1/R1 -> wrist_roll")
            print("  L2 -> close gripper")
            print("  R2 -> open gripper")
            print("  Cross -> print LIVE positions")
            print("  Options -> exit")

            while True:
                loop_start = time.monotonic()
                update_controller_state(js)

                if button_state[BTN_OPTIONS]:
                    print("\nOptions pressed. Exiting.")
                    break

                cross_now = button_state[BTN_CROSS]
                if cross_now and not cross_prev:
                    print_live_positions(bus)
                cross_prev = cross_now

                lsx = normalize_stick(axis_state[AXIS_LSX])
                lsy = normalize_stick(axis_state[AXIS_LSY])
                rsx = normalize_stick(axis_state[AXIS_RSX])
                rsy = normalize_stick(axis_state[AXIS_RSY])

                l1 = float(button_state[BTN_L1])
                r1 = float(button_state[BTN_R1])

                l2 = normalize_trigger(axis_state[AXIS_L2])
                r2 = normalize_trigger(axis_state[AXIS_R2])


                if gripper_fault and l2 < 0.05 and r2 < 0.05:
                    gripper_fault = False
                    print("\nGripper fault cleared.")

            

                target["shoulder_pan"] += lsx * SPEED * DT
                target["shoulder_lift"] -= lsy * SPEED * DT
                target["elbow_flex"] += rsx * SPEED * DT
                target["wrist_flex"] -= rsy * SPEED * DT
                target["wrist_roll"] += (r1 - l1) * ROLL_SPEED * DT
                target["gripper"] += (l2 - r2) * GRIPPER_SPEED * DT

                for name, value in target.items():
                    if name == "gripper" and gripper_fault:
                        continue

                    try:
                        bus.write("Goal_Position", name, value)
                    except RuntimeError as e:
                        print(f"\nWrite failed for {name}: {e}")

                        if name == "gripper" and "Overload error" in str(e):
                            gripper_fault = True
                            print("Gripper overload latched. Release triggers to clear.")
                            continue

                        raise

                elapsed = time.monotonic() - loop_start
                sleep_time = DT - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

    finally:
        try:
            bus.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
