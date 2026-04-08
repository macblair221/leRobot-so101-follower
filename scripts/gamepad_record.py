#!/usr/bin/env python3
import time
from pathlib import Path

from ps4_joint_teleop import PS4JointTeleop, PS4JointTeleopConfig

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.pipeline_features import aggregate_pipeline_dataset_features, create_initial_features
from lerobot.datasets.utils import build_dataset_frame, combine_feature_dicts
from lerobot.datasets.video_utils import VideoEncodingManager
from lerobot.processor import make_default_processors
from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.robots.so_follower.so_follower import SOFollower
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.robot_utils import precise_sleep

# -------------------------
# User settings
# -------------------------
ROBOT_PORT = "/dev/ttyACM0"
ROBOT_ID = "my_follower_arm"

GAMEPAD_DEVICE = "/dev/input/js0"

CAMERA_NAME = "front"
CAMERA_DEVICE = "/dev/video0"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

DATASET_REPO_ID = "macblair221/so101-block-place-release-completerom-v1"
DATASET_ROOT = None  # e.g. Path.home() / "lerobot_datasets"
TASK = "Pick up the block and place it."

FPS = 30
EPISODE_TIME_S = 60
NUM_EPISODES_TO_KEEP = 20

PUSH_TO_HUB = False
PRIVATE = False
USE_VIDEOS = True

IMAGE_WRITER_PROCESSES = 0
IMAGE_WRITER_THREADS_PER_CAMERA = 4

VCODEC = "h264"
STREAMING_ENCODING = True
ENCODER_QUEUE_MAXSIZE = 30
ENCODER_THREADS = 2

# Review buttons on PS4
BTN_CROSS = 0      # discard
BTN_TRIANGLE = 2   # keep
BTN_OPTIONS = 9    # stop episode / enter review

RESET_TIME_S = 15


def make_robot() -> SOFollower:
    robot_cfg = SOFollowerRobotConfig(
        port=ROBOT_PORT,
        id=ROBOT_ID,
        use_degrees=True,
        cameras={
            CAMERA_NAME: OpenCVCameraConfig(
                index_or_path=CAMERA_DEVICE,
                width=CAMERA_WIDTH,
                height=CAMERA_HEIGHT,
                fps=CAMERA_FPS,
            )
        },
    )
    return SOFollower(robot_cfg)


def make_teleop() -> PS4JointTeleop:
    teleop_cfg = PS4JointTeleopConfig(id="ps4")
    teleop_cfg.device = GAMEPAD_DEVICE
    teleop_cfg.deadzone = 0.08
    teleop_cfg.speed = 35.0
    teleop_cfg.roll_speed = 20.0
    teleop_cfg.gripper_speed = 15.5
    teleop_cfg.loop_hz = float(FPS)
    return PS4JointTeleop(teleop_cfg)

def initial_target_from_observation(obs: dict) -> dict[str, float]:
    return {
        "shoulder_pan.pos": float(obs["shoulder_pan.pos"]),
        "shoulder_lift.pos": float(obs["shoulder_lift.pos"]),
        "elbow_flex.pos": float(obs["elbow_flex.pos"]),
        "wrist_flex.pos": float(obs["wrist_flex.pos"]),
        "wrist_roll.pos": float(obs["wrist_roll.pos"]),
        "gripper.pos": float(obs["gripper.pos"]),
    }


def build_dataset(robot: SOFollower, teleop_action_processor, robot_observation_processor) -> LeRobotDataset:
    dataset_features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_action_processor,
            initial_features=create_initial_features(action=robot.action_features),
            use_videos=USE_VIDEOS,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(observation=robot.observation_features),
            use_videos=USE_VIDEOS,
        ),
    )

    return LeRobotDataset.create(
        repo_id=DATASET_REPO_ID,
        fps=FPS,
        root=DATASET_ROOT,
        robot_type=robot.name,
        features=dataset_features,
        use_videos=USE_VIDEOS,
        image_writer_processes=IMAGE_WRITER_PROCESSES,
        image_writer_threads=IMAGE_WRITER_THREADS_PER_CAMERA * len(robot.cameras),
        batch_encoding_size=1,
        vcodec=VCODEC,
        streaming_encoding=STREAMING_ENCODING,
        encoder_queue_maxsize=ENCODER_QUEUE_MAXSIZE,
        encoder_threads=ENCODER_THREADS,
    )
def reset_phase(
    robot: SOFollower,
    teleop: PS4JointTeleop,
    robot_action_processor,
    robot_observation_processor,
    reset_time_s: float,
    episode_idx: int,
    total_episodes: int,
) -> None:
    print("\nReset phase started.")
    print("Use controller to move robot back. Nothing is being recorded.")

    reset_start_t = time.perf_counter()

    while True:
        loop_start_t = time.perf_counter()

        obs = robot.get_observation()
        _ = robot_observation_processor(obs)

        act = teleop.get_action()
        robot_action_to_send = robot_action_processor((act, obs))

        try:
            robot.send_action(robot_action_to_send)
        except RuntimeError as e:
            print(f"\nRobot send_action failed during reset: {e}")
            if "Overload error" in str(e):
                teleop.latch_gripper_fault()
                print("Gripper overload latched. Release triggers to clear.")

        elapsed_t = time.perf_counter() - reset_start_t
        time_left = max(0.0, reset_time_s - elapsed_t)

        print(
            f"\rReset | Episode {episode_idx}/{total_episodes} | "
            f"time left: {time_left:5.1f}s",
            end="",
            flush=True,
        )

        if elapsed_t >= reset_time_s:
            print("\nReset phase complete.")
            break

        dt_s = time.perf_counter() - loop_start_t
        precise_sleep(max(1.0 / FPS - dt_s, 0.0))

def wait_for_review_choice(teleop: PS4JointTeleop) -> str:
    print("\nReview mode | Triangle=keep | Cross=discard")
    print("Release all review buttons first...")

    # Wait until review buttons are released so we do not reuse an old press.
    while True:
        teleop.update_controller_state()
        if (
            teleop.button_state[BTN_CROSS] == 0
            and teleop.button_state[BTN_TRIANGLE] == 0
            and teleop.button_state[BTN_OPTIONS] == 0
        ):
            break
        time.sleep(0.05)

    triangle_prev = 0
    cross_prev = 0

    while True:
        teleop.update_controller_state()

        triangle_now = teleop.button_state[BTN_TRIANGLE]
        cross_now = teleop.button_state[BTN_CROSS]

        if triangle_now and not triangle_prev:
            print("Keeping episode.")
            return "keep"

        if cross_now and not cross_prev:
            print("Discarding episode.")
            return "discard"

        triangle_prev = triangle_now
        cross_prev = cross_now
        time.sleep(0.05)


def record_episode(
    robot: SOFollower,
    teleop: PS4JointTeleop,
    dataset: LeRobotDataset,
    teleop_action_processor,
    robot_action_processor,
    robot_observation_processor,
    episode_idx: int,
    total_episodes: int,
) -> str:
    print("\nStarting episode.")
    print("Press Options to stop and review.")

    episode_start_t = time.perf_counter()
    prev_options = 0

    # Reset teleop target from live robot state
    obs0 = robot.get_observation()
    teleop.set_initial_target(initial_target_from_observation(obs0))

    while True:
        loop_start_t = time.perf_counter()

        obs = robot.get_observation()
        obs_processed = robot_observation_processor(obs)

        observation_frame = build_dataset_frame(dataset.features, obs_processed, prefix=OBS_STR)

        act = teleop.get_action()
        act_processed = teleop_action_processor((act, obs))
        robot_action_to_send = robot_action_processor((act_processed, obs))

        try:
            robot.send_action(robot_action_to_send)
        except RuntimeError as e:
            print(f"\nRobot send_action failed: {e}")
            if "Overload error" in str(e):
                teleop.latch_gripper_fault()
                print("Gripper overload latched. Release triggers to clear.")
            dt_s = time.perf_counter() - loop_start_t
            precise_sleep(max(1.0 / FPS - dt_s, 0.0))
            continue

        action_frame = build_dataset_frame(dataset.features, act_processed, prefix=ACTION)
        frame = {**observation_frame, **action_frame, "task": TASK}
        dataset.add_frame(frame)

        options_now = teleop.button_state[BTN_OPTIONS]
        if options_now and not prev_options:
            break
        prev_options = options_now

        elapsed_t = time.perf_counter() - episode_start_t
        time_left = max(0.0, EPISODE_TIME_S - elapsed_t)

        print(
            f"\rEpisode {episode_idx}/{total_episodes} | "
            f"time left: {time_left:5.1f}s | "
            f"Options=stop",
            end="",
            flush=True,
        )

        if elapsed_t >= EPISODE_TIME_S:
            print("\nEpisode time reached.")
            break

        dt_s = time.perf_counter() - loop_start_t
        precise_sleep(max(1.0 / FPS - dt_s, 0.0))

    return "Stopped"


def main() -> None:
    robot = make_robot()
    teleop = make_teleop()

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()
    dataset = None
    kept = 0

    try:
        print("Connecting robot...")
        robot.connect(calibrate=False)

        print("Connecting PS4 teleop...")
        teleop.connect()

        print("Creating dataset...")
        dataset = build_dataset(robot, teleop_action_processor, robot_observation_processor)

        with VideoEncodingManager(dataset):
            while kept < NUM_EPISODES_TO_KEEP:
                record_episode(
                    robot=robot,
                    teleop=teleop,
                    dataset=dataset,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    episode_idx=kept + 1,
                    total_episodes=NUM_EPISODES_TO_KEEP,
                )

                reset_phase(
                    robot=robot,
                    teleop=teleop,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    reset_time_s=RESET_TIME_S,
                    episode_idx=kept + 1,
                    total_episodes=NUM_EPISODES_TO_KEEP,
                )

                choice = wait_for_review_choice(teleop)

                if choice == "keep":
                    dataset.save_episode()
                    kept += 1
                    print(f"Saved episode. Total kept: {kept}/{NUM_EPISODES_TO_KEEP}")
                else:
                    dataset.clear_episode_buffer()
                    print(f"Episode discarded. Total kept: {kept}/{NUM_EPISODES_TO_KEEP}")

    finally:
        if dataset is not None:
            print("Finalizing dataset...")
            dataset.finalize()

            if PUSH_TO_HUB:
                print("Pushing dataset to Hugging Face Hub...")
                dataset.push_to_hub(private=PRIVATE)

        if teleop.is_connected:
            teleop.disconnect()

        if robot.is_connected:
            robot.disconnect()

        print("Done.")


if __name__ == "__main__":
    main()