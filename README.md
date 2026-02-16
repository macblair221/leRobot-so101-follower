# LeRobot Project – Real Robot Arm + RL Experiments

Building a low-cost 6-DOF follower arm (SO-ARM101) using Hugging Face LeRobot to train reinforcement learning policies for manipulation tasks.

## Current Status
- LeRobot installed and working in simulation
- PushT environment running with gym-pusht
- Random policy baseline: avg reward ~2.0–2.1 over 10 episodes
- Videos of random behavior saved
- PS4 controller connected to lerobot environment
    - This is how we will teleoperate the arm at first

## Demos
Random actions baseline:

<video src="videos/pusht_random_baseline.mp4" width="640" controls autoplay loop muted></video>

(More demos coming soon: trained policy, real arm experiments)

## Setup
- Clone repo
- Activate environment: `lerobot_env\Scripts\activate` (Windows)
- Install LeRobot: `cd lerobot && pip install -e ".[feetech, pusht]"`

Work in progress — updates coming as hardware arrives!
