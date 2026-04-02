# LeRobot Project – Real Robot Arm + RL Experiments

Building a low-cost 6-DOF follower arm (SO-ARM101) using Hugging Face LeRobot to train reinforcement learning policies for manipulation tasks.

## Current Status
- Collected 100+ episodes of picking up a block and placing it into a bowl using PS4 controller teleoperation
- Used a set of 50 high-quality episodes to help train an ACT policy.
- The model performed well, over 80k steps, we reduced loss from 6.5 to .034. 

## Demos
Random actions baseline:

<video src="videos/pusht_random_baseline.mp4" width="640" controls autoplay loop muted></video>


