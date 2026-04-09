# LeRobot Project – Robot Arm Manipulation with Imitation Learning

Built a low-cost 6-DoF follower robot arm (SO101) using Hugging Face LeRobot and trained an ACT policy to perform a real-world manipulation task.

## Project Overview
This project covers the full pipeline from hardware setup to deployment on a physical robot arm:
- assembled and validated the robot hardware
- created custom PS4 gamepad teleoperation scripts
- recorded demonstration episodes of picking up a block and placing it into a bowl
- trained an ACT imitation learning policy on the collected data
- deployed the trained policy back onto the physical robot

## Current Status
- Collected 50+ high-quality teleoperation episodes
- Trained an ACT policy on the recorded demonstrations
- Successfully deployed the policy to the real robot
- The robot can autonomously pick up a block and place it into a bowl

## Repository Notes
This repo contains custom scripts I wrote on top of the LeRobot workflow, especially for:
- PS4 controller teleoperation
- data recording
- camera checking
- evaluation on the physical robot

Original LeRobot-based scripts are also included for reference.

## Portfolio
Video available on my website: [macblair221.github.io](https://macblair221.github.io/)

## Dependencies
Most dependencies come from the LeRobot package and its setup instructions, along with hardware-specific setup for the robot arm, MotorBus connection, and camera.







