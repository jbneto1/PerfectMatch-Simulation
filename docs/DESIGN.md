# Design Document for AMR Simulator Interface

This document describes the high-level design of the AMR Simulator Interface project.

## Overview

The project is divided into three main components:

1. **Simulator Interface:** This component interfaces with the robotic simulator, sending and receiving data via UDP.

2. **Localization:** This component processes sensor data to estimate the robot's position and orientation. It uses the Perfect Match (PM) algorithm for initial matching, and the Extended Kalman Filter (EKF) for more accurate estimation.

3. **AMR Controller:** This component uses the estimated position and orientation to compute the wheel speeds necessary for the robot to reach its target position.

## Detailed Design

(Detailed descriptions of each component and how they interact.)

## Future Work

(Possible future improvements or extensions of the project.)
