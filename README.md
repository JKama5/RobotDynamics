# # RBE 501 - Final Project: Robot Dynamics

## Overview

This project explores the dynamics and control of a robotic manipulator as part of the RBE 501 course. The objective is to apply theoretical knowledge and computational tools to solve practical robotics problems. The tasks include calculating transformations, implementing motion planning, and analyzing torque relationships. The final demonstration highlights the integration of these concepts into a functioning system.

## Tasks

### Task 1: Transformation and Joint Angles
- Calculate the 4x4 transformation matrices for points **A** and **B**:
  - A = (185 mm, -185 mm, 185 mm)
  - B = (185 mm, 170 mm, 70 mm)
- Determine the sets of joint angles (θ) for both positions with the gripper parallel to the table.
- Analyze the uniqueness of the solutions.
- Compare motor currents for Joint 2 at varying speeds (100%, 80%, 60%, 40%).

### Task 2: Motion Planning with LSPB
- Introduce an obstacle at a height of 200 mm between **A** and **B**.
- Define a waypoint **C** at (185 mm, 0 mm, 240 mm).
- Use Linear Segment with Parabolic Blend (LSPB) to transition from:
  - **A** → **C** → **B** (with a brief stop at **C**).
- Visualize joint angle changes during a 20-second trajectory.

### Task 3: Velocity-Based Control
- Transition to velocity-based control for smooth motion between **A**, **C**, and **B**.
- Maintain a constant velocity at **C** for a seamless transition.
- Update velocity values dynamically to ensure precise path following over 20 seconds.

### Task 4: Torque Estimation
- Use current-based position control to hold the manipulator at **A**.
- Derive torque estimates using current values and the `InverseDynamics.m` simulation from HW4.
- Validate torque calculations with a known wrench applied at the end-effector.
- Infer an unknown wrench based on torque estimates and compare with previous results.

## Final Demonstration
The final demonstration consists of:
1. **Position-Based Control**: Transition directly from **A** to **B**.
2. **Velocity-Based Control**: Navigate the path **A** → **C** → **B** at constant velocity at **C**.
3. **Torque Estimation**: Calculate the applied wrench using sensor data.

## Report
- Provide detailed explanations, equations, and solutions for all tasks.
- Include graphs and figures illustrating the transformations, trajectories, and torque calculations.
- Address challenges encountered and how they were resolved.

## Requirements
- **Hardware**: Robotic manipulator with current and velocity control capabilities.
- **Software**: MATLAB (for `InverseDynamics.m`), simulation tools, and data visualization utilities.

## Instructor
- **Course**: RBE 501 - Robot Dynamics
- **Instructor**: Andre Rosendo

**Report and Presentation Deadline**: Monday, December 11th

---

