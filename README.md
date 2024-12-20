# Inverted Pendulum Control Project

The control of nonlinear dynamic systems is a core challenge in engineering. Among these, the **inverted pendulum** is a classic example, highlighting the complexities of managing unstable systems with nonlinear dynamics. Due to its inherent open-loop and closed-loop instability, the inverted pendulum serves as a fundamental benchmark in control engineering, extensively used to test and refine both classical and modern control methods. Its significance lies in its practical applications, including **robotic stabilization** and **altitude control** in aerospace systems.

## Problem Statement

The inverted pendulum system, as an **under-actuated mechanical structure** with fewer control inputs than degrees of freedom, presents unique challenges in designing control laws. Controlling this system requires:
- **Stability:** Ensuring the pendulum remains upright.
- **Precision:** Quickly and accurately adjusting the cart's position to maintain balance during movement.

## Objectives

This project aims to:
1. **Model the nonlinear dynamics** of an inverted pendulum system.
2. Demonstrate control techniques, including:
   - **Proportional-Integral-Derivative (PID)** Controller.
   - **Linear Quadratic Regulator (LQR)**.
3. Provide a **MATLAB simulation** to validate the control strategies.

## Approach

1. **System Modeling:** Develop a mathematical model for the inverted pendulum system.
2. **Control Techniques:**
   - Implement PID for basic stabilization.
   - Use LQR for optimal control.
3. **Simulation:** Run simulations in MATLAB to test and compare the effectiveness of the control methods.

## Applications

The inverted pendulum system is widely applicable in:
- **Robotics:** Balancing robots and manipulators.
- **Aerospace:** Altitude and orientation control.
- **Control Theory Research:** As a benchmark for testing and refining advanced control algorithms.

## How to Run the Code

1. 
