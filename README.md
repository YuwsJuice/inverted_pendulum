# 1.1 Background
The control of nonlinear dynamic systems is a central challenge in engineering. Among such systems, the inverted pendulum stands as a classic problem, exemplifying the difficulties of controlling inherently unstable systems with nonlinear dynamics. Characterized by its open-loop and closed-loop instability, the inverted pendulum has become a fundamental benchmark in control engineering. It is widely used to test, evaluate, and refine both classical and contemporary control techniques. 

Its importance in control theory research stems from its practical applications, such as robotic stabilization and altitude control in aerospace. The inverted pendulum system, as an under-actuated mechanical structure with fewer control inputs than degrees of freedom, presents unique challenges in designing control laws. Controlling this system demands not only stability but also the ability to quickly and accurately adjust the position of a cart to keep the pendulum upright during movement.

This project aims to demonstrate the modeling and control of a nonlinear inverted pendulum system with the help of MATLAB simulation. By focusing on the development of a dynamic model and the application of control laws, such as the Proportional-Integral-Derivative (PID) controller and Linear Quadratic Regulator (LQR), this work seeks to provide a comprehensive approach to stabilizing the inverted pendulum system.

# 1.2 Objectives
The goal is to develop a stable inverted pendulum setup by designing a classic PID-controller and a model-based controller to maintain the pendulum in a vertical position around the center of the horizontal transport rail.

### Specific Objectives:
- Model and simulate an inverted pendulum system
- Design and develop an inverted pendulum system
- Apply control methods and techniques to the inverted pendulum system

# 1.3 Scope and Limitations
This study focuses on the modeling and control of a nonlinear single inverted pendulum system, aiming to achieve stabilization with a comparison of PID and LQR control techniques. The scope includes both MATLAB-based simulations and the development of a physical prototype to validate the simulation results. Key performance metrics, such as response time and system stability, will be analyzed to assess the effectiveness of control methods.

### Limitations:
- **Modeling Assumptions:** The mathematical model used for simulation will incorporate ideal conditions, excluding real-world factors such as friction and air resistance. This means that the physical system may exhibit slight deviations from simulated performance.
- **Single Pendulum Focus:** The project will be limited to a single inverted pendulum system and will not explore multi-link or double pendulum dynamics, restricting the findings to simpler configurations.

