# Line Follower Robot - Digital Twin Project

This repository contains the complete implementation of a **Digital Twin for a Line Follower Robot**, developed as the capstone requirement for the **Siemens Digital Twin Course**. 

The project bridges the gap between physical dynamics and virtual simulation, creating a high-fidelity digital replica of a dual-motor differential drive robot tasked with precise path-following. It features virtual commissioning, control loop tuning, and real-time behavioral synchronization.

---

## 📌 Project Overview

The objective of this project was to design, model, and simulate a line-following robot using Digital Twin (DT) methodologies. By mirroring the physical characteristics of the robot (mass, inertia, motor torque, and sensor arrays) in a virtual environment, the system allows for risk-free testing, control optimization, and predictive performance analysis before physical deployment.

### Key Features
*   **Physics-Based Modeling:** Accurate mathematical representation of DC motor dynamics and kinematics.
*   **Advanced Control Loop:** Implementation and tuning of a Proportional-Integral-Derivative (PID) controller for smooth line tracking.
*   **Transient Response Analysis:** Evaluation of system stability, comparing underdamped, critically damped, and overdamped states.
*   **Virtual Commissioning:** Simulation of sensor-to-actuator latency, surface friction changes, and track anomalies.

---

## 🛠️ System Architecture & Stack

The digital twin pipeline leverages a combination of mathematical modeling, simulation environments, and control scripts:

*   **Kinematics & Dynamics:** State-space representation of a differential drive robot.
*   **Simulation Engine:** Siemens Virtual System Interconnect/ QuestaSim Simulator.
*   **Programming Languages:** Python.
*   **Version Control:** Git & GitHub for iterative development.

---

## 📈 Control System & Mathematical Modeling

The core of the line follower's behavior is driven by a feedback loop analyzing the error offset from the track centerline.

### 1. Kinematics & State-Space
The robot's motion is governed by non-holonomic constraints, mapped via state-space equations to translate linear ($v$) and angular ($\omega$) velocities into individual wheel velocities ($v_L, v_R$).

### 2. PID Tuning & Transient Response
To ensure optimal path-following with minimal oscillation, the PID controller was rigorously tuned. The performance was benchmarked against classic control theory metrics:

*   **Underdamped Response:** Fast rise time but high overshoot ($M_p$) and prolonged settling time ($t_s$), causing the robot to weave aggressively or lose the line.
*   **Overdamped Response:** Smooth, non-oscillatory tracking but sluggish reaction time, leading to failures at sharp corners.
*   **Critically Damped / Optimally Tuned Response:** The ideal balance, yielding minimal overshoot and rapid settling time for stable high-speed tracking.

| Metric | Target Specification | Achieved Results |
| :--- | :--- | :--- |
| **Overshoot ($M_p$)** | < 5% | 
| **Settling Time ($t_s$)** | < 1.2s |
| **Steady-State Error ($e_{ss}$)** | 0 | 0 (Eliminated via Integral Action) |

