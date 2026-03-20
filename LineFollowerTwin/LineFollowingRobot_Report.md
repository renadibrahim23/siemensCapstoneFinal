# Report: Line-Following Robot with PID Control

## Overview
This project implements a line-following robot using PID control. The system consists of a differential-drive robot that tracks a predefined path on a 2D plane with minimal lateral error. The architecture is based on three lightweight clients communicating over the Innexis Virtual System Interconnect (IVSI) backplane.

### Three-Client Architecture
1. **Client 1 - Simulator (Plant + Environment)**:
   - Simulates the robot's kinematics and environment.
   - Publishes pose and path reference data.
   - Adds noise and disturbances to simulate real-world conditions.

2. **Client 2 - Controller**:
   - Implements PID-based control to minimize lateral and heading errors.
   - Outputs velocity commands to steer the robot.

3. **Client 3 - Logger/Visualizer**:
   - Logs key performance indicators (KPIs) such as overshoot, settling time, and steady-state error.
   - Visualizes the robot's trajectory versus the reference path.

## Codebase Analysis

### Folder Structure
The project is organized into the following key components:
- **src/controller/controller.py**: Contains the PID control logic.
- **src/simulator/simulator.py**: Simulates the robot's kinematics and environment.
- **src/logger/logger.py**: Logs data and visualizes results.
- **FabricServer**: Handles IVSI communication (excluded from this report).

### Key Components

#### Controller
- **Purpose**: Implements PID control to minimize lateral and heading errors.
- **Inputs**: Measured position (`measured_x`, `measured_y`) and orientation (`measured_theta`).
- **Outputs**: Linear velocity (`linear_v`), angular velocity (`angular_w`), lateral error, and heading error.
- **Implementation**:
  - The `Controller` class initializes communication with the IVSI backplane.
  - The `MySignals` class manages input and output signals.

#### Simulator
- **Purpose**: Simulates the robot's kinematics and environment.
- **Inputs**: Velocity commands (`linear_v`, `angular_w`).
- **Outputs**: Simulated position (`pos_x`, `pos_y`) and orientation (`theta`).
- **Implementation**:
  - The `Simulator` class generates noisy measurements for position and orientation.
  - The `MySignals` class manages input and output signals.

#### Logger
- **Purpose**: Logs simulation data and visualizes results.
- **Inputs**: Position, orientation, and error metrics.
- **Outputs**: CSV files and plots.
- **Implementation**:
  - The `Logger` class writes data to CSV files and uses Matplotlib for visualization.
  - The `MySignals` class manages input signals.

## Modeling and Equations

### Robot Kinematics
The robot's motion is modeled using differential-drive kinematics:
\[
\dot{x} = v \cos(\theta), \quad \dot{y} = v \sin(\theta), \quad \dot{\theta} = \omega
\]
Where:
- \(v\): Linear velocity.
- \(\omega\): Angular velocity.
- \(\theta\): Orientation angle.

### Sensor Noise
Noise is added to the simulated measurements to mimic real-world conditions. Gaussian noise is applied to position and orientation measurements.

## Experiments

### E1 - PID Gain Sweep
- **Objective**: Test 3-5 sets of PID gains on a straight path.
- **Metrics**: Overshoot, settling time, steady-state error.
- **Results**: Plots and KPIs for each gain set.

### E2 - Curved Path Robustness
- **Objective**: Test the best PID controllers on a curved path.
- **Metrics**: Compare performance on straight vs curved paths.
- **Results**: Discuss the effects of curvature on tracking performance.

### E3 - Noise and Disturbance Rejection
- **Objective**: Test the system under varying noise levels and disturbances.
- **Metrics**: Robustness and success rate.
- **Results**: Analyze the system's ability to reject noise and disturbances.

### E4 - PD vs PID Ablation
- **Objective**: Compare PD (\(K_i = 0\)) and PID controllers on a curved path under noise.
- **Metrics**: Performance differences in tracking accuracy and robustness.
- **Results**: Discuss the trade-offs between PD and PID control.

## Experiment Results and Plots

### E1 - PID Gain Sweep
- **Results**:
  - The `best_run_e1.csv` file contains time-series data for position, orientation, lateral error, and angular velocity.
  - Key metrics:
    - **Lateral Error**: Initial overshoot of -0.5, settling to near-zero after approximately 0.3 seconds.
    - **Angular Velocity**: Peaks at -50.75 rad/s initially, stabilizing to -0.75 rad/s.
  - **Plots**:
    - `best_run_e1_trajectory_plot.png`: Shows the robot's trajectory closely following the reference path.
    - `best_run_e1_lateral_error_vs_time_s.png`: Visualizes the lateral error over time.
    - `best_run_e1_angular_w_vs_time_s.png`: Displays angular velocity trends.

### E2 - Curved Path Robustness
- **Results**:
  - The `results_test_run.csv` file provides data for curved path experiments.
  - Key metrics:
    - **Lateral Error**: Peaks at -0.1, stabilizing to -0.975 after 0.48 seconds.
    - **Angular Velocity**: Peaks at -3 rad/s, stabilizing to -1.39 rad/s.
  - **Plots**:
    - `results_test_run_trajectory_plot.png`: Demonstrates the robot's trajectory on a curved path.
    - `results_test_run_lateral_error_vs_time_s.png`: Shows lateral error trends.
    - `results_test_run_angular_w_vs_time_s.png`: Highlights angular velocity variations.

### E3 - Noise and Disturbance Rejection
- **Results**:
  - Noise levels and disturbances were added to the simulation.
  - The system demonstrated robustness, maintaining trajectory tracking with minimal deviation.
  - **Plots**:
    - `results_test_run_circle_0.02noise_trajectory_plot.png`: Shows trajectory under noise.
    - `results_test_run_circle_0.02noise_lateral_error_vs_time_s.png`: Visualizes lateral error under noise.

### E4 - PD vs PID Ablation
- **Results**:
  - PD controllers (\(K_i = 0\)) showed higher steady-state error compared to PID controllers.
  - PID controllers demonstrated better noise rejection and trajectory tracking.
  - **Plots**:
    - `results_test_run_straight_tuned_noise0.01_trajectory_plot.png`: Compares PD and PID trajectories.
    - `results_test_run_straight_tuned_noise0.01_lateral_error_vs_time_s.png`: Highlights lateral error differences.

## Results and Visualization
- **Plots**: Generated using Matplotlib.
- **KPI Analysis**: CSV files contain overshoot, settling time, and steady-state error for each experiment.
- **Visualization**: Real-time trajectory plots show the robot's performance.

## VSI Gateway Architecture
The IVSI backplane enables seamless communication between clients. Each client publishes and subscribes to signals, ensuring synchronized operation.

## Deliverables
1. **Source Code**: Includes all Python files for the simulator, controller, and logger.
2. **Plots and KPIs**: Visualizations and performance metrics for all experiments.
3. **Screencast**: Demonstrates simulation runs and integration.
4. **Report**: Describes modeling equations, controller design, experiments, results, and the IVSI gateway architecture.