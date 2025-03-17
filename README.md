Code developed by Jialin Tan, QMUL
Student id: 210926082

## Description
   -- This ROS 2 package generates and plots cubic trajectories connecting pairs of randomly generated points. It consists of four nodes that work together to compute and visualize position, velocity, and acceleration trajectories.

## Package Structure
The package contains the following components:
   **`ar_test`**: Main package containing the nodes for generating points, computing trajectories, and plotting.
   **`ar_interface`**: Package containing custom message and service definitions.

## Nodes
1. **`points_generator.py`**:
   - Generates random initial and final positions, velocities, and times.
   - Publishes these parameters to the `/cubic_traj_params` topic.

2. **`cubic_traj_planner.py`**:
   - Subscribes to `/cubic_traj_params`.
   - Calls the `compute_cubic_traj` service to compute the coefficients of the cubic polynomial.
   - Publishes the coefficients and time parameters to the `/cubic_traj_coeffs` topic.

3. **`compute_cubic_coeffs.py`**:
   - Provides the `compute_cubic_traj` service.
   - Computes the coefficients of the cubic polynomial based on the input parameters.

4. **`plot_cubic_traj.py`**:
   - Subscribes to `/cubic_traj_coeffs`.
   - Computes and publishes position, velocity, and acceleration trajectories.
   - Publishes to `/position_trajectory`, `/velocity_trajectory`, and `/acceleration_trajectory` topics.


## Installation:
   - access to workspace: cd:~/your_ros2_ws
   - package building: colcon build --packages-select ar_test
   - source the setup file: source install/setup.bash


##Usage:
   - Run: launch ar_test cubic_traj_gen.launch.py
   - Run: launch rqt in a new terminal. Then goes to Plugins->Visualization->Plot to observe the plotting of data or Plugins->Inrospection->Node Graph to check the structure.
