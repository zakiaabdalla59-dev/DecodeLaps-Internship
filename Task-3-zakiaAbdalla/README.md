# Task 3: Autonomous Mobile Robot (AMR) Navigation

This ROS 2 package provides a simulation of a differential-drive Autonomous Mobile Robot (AMR) operating in a Gazebo maze environment. It combines sensor information using an Extended Kalman Filter (EKF), creates a real-time map with Ceres SLAM, calculates optimal routes using a custom 8-connected grid A\* pathfinder, and includes a reflex-level safety deceleration controller.

---

## Navigation System Architecture

The robot navigation system is made up of the following components:

1. **Planar State Estimation & Sensor Fusion (`config/ekf.yaml`)**:

   - Combines IMU orientation data (`/imu/data`) and wheel odometry velocities (`/odom`) using `robot_localization`.
   - Continuously publishes the estimated state on `/odometry/filtered` and broadcasts the `odom` -> `base_footprint` TF transform.

2. **2D Ceres SLAM Mapping (`config/nav2_params.yaml`)**:

   - Processes LiDAR scanner data (`/scan`) through the `slam_toolbox` asynchronous mapper.
   - Builds a 2D occupancy grid `/map` in real time while tracking mapping loop-closures.

3. **Custom A* Global Planner & Lookahead Follower (`amr_navigation/astar_planner_node.py`)**:

   - Subscribes to `/map` and uses array dilation to inflate occupied cells, helping prevent the robot chassis from scraping against walls.
   - Subscribes to `/goal_pose` and `/odometry/filtered` to calculate the shortest grid path using Python's `heapq` and the Manhattan heuristic.
   - Runs a 10 Hz pure-pursuit style steering controller that guides the AMR through the waypoints by publishing raw velocity commands to `/cmd_vel_raw`.

4. **Reflex-Level Obstacle Avoidance Filter (`amr_navigation/dynamic_avoidance_node.py`)**:

   - Subscribes to `/scan` and the planned velocity from `/cmd_vel_raw`.
   - Examines the front $60^\circ$ window to calculate the safe margin error: $\text{error} = d\_{\text{obstacle}} - d\_{\text{safe}}$.
   - **Emergency Stop**: When $\text{error} \le 0$, it immediately publishes $v\_x = 0.0, \omega = 0.0$ to stop the motors and logs `PLC_FAIL_TRIGGER = 1`.
   - **Dynamic Deceleration**: When $\text{error} > 0$, it scales the planned linear velocity using $\tanh(\text{error})$, keeping steering active and logging `PLC_FAIL_TRIGGER = 0`.

---

## How to Install and Run

Since ROS 2 is not natively pre-packaged for Ubuntu 26.04, running the system inside a Docker container with GUI socket forwarding is the most reliable and straightforward method.

### 1. Launch the Simulation

Run the pre-configured launcher script from this directory. It grants authorization to the local X11 display socket and starts the container composition:

```bash
cd "/home/ikram/DecodeLabs Internship/Task-3-IkramAbdiaziz"
./run_docker.sh
