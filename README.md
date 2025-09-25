# “Warm Up” Project: Finite State Machines, Wall Following, Obstacle Avoidance  

**Joseph Liu and David Barsoum**  
Computational Robotics, Fall 2025  

---

## 1. Overview  
This warm-up project introduced us to ROS2 and the Neato robots through a series of behavior implementations. We developed six key behaviors: teleoperation, square-path driving, wall following, person tracking, and obstacle avoidance. In addition to these six behaviors, we also developed a finite-state machine that ties obstacle avoidance, wall following, teleoperation, and emergency stop functionality together.  

---

## 2. Teleoperation  

![Teleoperation Demo Placeholder](teleop_demo.gif)  

### 2.1 Overview  
The goal of the “Teleop” behavior is to enable manual control of a Neato’s linear and angular velocity using a standard keyboard interface. Being able to manually control a robot is especially valuable for tasks such as initial testing, manual repositioning, or overriding autonomous behaviors in uncertain environments. Teleop serves as both a debugging tool and a fallback mode, ensuring that the robot can still be safely navigated even if higher-level behaviors fail or produce unexpected results.  

### 2.2 Methodology  
The following keybinds, shown in the table below, were chosen based on the common “WASD” control scheme widely used in video games. This control scheme allows for intuitive direction control that many users are already familiar with.  

| Key    | Linear Velocity | Angular Velocity |
|--------|----------------|------------------|
| w      | +0.1 m/s       | +0.0 rad/s       |
| s      | -0.1 m/s       | +0.0 rad/s       |
| a      | +0.0 m/s       | +0.5 rad/s       |
| d      | +0.0 m/s       | -0.5 rad/s       |
| e      | +0.0 m/s       | 0.0 rad/s        |
| q      | 0.0 m/s        | +0.0 rad/s       |
| space  | 0.0 m/s        | 0.0 rad/s        |
| !      | exit node      | exit node        |

Keystrokes increment linear or angular velocity by fixed steps, allowing for finer control compared to preset velocities. Additional keys reset either linear or angular velocity to zero, and the spacebar clears all velocity. Velocity commands were published over the `/cmd_vel` topic as `Twist()` messages. The “!” key was used to terminate manual control and exit the node, reducing the chance of accidental exits.  

Keystroke inputs were made non-blocking using Python’s `tty` and `termios` modules to read single keypresses without requiring Enter. A separate thread handled key reading so that velocity publishing could continue in parallel.  

### 2.3 Limitations/Future Improvements  
Future improvements could include requiring periodic input to confirm that the user is still in control. Without it, an emergency stop would be triggered. A UI showing current speed, key mappings, and active commands would also improve usability.  

---

## 3. Square Drive  

![Square Drive Demo Placeholder](square_drive_demo.gif)  

### 3.1 Overview  
The goal of the “Square Drive” behavior is to autonomously command a Neato robot to navigate in a square trajectory using odometry-based feedback control.  

### 3.2 Methodology  
The robot was programmed to move through four waypoints forming a square one meter apart. It used proportional control for smooth transitions and relied on odometry from the `/odom` topic for position and orientation.  

![Equation Placeholder](square_drive_equation.png)  

### 3.3 Limitations/Future Improvements  
Future improvements include dynamic waypoint reconfiguration and using more advanced controllers such as PID to improve trajectory accuracy.  

---

## 4. Wall Follow  

![Wall Follow Demo Placeholder](wall_follow_demo.gif)  

### 4.1 Overview  
The wall-following behavior enables the robot to detect, approach, and follow a wall using LIDAR data.  

### 4.2 Methodology  
The robot subscribed to `/scan`, used a limited forward-facing window of ranges, and fit a regression line through the data to detect walls.  

#### 4.2.1 Detecting a Wall  
A wall was detected when the regression’s R² value exceeded a set threshold, ensuring curved objects were not mistaken for walls.  

#### 4.2.2 Approaching and Following the Wall  
The robot corrected heading using wall slope and controlled distance using the perpendicular offset from the wall. The scan window dynamically rotated to remain perpendicular to the detected wall, allowing continuous tracking.  

### 4.3 Limitations/Future Improvements  
One limitation is that the robot may spin in place when searching for walls. Additionally, sharp 90-degree turns may be missed, causing crashes. Future improvements include adaptive scan windows and corner detection.  

---

## 5. Person Follow  

![Person Follow Demo Placeholder](person_follow_demo.gif)  

### 5.1 Overview  
This behavior enables the Neato to follow a person detected via LIDAR.  

### 5.2 Methodology  
The robot filtered points within a 90° forward-facing cone, converted them to Cartesian coordinates, and tracked the mean position as a waypoint. A TF2 transform converted the target into the odom frame.  

### 5.3 Limitations/Future Improvements  
The system may confuse static obstacles for a person and cannot differentiate between multiple moving targets. Future improvements include clustering algorithms such as DBSCAN for object segmentation.  

---

## 6. Obstacle Avoidance  

![Obstacle Avoidance Demo Placeholder](obstacle_avoidance_demo.gif)  

### 6.1 Overview  
This behavior allows the robot to navigate toward a waypoint while avoiding obstacles using potential fields.  

### 6.2 Methodology  
Attractive forces toward the goal and repulsive forces from obstacles were combined to produce a final velocity vector.  

### 6.3 Limitations/Future Improvements  
Local minima may trap the robot, and gains require careful tuning. Future improvements include global planners, adaptive gains, and dynamic obstacle detection.  

---

## 7. Finite State Machine Control  

### 7.1 States & Transitions  
The FSM combined four states: obstacle avoidance, wall following, teleop, and estop.  

![FSM State Diagram Placeholder](fsm_state_diagram.png)  

### 7.2 Nodes and Topic Structure  
Each behavior was isolated in its own node, publishing trigger and velocity topics. The FSM node subscribed to these and published only the active velocity commands to `/cmd_vel`.  

### 7.3 Example Run  
Assume we begin in obstacle avoidance mode. The FSM node publishes `obstacle_avoidance` to `/current_state`. If the Wall Follower detects a wall, it publishes to `/wall_detected`. The FSM then transitions to `wall_following` and publishes commands from `/wall_following_cmd` to `/cmd_vel`.  

### 7.4 RQT Graph  
Our nodes and topics are shown below.  

![RQT Graph Placeholder](rqt_graph.png)  

---

## 8. Conclusion  

### 8.1 Individual Contributions  
Jojo implemented teleoperation, square drive, person following, and obstacle avoidance. David implemented wall following and the finite state machine.  

### 8.2 Takeaways  
We gained practical experience in structuring nodes and topics, handling shared resources safely, and debugging real-world issues. We also learned the importance of thoughtful system architecture and thorough edge-case testing.  

### 8.3 Challenges  
We encountered issues in Gazebo where objects remained collidable after being moved. Early designs allowed multiple nodes to publish velocity commands, leading to unpredictable behavior. This was resolved by centralizing all velocity control in the FSM node. Debugging also showed us the importance of isolating edge cases before integrating.  

### 8.4 Next Steps  
Future improvements include combining wall following and obstacle avoidance into a hybrid state, strengthening wall-following robustness, and refining estop recovery so the robot can resume safely without requiring a restart.  

---

