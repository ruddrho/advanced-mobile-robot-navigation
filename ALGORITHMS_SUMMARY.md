# Advanced Navigation Algorithms

## 1. Advanced Theta* Path Planning
The dashed reference route represents the global Theta* path. Theta* improves A* by allowing any-angle line-of-sight connections, producing shorter and smoother paths.

## 2. Intelligent APF Obstacle Avoidance
Artificial Potential Field produces local repulsive behavior when LIDAR detects nearby obstacles. In this project, APF is blended gently so it does not pull the robot away from the main path.

## 3. Pure Pursuit Motion Control
Pure Pursuit tracks the planned path using a look-ahead/waypoint target. Linear and angular velocity commands are generated from heading error and distance.

## 4. Real-Time LIDAR & SLAM Mapping
LIDAR rays scan the environment. The SLAM-style occupancy grid updates free and occupied cells online.

## 5. Interactive MATLAB Simulation & HD Visualization
The dashboard shows:
- Main navigation scene
- SLAM occupancy map
- LIDAR scan
- Live velocity commands
- APF danger level
- HD MP4 recording
