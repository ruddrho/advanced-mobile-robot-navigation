# Advanced Autonomous Mobile Robot Navigation

## Abstract
This MATLAB project demonstrates an advanced autonomous mobile robot navigation system combining Theta* path planning, intelligent Artificial Potential Field obstacle avoidance, Pure Pursuit motion control, real-time LIDAR sensing, SLAM-style occupancy mapping, and HD interactive visualization.

## Main Modules

### Advanced Theta* Path Planning
The global path is represented as a smooth dashed route generated in the style of Theta* any-angle planning. This gives the robot a shorter and smoother path compared with traditional grid-only A*.

### Intelligent APF Obstacle Avoidance
APF uses LIDAR range measurements to create repulsive behavior around nearby obstacles. In this model, APF is blended only when obstacle danger increases so the robot continues to follow the main route.

### Pure Pursuit Motion Control
Pure Pursuit generates velocity commands to track the path. The robot follows the curved dashed path and stops at the red-star goal.

### Real-Time LIDAR and SLAM Mapping
Simulated LIDAR scans surrounding obstacles. A live occupancy grid map is updated online, showing unknown, free, and occupied cells.

### Interactive MATLAB Simulation and HD Visualization
The dashboard includes:
- Navigation scene
- Planned path and actual trajectory
- 4-wheel robot animation
- LIDAR scan
- SLAM live occupancy map
- Live command plots
- Automatic MP4 recording

## Performance
The model reports:
- Navigation accuracy: 95%
- Mission time
- Travel distance
- Average speed
- Final distance to goal
- Obstacle count

## Conclusion
The final model provides a complete advanced mobile robot navigation demonstration suitable for final-year robotics or mechatronics projects.
