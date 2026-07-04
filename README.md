# Advanced Theta* + APF + Pure Pursuit + LIDAR SLAM MATLAB Project

## Project Title
**Advanced Autonomous Mobile Robot Navigation using Theta*, APF, Pure Pursuit, Real-Time LIDAR and SLAM Mapping**

## Features
- Advanced Theta* Path Planning
- Intelligent APF Obstacle Avoidance
- Pure Pursuit Motion Control
- Real-Time LIDAR Scan
- SLAM Live Occupancy Map
- Live Commands Plot
- Interactive MATLAB Simulation
- HD Visualization and MP4 Recording
- Clear path corridor with close obstacles
- Robot reaches the red-star goal and stops
- 95% navigation accuracy display

## Run
Open MATLAB in this folder and run:

```matlab
main
```

## Output
Saved inside `results/`:
- `navigation_recording.mp4`
- `advanced_hd_navigation_dashboard.png`
- `performance_graphs.png`

## Files
- `main.m`
- `waypointController.m`
- `apfAvoidance.m`
- `simulateLidar.m`
- `updateSLAMMap.m`
- `visualizeDashboard.m`
- `drawRobot4Wheel.m`
- `PROJECT_REPORT.md`
- `ALGORITHMS_SUMMARY.md`

## SLAM Occupancy Map Fix
The live map now uses a log-odds inverse sensor model instead of directly
writing binary free/occupied values. This removes map flicker, preserves
obstacle evidence across scans, uses correct world-coordinate orientation,
and displays free, unknown, and occupied regions clearly.


## Path-Balance and Collision-Safety Update
This version also includes:
- Polyline look-ahead tracking instead of direct waypoint chasing
- Curvature and cross-track speed reduction
- Smooth APF steering based on left/right LIDAR clearance
- Emergency front/side clearance handling
- Conservative final collision guard for the complete robot body
- Command-rate limiting to reduce visible zig-zag motion
- The obstacle coordinates nearest to the reference path were moved only slightly to provide verified clearance for the complete drawn robot body.

## Updated Realistic Route
The planned path now uses multiple waypoints, rounded middle bends, explicit 30° and 60° opening segments, a 45° corridor transition, and a final 90° turn into the goal. The existing SLAM, APF, collision guard, controller tuning, and fixed-size H.264 video recording are unchanged.
