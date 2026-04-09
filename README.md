# Turtlebot Toyohashi

ROS Noetic navigation system for Turtlebot 3 with AprilTag detection and camera-based obstacle avoidance.

## Packages

### Navigation Scripts

- **odometry_navigator.py** - Main waypoint navigation using odometry. Follows a flight plan through waypoints, approaches gates visually using AprilTag detection, and performs automated docking.

- **tag_navigator.py** - AprilTag waypoint mapper. Drive the robot near tags to automatically record their positions (x, y, yaw) into `lab_waypoints.json`.

- **tag_radar.py** - AprilTag radar for detection.

- **tag_waypoint_mapper.py** - Legacy waypoint mapping tool.

### Obstacle Avoidance

- **tb3_camera_avoid/Scripts/obstacle_avoidance.py** - Camera-based obstacle avoidance using Raspberry Pi Camera V2.

## Dependencies

- ROS Noetic
- Turtlebot3 packages
- apriltag_ros
- raspicam_node
- tf (transform library)

## Usage

### Waypoint Mapping
```bash
rosrun <package> tag_navigator.py
```
Drive slowly near each AprilTag and keep it centered in camera for ~2 seconds to save its position.

### Navigation
```bash
roslaunch turtlebot3_bringup burger.launch
rosrun <package> odometry_navigator.py
```

## Configuration

Waypoint positions are stored in `lab_waypoints.json`. Edit the flight plan in `odometry_navigator.py` to modify the route.

## License

MIT License