# Turtlebot Toyohashi

ROS Noetic navigation system for TurtleBot3 with AprilTag-based waypoint mapping, tag-guided docking, and camera-based obstacle avoidance.

## Packages

### Navigation And Mapping Scripts

- **autonomous_mapper_dock.py** - Combined exploration and docking node. Follows the right wall, detects unvisited AprilTags, docks to them, and records waypoint poses.

- **autonomous_mapper.py** - Autonomous wall-following mapper that explores the perimeter and records AprilTag waypoints.

- **odometry_navigator.py** - Main waypoint navigation script using odometry and TF. Flies a fixed gate sequence, approaches gates visually using AprilTag detection, and performs docking plus pushback.

- **nav2_navigator.py** - `move_base`/Nav2-style staging navigator. Drives to a pre-dock pose from `lab_waypoints.json` and hands over to visual docking when the target tag is acquired.

- **tag_navigator.py** - Sequential waypoint navigator that drives to saved AprilTag waypoints and finishes with visual centering/docking.

- **tag_waypoint_mapper.py** - Basic AprilTag waypoint mapper using `odom -> tag_*` TF.

- **tag_waypoint_slam.py** - SLAM-aware waypoint mapper using `map -> tag_*` TF for waypoint persistence.

- **tag_radar.py** - Terminal radar/telemetry view for currently visible and recently seen AprilTags.

### Obstacle Avoidance

- **tb3_camera_avoid/Scripts/obstacle_avoidance.py** - Camera-based obstacle avoidance using Raspberry Pi Camera V2.

### Data And Config

- **lab_waypoints.json** - Persistent AprilTag waypoint database used by the navigation scripts.

- **apriltag_ros_config/** - AprilTag detector launch/config files, including tag family settings and camera/detector parameters.

## Dependencies

- ROS Noetic
- Turtlebot3 packages
- apriltag_ros
- raspicam_node
- tf (transform library)
- OpenCV / `cv_bridge`
- `move_base` action server

## Usage

### Waypoint Mapping
```bash
rosrun turtlebot_toyohashi tag_waypoint_mapper.py
```
Drive slowly near each AprilTag and keep it centered in camera long enough to accumulate confirmation detections before the pose is written to `lab_waypoints.json`.

Alternative mapping modes:

```bash
rosrun turtlebot_toyohashi autonomous_mapper.py
rosrun turtlebot_toyohashi autonomous_mapper_dock.py
rosrun turtlebot_toyohashi tag_waypoint_slam.py
```

### Navigation
```bash
roslaunch turtlebot3_bringup burger.launch
rosrun turtlebot_toyohashi odometry_navigator.py
```

Alternative navigation modes:

```bash
rosrun turtlebot_toyohashi nav2_navigator.py tag_18
rosrun turtlebot_toyohashi tag_navigator.py
rosrun turtlebot_toyohashi tag_radar.py
```

## Configuration

Waypoint positions are stored in `lab_waypoints.json`.

Key tuning points:

- Edit the flight plan in `odometry_navigator.py` to change the gate order.

- Adjust staging, docking, and wall-follow distances in the individual scripts for your lab layout.

- Update `apriltag_ros_config/tags.yaml` and related launch/config files when tag IDs, sizes, or detector settings change.

## License

MIT License
