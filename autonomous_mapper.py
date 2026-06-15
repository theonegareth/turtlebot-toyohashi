#!/usr/bin/env python3

import rospy
import tf
import json
import os
import cv2
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class AutonomousMapper:
    def __init__(self):
        rospy.init_node("autonomous_tag_mapper")

        # --- 1. Mapping Setup ---
        self.distance_limit = 0.75
        self.required_detections = 5
        self.pending_tags = {}
        self.saved_waypoints = {}
        self.latest_image = None
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        default_json = os.path.join(script_dir, "lab_waypoints.json")
        default_snaps = os.path.join(script_dir, "snapshots")
        self.json_path = rospy.get_param("~waypoint_file", os.path.expanduser(default_json))
        self.snapshot_dir = rospy.get_param("~snapshot_dir", os.path.expanduser(default_snaps))
        if not os.path.exists(self.snapshot_dir):
            os.makedirs(self.snapshot_dir)
        if not os.path.exists(os.path.dirname(self.json_path)):
            os.makedirs(os.path.dirname(self.json_path))

        # --- 2. Wall Following Setup ---
        self.state = "SEARCHING"
        self.desired_dist = 0.40
        self.front_limit = 0.49
        self.kp_dist = 1.3
        self.kd_dist = 0.7
        self.prev_dist_error = 0.0
        self.kp_yaw = 2.5
        self.current_yaw = 0.0
        self.target_yaw = None
        self.cooldown_end_time = rospy.Time(0)
        self.front_blocked_count = 0
        self.pivot_goal_yaw = 0.0

        # LiDAR cached values
        self.front_val = 10.0
        self.right_val = 10.0
        self.right_corner = 10.0

        # --- 3. Completion Tracking ---
        self.start_x = None
        self.start_y = None
        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0
        self.mapping_complete = False

        # --- 4. ROS Communications ---
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        # Subscribers
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_callback)
        rospy.Subscriber("/tag_detections_image", Image, self.image_callback)

        rospy.on_shutdown(self.save_waypoints)
        rospy.Timer(rospy.Duration(0.1), self.control_loop)
        rospy.loginfo("Autonomous Mapper Started: Following walls and searching for tags.")

    # ===== MAPPING LOGIC =====

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CV Bridge Error: {}".format(e))

    def tag_callback(self, msg):
        if not msg.detections:
            return

        for detection in msg.detections:
            tag_id = detection.id[0]
            tag_name = "tag_{}".format(tag_id)
            dist = detection.pose.pose.pose.position.z

            if dist > self.distance_limit:
                continue

            try:
                timestamp = msg.header.stamp
                self.listener.waitForTransform("map", tag_name, timestamp, rospy.Duration(0.1))
                (trans, rot) = self.listener.lookupTransform("map", tag_name, timestamp)

                curr_x, curr_y = trans[0], trans[1]
                curr_yaw = euler_from_quaternion(rot)[2]

                if tag_name not in self.saved_waypoints:
                    self.pending_tags[tag_name] = self.pending_tags.get(tag_name, 0) + 1
                    if self.pending_tags[tag_name] >= self.required_detections:
                        self.saved_waypoints[tag_name] = {
                            "x": curr_x, "y": curr_y, "yaw": curr_yaw,
                            "count": 1, "closest_distance": round(dist, 3)
                        }
                        self.save_snapshot(tag_name, dist)
                        rospy.loginfo("FOUND NEW TAG: {}".format(tag_name))
                else:
                    data = self.saved_waypoints[tag_name]
                    n = data["count"]
                    
                    # Outlier rejection to prevent sudden map jumps
                    shift_distance = math.hypot(curr_x - data["x"], curr_y - data["y"])
                    if n >= 5 and shift_distance > 0.15:
                        continue
                    
                    data["x"] += (curr_x - data["x"]) / (n + 1)
                    data["y"] += (curr_y - data["y"]) / (n + 1)
                    data["yaw"] = curr_yaw
                    if n < 100: data["count"] += 1

            except (tf.Exception):
                continue

    def save_snapshot(self, tag_name, distance):
        if self.latest_image is not None:
            filename = os.path.join(self.snapshot_dir, "{}.jpg".format(tag_name))
            cv2.imwrite(filename, self.latest_image)

    def save_waypoints(self):
        try:
            os.makedirs(os.path.dirname(self.json_path), exist_ok=True)
            with open(self.json_path, 'w') as f:
                json.dump(self.saved_waypoints, f, indent=4)
            rospy.loginfo("Waypoints saved to JSON.")
        except IOError as e:
            rospy.logerr(f"Failed to save waypoints: {e}")

    # ===== NAVIGATION LOGIC =====

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        quat = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.current_yaw = euler_from_quaternion(quat)[2]
        if self.target_yaw is None:
            self.target_yaw = self.current_yaw

        # Loop completion tracking
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = curr_x
            self.start_y = curr_y
            self.prev_x = curr_x
            self.prev_y = curr_y
            return

        step_dist = math.hypot(curr_x - self.prev_x, curr_y - self.prev_y)
        self.total_distance += step_dist
        self.prev_x = curr_x
        self.prev_y = curr_y

        dist_to_start = math.hypot(curr_x - self.start_x, curr_y - self.start_y)
        
        if self.total_distance > 3.0 and dist_to_start < 0.4:
            self.mapping_complete = True

    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        num = len(ranges)
        self.front_val = ranges[0] if ranges[0] > 0.05 else 10.0
        self.right_val = ranges[int(3*num/4)] if ranges[int(3*num/4)] > 0.02 else 10.0
        self.right_corner = ranges[int(7*num/8)] if ranges[int(7*num/8)] > 0.02 else 10.0

    def control_loop(self, event):
        if self.mapping_complete:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            rospy.loginfo_throttle(5, "PERIMETER COMPLETE: Returned to start point. Stopping.")
            return

        if self.target_yaw is None:
            return

        cmd = Twist()

        if self.state == "SEARCHING":
            effective_front = self.front_val
            if rospy.Time.now() < self.cooldown_end_time:
                effective_front = 10.0

            if effective_front < self.front_limit:
                self.front_blocked_count += 1
            else:
                self.front_blocked_count = 0

            if self.front_blocked_count >= 3:
                rospy.loginfo("FRONT BLOCKED: Pivoting 90 deg.")
                self.state = "PIVOTING"
                goal_yaw = self.current_yaw + (math.pi / 2.0)
                if goal_yaw > math.pi: goal_yaw -= 2.0 * math.pi
                self.pivot_goal_yaw = goal_yaw
                self.front_blocked_count = 0
                return

            if self.right_corner < 0.18:
                cmd.linear.x = 0.05
                cmd.angular.z = 0.5
            else:
                dist_error = self.desired_dist - self.right_val
                d_error = dist_error - self.prev_dist_error
                self.prev_dist_error = dist_error

                yaw_error = self.target_yaw - self.current_yaw
                if yaw_error > math.pi: yaw_error -= 2*math.pi
                if yaw_error < -math.pi: yaw_error += 2*math.pi

                correction = (self.kp_dist * dist_error) + (self.kd_dist * d_error) + (self.kp_yaw * yaw_error)
                cmd.linear.x = 0.08
                cmd.angular.z = max(min(correction, 0.6), -0.6)

        elif self.state == "PIVOTING":
            error = self.pivot_goal_yaw - self.current_yaw
            if error > math.pi: error -= 2.0 * math.pi
            if error < -math.pi: error += 2.0 * math.pi

            if abs(error) <= 0.06:
                rospy.loginfo("Pivot done. Cool-down active: Ignoring front for 1.5s.")
                self.target_yaw = self.current_yaw
                self.cooldown_end_time = rospy.Time.now() + rospy.Duration(1.5)
                self.state = "SEARCHING"
            else:
                cmd.angular.z = 0.5

        self.cmd_pub.publish(cmd)

if __name__ == "__main__":
    try:
        AutonomousMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
