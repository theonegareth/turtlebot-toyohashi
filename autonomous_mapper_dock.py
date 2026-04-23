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

class AutonomousExplorerDock:
    def __init__(self):
        rospy.init_node("autonomous_explore_dock")

        # --- State Machine Setup ---
        self.state = "SEARCHING"
        self.visited_tags = set()
        self.active_target_name = None
        self.target_x = 0.0
        self.target_z = 999.0
        self.last_tag_time = rospy.Time.now()
        self.pre_dock_yaw = None 

        # --- Mapping Setup ---
        self.distance_limit = 0.75
        self.required_detections = 5
        self.pending_tags = {}
        self.saved_waypoints = {}
        self.latest_image = None
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        
        self.json_path = os.path.expanduser("~/bnus_ws/src/cam_aprtag/scripts/lab_waypoints.json")
        self.snapshot_dir = os.path.expanduser("~/bnus_ws/src/cam_aprtag/scripts/snapshots/")
        if not os.path.exists(self.snapshot_dir):
            os.makedirs(self.snapshot_dir)

        # --- Wall Following Setup ---
        self.desired_dist = 0.40
        self.front_limit = 0.49
        self.kp_dist = 1.3
        self.kd_dist = 0.7
        self.prev_dist_error = 0.0
        self.kp_yaw = 2.5
        
        self.current_yaw = 0.0
        self.target_yaw = None
        self.front_val = 10.0
        self.right_val = 10.0
        self.right_corner = 10.0
        
        self.front_blocked_count = 0
        self.cooldown_end_time = rospy.Time(0)
        self.state_start_time = rospy.Time.now()
        self.pivot_goal_yaw = 0.0

        # --- Completion Tracking ---
        self.start_x = None
        self.start_y = None
        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0

        # --- ROS Communications ---
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_callback)
        rospy.Subscriber("/tag_detections_image", Image, self.image_callback)

        rospy.on_shutdown(self.save_waypoints)
        
        rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        rospy.loginfo("Explorer Started: Using memory-realignment for wall safety.")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            pass

    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        num = len(ranges)
        self.front_val = ranges[0] if ranges[0] > 0.05 else 10.0
        self.right_val = ranges[int(3*num/4)] if ranges[int(3*num/4)] > 0.02 else 10.0
        self.right_corner = ranges[int(7*num/8)] if ranges[int(7*num/8)] > 0.02 else 10.0

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        quat = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.current_yaw = euler_from_quaternion(quat)[2]
        if self.target_yaw is None:
            self.target_yaw = self.current_yaw

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
        
        # FIX 3: Detect completion regardless of current state
        if self.state != "COMPLETE" and self.total_distance > 3.0 and dist_to_start < 0.4:
            self.state = "COMPLETE"

    def tag_callback(self, msg):
        if not msg.detections:
            return

        for detection in msg.detections:
            tag_id = detection.id[0]
            tag_name = f"tag_{tag_id}"
            dist = detection.pose.pose.pose.position.z

            if self.state == "DOCKING" and tag_name == self.active_target_name:
                self.target_x = detection.pose.pose.pose.position.x
                self.target_z = dist
                self.last_tag_time = rospy.Time.now()

            if self.state == "SEARCHING" and tag_name not in self.visited_tags and dist < 1.0:
                rospy.loginfo(f"Found unvisited tag: {tag_name}. Saving heading and docking.")
                self.pre_dock_yaw = self.target_yaw 
                self.active_target_name = tag_name
                self.target_x = detection.pose.pose.pose.position.x
                self.target_z = dist
                self.last_tag_time = rospy.Time.now()
                self.visited_tags.add(tag_name)
                self.state = "DOCKING"

            if dist <= self.distance_limit:
                try:
                    timestamp = msg.header.stamp
                    self.listener.waitForTransform("map", tag_name, timestamp, rospy.Duration(0.05))
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
                            if self.latest_image is not None:
                                cv2.imwrite(os.path.join(self.snapshot_dir, f"{tag_name}.jpg"), self.latest_image)
                    else:
                        data = self.saved_waypoints[tag_name]
                        n = data["count"]
                        shift_distance = math.hypot(curr_x - data["x"], curr_y - data["y"])
                        if n >= 5 and shift_distance > 0.15:
                            continue
                        
                        data["x"] += (curr_x - data["x"]) / (n + 1)
                        data["y"] += (curr_y - data["y"]) / (n + 1)
                        data["yaw"] = curr_yaw
                        if n < 100: data["count"] += 1
                except (tf.Exception):
                    pass

    def save_waypoints(self):
        with open(self.json_path, 'w') as f:
            json.dump(self.saved_waypoints, f, indent=4)
        rospy.loginfo("Waypoints saved to JSON.")

    def control_loop(self, event):
        cmd = Twist()

        if self.state == "COMPLETE":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            rospy.loginfo_throttle(5, "PERIMETER COMPLETE. Robot halted.")
            return

        if self.target_yaw is None:
            return

        if self.state == "SEARCHING":
            effective_front = self.front_val
            if rospy.Time.now() < self.cooldown_end_time:
                effective_front = 10.0

            if effective_front < self.front_limit:
                self.front_blocked_count += 1
            else:
                self.front_blocked_count = 0

            if self.front_blocked_count >= 3:
                rospy.loginfo(f"[DEBUG] FRONT BLOCKED: {effective_front:.2f}m. Pivoting 90 deg.")
                self.state = "PIVOTING"
                goal_yaw = self.current_yaw + (math.pi / 2.0)
                if goal_yaw > math.pi: goal_yaw -= 2.0 * math.pi
                self.pivot_goal_yaw = goal_yaw
                self.front_blocked_count = 0
                return

            if self.right_corner < 0.18:
                cmd.linear.x = 0.06
                cmd.angular.z = 0.6
            else:
                dist_error = self.desired_dist - self.right_val
                d_dist_error = dist_error - self.prev_dist_error
                self.prev_dist_error = dist_error
                
                yaw_error = self.target_yaw - self.current_yaw
                if yaw_error > math.pi: yaw_error -= 2*math.pi
                if yaw_error < -math.pi: yaw_error += 2*math.pi

                dist_correction = (self.kp_dist * dist_error) + (self.kd_dist * d_dist_error)
                yaw_correction = self.kp_yaw * yaw_error
                
                cmd.linear.x = 0.08
                cmd.angular.z = max(min(dist_correction + yaw_correction, 0.6), -0.6)

        elif self.state == "PIVOTING":
            error = self.pivot_goal_yaw - self.current_yaw
            if error > math.pi: error -= 2.0 * math.pi
            if error < -math.pi: error += 2.0 * math.pi
            
            if abs(error) <= 0.06:
                rospy.loginfo("[DEBUG] Pivot done. Cool-down active: Ignoring front for 1.5s.")
                self.target_yaw = self.current_yaw
                self.cooldown_end_time = rospy.Time.now() + rospy.Duration(1.5)
                self.state = "SEARCHING"
            else:
                cmd.angular.z = 0.6

        elif self.state == "DOCKING":
            if (rospy.Time.now() - self.last_tag_time).to_sec() > 3.0:
                rospy.logwarn("Lost sight of tag. Aborting dock.")
                self.state_start_time = rospy.Time.now()
                self.state = "REVERSING"
                return

            angle_error = math.atan2(self.target_x, self.target_z)
            dist_error = self.target_z - 0.30 

            if dist_error < 0.05:
                rospy.loginfo("Dock complete. Reversing.")
                self.state_start_time = rospy.Time.now()
                self.state = "REVERSING"
            else:
                cmd.angular.z = -1.2 * angle_error
                cmd.linear.x = 0.08 if abs(angle_error) > 0.15 else 0.6 * dist_error
                cmd.linear.x = max(min(cmd.linear.x, 0.12), -0.12)

        elif self.state == "REVERSING":
            if (rospy.Time.now() - self.state_start_time).to_sec() < 2.0:
                cmd.linear.x = -0.12
            else:
                rospy.loginfo("Reverse complete. Realigning to original path.")
                self.state = "REALIGNING"

        elif self.state == "REALIGNING":
            error = self.pre_dock_yaw - self.current_yaw
            if error > math.pi: error -= 2.0 * math.pi
            if error < -math.pi: error += 2.0 * math.pi
            
            if abs(error) <= 0.08:
                rospy.loginfo("Realignment perfect. Resuming wall tracking.")
                self.target_yaw = self.current_yaw
                
                # FIX 1: Reset the derivative memory so it doesn't swerve
                self.prev_dist_error = self.desired_dist - self.right_val
                
                self.cooldown_end_time = rospy.Time.now() + rospy.Duration(1.0)
                self.state = "SEARCHING"
            else:
                # FIX 2: Smooth proportional turning instead of hard bang-bang turning
                cmd.angular.z = max(min(1.5 * error, 0.6), -0.6)

        self.cmd_pub.publish(cmd)

if __name__ == "__main__":
    try:
        AutonomousExplorerDock()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass