#!/usr/bin/env python3

import rospy
import tf
import json
import os
import cv2
import math
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class AutonomousExplorerDock:
    def __init__(self):
        rospy.init_node("autonomous_explore_dock")

        # --- Settings ---
        self.wall_to_follow = "right" 
        self.state = "SEARCHING"
        self.visited_tags = set()
        self.active_target_name = None
        
        # --- Navigation Data ---
        self.target_x = 0.0
        self.target_z = 999.0  
        self.pre_dock_yaw = None 
        self.current_yaw = 0.0
        self.target_yaw = None 
        self.front_val = 10.0  
        self.side_val = 10.0

        # --- Mapping & File Setup ---
        self.saved_waypoints = {}
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        self.json_path = os.path.expanduser("~/bnus_ws/src/cam_aprtag/scripts/lab_waypoints.json")

        # --- Thresholds & Timers ---
        self.desired_dist = 0.40
        self.lidar_obstacle_threshold = 0.35 # Increased slightly for safety
        self.camera_edge_threshold = 2000    # Lowered to be more sensitive to small objects
        self.tag_trigger_dist = 0.85 
        self.stop_threshold = 0.30           # UPDATED: Set to 30cm as requested
        
        self.camera_obstacle_detected = False
        
        # --- Periodic Detection Setup ---
        self.last_camera_check = rospy.Time.now()
        self.camera_check_interval = 10.0  
        self.detection_duration = 1.0     

        # --- ROS Communications ---
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        rospy.Subscriber("/scan_fused", LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=1)
        rospy.Subscriber("/tag_detections_image", Image, self.image_callback, queue_size=1)
    
        rospy.on_shutdown(self.save_waypoints)
        rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        rospy.loginfo("Autonomous Mapper Started: Stop distance set to 30cm.")

    def image_callback(self, msg):
        now = rospy.Time.now()
        
        # If we are docking, we MUST check for obstacles continuously 
        # to avoid objects like in Screenshot from 2026-05-05 12-01-19.png
        is_docking = self.state in ["DOCKING", "BLIND_APPROACH"]
        
        if not is_docking:
            time_since_check = (now - self.last_camera_check).to_sec()
            if time_since_check < self.camera_check_interval:
                self.camera_obstacle_detected = False
                return
            if time_since_check > (self.camera_check_interval + self.detection_duration):
                self.last_camera_check = now
                return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            h, w = gray.shape
            
            # ROI shifted lower to detect objects on the floor near the robot
            roi = gray[int(0.7*h):h, int(0.25*w):int(0.75*w)] 
            edges = cv2.Canny(roi, 50, 150)
            edge_pixels = np.sum(edges) / 255
            
            self.camera_obstacle_detected = edge_pixels > self.camera_edge_threshold
        except CvBridgeError:
            pass

    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        self.front_val = ranges[0] if ranges[0] > 0.05 else 10.0
        num = len(ranges)
        if self.wall_to_follow == "right":
            self.side_val = ranges[int(3*num/4)] if ranges[int(3*num/4)] > 0.02 else 10.0      
        else:
            self.side_val = ranges[int(num/4)] if ranges[int(num/4)] > 0.02 else 10.0          

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        quat = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.current_yaw = euler_from_quaternion(quat)[2]
        if self.target_yaw is None:
            self.target_yaw = self.current_yaw

    def tag_callback(self, msg):
        if not msg.detections: return

        for detection in msg.detections:
            tag_name = f"tag_{detection.id[0]}"
            dist = detection.pose.pose.pose.position.z

            if self.state in ["DOCKING", "BLIND_APPROACH"] and tag_name == self.active_target_name:
                self.target_x = detection.pose.pose.pose.position.x
                self.target_z = dist

            if self.state == "SEARCHING" and tag_name not in self.visited_tags and dist < self.tag_trigger_dist:
                rospy.loginfo(f"Tag {tag_name} found! Moving to save coordinates.")
                self.pre_dock_yaw = self.current_yaw 
                self.active_target_name = tag_name
                self.state = "DOCKING"

    def control_loop(self, event):
        if self.target_yaw is None: return
        cmd = Twist()
        now = rospy.Time.now()
        
        # Check both LiDAR and Camera (Camera is active during docking)
        is_blocked = (self.front_val < self.lidar_obstacle_threshold) or self.camera_obstacle_detected

        if self.state == "SEARCHING":
            if is_blocked:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5 if self.wall_to_follow == "right" else -0.5
            else:
                dist_error = self.desired_dist - self.side_val
                yaw_error = self.target_yaw - self.current_yaw
                while yaw_error > math.pi: yaw_error -= 2*math.pi
                while yaw_error < -math.pi: yaw_error += 2*math.pi
                cmd.linear.x = 0.11
                cmd.angular.z = max(min((1.4 * dist_error) + (0.8 * yaw_error), 0.8), -0.8)

        elif self.state == "DOCKING":
            angle_error = math.atan2(self.target_x, self.target_z)
            if abs(angle_error) < 0.08:
                self.state = "BLIND_APPROACH"
            else:
                # Even in docking, if camera sees an object, stop
                if self.camera_obstacle_detected:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.3 # Slow rotation to try to clear view
                else:
                    cmd.linear.x = 0.02
                    cmd.angular.z = max(min(-0.8 * angle_error, 0.6), -0.6)

        elif self.state == "BLIND_APPROACH":
            # Safety stop if LiDAR or Camera detects an object within 30cm
            if self.front_val <= self.stop_threshold or self.camera_obstacle_detected:
                rospy.loginfo(f"STOPPED AT 30CM: Saving coordinates for {self.active_target_name}")
                self.record_waypoint()
                self.visited_tags.add(self.active_target_name)
                self.state_start_time = now
                self.state = "REVERSING"
            else:
                cmd.linear.x = 0.06 # Precise approach speed
                angle_error = math.atan2(self.target_x, self.target_z)
                cmd.angular.z = max(min(-0.5 * angle_error, 0.3), -0.3)

        elif self.state == "REVERSING":
            if (now - self.state_start_time).to_sec() < 2.5:
                cmd.linear.x = -0.10
            else:
                self.state = "REALIGNING"

        elif self.state == "REALIGNING":
            error = self.pre_dock_yaw - self.current_yaw
            while error > math.pi: error -= 2.0 * math.pi
            while error < -math.pi: error += 2.0 * math.pi
            if abs(error) <= 0.1:
                self.state = "SEARCHING"
            else:
                cmd.angular.z = 0.6 if error > 0 else -0.6

        self.cmd_pub.publish(cmd)

    def record_waypoint(self):
        try:
            (trans, rot) = self.listener.lookupTransform("map", "base_link", rospy.Time(0))
            self.saved_waypoints[self.active_target_name] = {
                "x": trans[0], "y": trans[1], "yaw": euler_from_quaternion(rot)[2]
            }
            self.save_waypoints()
        except Exception as e:
            rospy.logerr(f"TF Error: {e}")

    def save_waypoints(self):
        try:
            with open(self.json_path, 'w') as f:
                json.dump(self.saved_waypoints, f, indent=4)
        except: pass

if __name__ == "__main__":
    try:
        AutonomousExplorerDock()
        rospy.spin()
    except rospy.ROSInterruptException: pass
