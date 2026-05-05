#!/usr/bin/env python3

import rospy
import tf
import json
import os
import cv2
import math
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class AutonomousExplorerDock:
    def __init__(self):
        rospy.init_node("autonomous_explore_dock")

        self.wall_to_follow = "right"

        self.state = "SEARCHING"
        self.visited_tags = set()
        self.active_target_name = None
        self.target_x = 0.0
        self.target_z = 999.0
        self.last_tag_time = rospy.Time.now()

        self.saved_waypoints = {}
        self.latest_image = None
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        rospack = rospkg.RosPack()
        try:
            pkg_path = rospack.get_path('cam_aprtag')
        except rospkg.ResourceNotFound:
            rospy.logerr("Package 'cam_aprtag' not found.")
            pkg_path = os.path.expanduser("~")

        self.json_path = os.path.join(pkg_path, "scripts", "lab_waypoints.json")
        self.snapshot_dir = os.path.join(pkg_path, "scripts", "snapshots")
        
        if not os.path.exists(self.snapshot_dir):
            os.makedirs(self.snapshot_dir)

        self.desired_dist = 0.24
        self.front_limit = 0.24 
        
        self.kp_dist = 1.3 
        self.kd_dist = 0.6
        self.prev_dist_error = 0.0

        self.current_yaw = 0.0
        self.front_val = 10.0
        self.rear_val = 10.0 
        self.side_val = 10.0
        self.corner_val = 10.0
        self.opp_side_val = 10.0 
        self.opp_corner_val = 10.0

        self.front_blocked_count = 0
        self.cooldown_end_time = rospy.Time(0)
        self.state_start_time = rospy.Time.now()
        self.maneuver_start_time = rospy.Time.now()
        self.pivot_goal_yaw = 0.0

        self.start_x = None
        self.start_y = None
        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0
        self.lapped_room = False

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_callback)
        rospy.Subscriber("/tag_detections_image", Image, self.image_callback)

        rospy.on_shutdown(self.save_waypoints)

        rospy.Timer(rospy.Duration(0.1), self.control_loop)

        rospy.loginfo(f"Explorer Started: Tracking the {self.wall_to_follow.upper()} wall. Corner Safety +5cm.")

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            pass

    def get_min_range(self, ranges, center_idx, window):
        num = len(ranges)
        slice_vals = []
        for i in range(-window, window + 1):
            idx = (center_idx + i) % num
            val = ranges[idx]
            if 0.05 < val < 10.0:
                slice_vals.append(val)
        return min(slice_vals) if slice_vals else 10.0

    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        num = len(ranges)

        front_window = int((15.0 / 360.0) * num) 
        self.front_val = self.get_min_range(ranges, 0, front_window)
        
        rear_window = int((20.0 / 360.0) * num) 
        self.rear_val = self.get_min_range(ranges, int(num / 2), rear_window)
        
        side_window = int((20.0 / 360.0) * num) 

        if self.wall_to_follow == "right":
            self.side_val = self.get_min_range(ranges, int(3*num/4), side_window)
            self.corner_val = self.get_min_range(ranges, int(7*num/8), side_window)
            self.opp_side_val = self.get_min_range(ranges, int(num/4), side_window) 
            self.opp_corner_val = self.get_min_range(ranges, int(num/8), side_window) 
        else:
            self.side_val = self.get_min_range(ranges, int(num/4), side_window)
            self.corner_val = self.get_min_range(ranges, int(num/8), side_window)
            self.opp_side_val = self.get_min_range(ranges, int(3*num/4), side_window)
            self.opp_corner_val = self.get_min_range(ranges, int(7*num/8), side_window)

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        quat = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.current_yaw = euler_from_quaternion(quat)[2]

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
            self.lapped_room = True

        if self.lapped_room and self.state == "SEARCHING":
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

            if self.state == "SEARCHING" and tag_name not in self.visited_tags and dist < 3.0:
                if self.active_target_name is not None and self.active_target_name not in self.visited_tags:
                    if tag_name != self.active_target_name:
                        continue 

                rospy.loginfo(f"Target Acquired: {tag_name} at {dist:.2f}m. Initiating dock.")
                self.active_target_name = tag_name
                self.target_x = detection.pose.pose.pose.position.x
                self.target_z = dist
                self.last_tag_time = rospy.Time.now()
                self.state = "DOCKING"

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
            rospy.loginfo_throttle(5, "MISSION COMPLETE. Perimeter swept.")
            return

        if self.state == "SEARCHING":
            effective_front = self.front_val
            if rospy.Time.now() < self.cooldown_end_time:
                effective_front = 10.0

            if self.front_val < 0.22 and self.corner_val < 0.22 and self.side_val < 0.22:
                rospy.logwarn("Trapped in clutter! Initiating recovery.")
                self.state_start_time = rospy.Time.now()
                self.state = "RECOVERY"
                return

            if effective_front < self.front_limit:
                if self.side_val > 0.35 or self.corner_val > 0.35:
                    self.front_blocked_count = 0
                    cmd.linear.x = 0.04
                    cmd.angular.z = -0.6 if self.wall_to_follow == "right" else 0.6
                    self.cmd_pub.publish(cmd)
                    return
                self.front_blocked_count += 1
            else:
                self.front_blocked_count = 0

            if self.front_blocked_count >= 3:
                rospy.loginfo(f"[DEBUG] FRONT BLOCKED: {effective_front:.2f}m. Initiating dynamic backup.")
                self.state = "PRE_PIVOT_REVERSE"
                self.state_start_time = rospy.Time.now()
                self.front_blocked_count = 0
                return

            if effective_front < (self.front_limit + 0.10):
                cmd.linear.x = 0.04
                cmd.angular.z = 0.35 if self.wall_to_follow == "right" else -0.35
                self.cmd_pub.publish(cmd)
                return

            effective_side_val = min(self.side_val, 1.2) 

            dist_error = self.desired_dist - effective_side_val
            d_dist_error = dist_error - self.prev_dist_error
            self.prev_dist_error = dist_error

            corner_push = 0.0
            # INCREASED: Corner threshold +5cm to 0.23m
            if self.corner_val < 0.23:
                 corner_push = 0.3  

            opp_push = 0.0
            # INCREASED: Opposite corner threshold +5cm to 0.23m
            if self.opp_side_val < 0.23 or self.opp_corner_val < 0.23:
                 opp_push = -0.3 if self.wall_to_follow == "right" else 0.3

            dist_correction = (self.kp_dist * dist_error) + (self.kd_dist * d_dist_error) + corner_push + opp_push

            if self.wall_to_follow == "left":
                dist_correction = -dist_correction

            if opp_push == 0.0 and self.side_val > 0.6:
                if self.opp_side_val < 0.80:
                    corridor_error = 0.35 - self.opp_side_val
                    if self.wall_to_follow == "right":
                        dist_correction = -(self.kp_dist * corridor_error)
                    else:
                        dist_correction = (self.kp_dist * corridor_error)
                else:
                    if self.wall_to_follow == "right":
                        dist_correction -= 0.35  
                    else:
                        dist_correction += 0.35  

            cmd.linear.x = 0.08
            if self.wall_to_follow == "right":
                cmd.angular.z = max(min(dist_correction, 0.5), -0.6)
            else:
                cmd.angular.z = max(min(dist_correction, 0.6), -0.5)

        elif self.state == "PRE_PIVOT_REVERSE":
            elapsed = (rospy.Time.now() - self.state_start_time).to_sec()
            
            if self.front_val < 0.40 and elapsed < 3.0 and self.rear_val > 0.15:
                cmd.linear.x = -0.06
                cmd.angular.z = 0.0
            else:
                turn_rads = (math.pi / 2.0) if self.wall_to_follow == "right" else -(math.pi / 2.0)
                self.pivot_goal_yaw = self.normalize_angle(self.current_yaw + turn_rads)
                self.state = "PIVOTING"

        elif self.state == "PIVOTING":
            error = self.normalize_angle(self.pivot_goal_yaw - self.current_yaw)

            if abs(error) <= 0.06:
                rospy.loginfo("[DEBUG] Pivot done. Cool-down active.")
                self.cooldown_end_time = rospy.Time.now() + rospy.Duration(1.5)
                self.state = "SEARCHING"
            else:
                cmd.angular.z = max(min(1.0 * error, 0.35), -0.35)

        elif self.state == "DOCKING":
            if (rospy.Time.now() - self.last_tag_time).to_sec() > 4.0:
                rospy.logwarn(f"Lost sight of {self.active_target_name} for too long. Aborting dock to realign with the wall.")
                self.state_start_time = rospy.Time.now()
                self.state = "REVERSING"
                return

            angle_error = math.atan2(self.target_x, self.target_z)
            dist_error = self.target_z - 0.30

            if dist_error < 0.05:
                rospy.loginfo("Dock complete. Capturing safe robot coordinates.")
                self.visited_tags.add(self.active_target_name)

                try:
                    try:
                        (trans, rot) = self.listener.lookupTransform("map", "base_footprint", rospy.Time(0))
                    except tf.Exception:
                        (trans, rot) = self.listener.lookupTransform("map", "base_link", rospy.Time(0))

                    curr_x, curr_y = trans[0], trans[1]
                    curr_yaw = euler_from_quaternion(rot)[2]

                    self.saved_waypoints[self.active_target_name] = {
                        "x": curr_x,
                        "y": curr_y,
                        "yaw": curr_yaw,
                        "distance_to_tag_when_saved": round(self.target_z, 3)
                    }
                    self.save_waypoints()
                    rospy.loginfo(f"Successfully locked safe waypoint for {self.active_target_name}")

                    if self.latest_image is not None:
                        cv2.imwrite(os.path.join(self.snapshot_dir, f"{self.active_target_name}.jpg"), self.latest_image)

                    self.state_start_time = rospy.Time.now()
                    self.state = "REVERSING"

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logwarn_throttle(1.0, f"Waiting for TF to save waypoint: {e}")
                    return 

            else:
                if self.front_val < 0.30:
                    rospy.loginfo("Obstacle directly in dock path! Initiating dynamic evasive backup.")
                    self.state_start_time = rospy.Time.now()
                    self.state = "DOCK_EVASION"
                    return
                
                # INCREASED: Docking corner threshold +5cm to 0.30m
                elif self.corner_val < 0.30:
                    cmd.linear.x = 0.05   
                    cmd.angular.z = 0.3   
                
                # INCREASED: Opposite corner threshold +5cm to 0.30m
                elif self.opp_corner_val < 0.30:
                    cmd.linear.x = 0.05   
                    cmd.angular.z = -0.3  
                    
                else:
                    cmd.angular.z = max(min(-1.2 * angle_error, 0.45), -0.45)
                    
                    if abs(angle_error) > 0.2:
                        cmd.linear.x = 0.03
                    else:
                        cmd.linear.x = max(min(0.6 * dist_error, 0.12), -0.12)

        elif self.state == "DOCK_EVASION":
            elapsed_total = (rospy.Time.now() - self.state_start_time).to_sec()
            
            if self.front_val < 0.40 and elapsed_total < 2.0 and self.rear_val > 0.15:
                cmd.linear.x = -0.06
                cmd.angular.z = 0.0
                self.maneuver_start_time = rospy.Time.now()
            else:
                elapsed_phase = (rospy.Time.now() - self.maneuver_start_time).to_sec()
                
                if elapsed_phase < 0.7:
                    cmd.linear.x = 0.04
                    cmd.angular.z = -0.5 if self.wall_to_follow == "right" else 0.5 
                elif elapsed_phase < 1.7:
                    cmd.linear.x = 0.06
                    cmd.angular.z = 0.2 if self.wall_to_follow == "right" else -0.2 
                else:
                    rospy.loginfo("Evasion maneuver complete. Resuming dock.")
                    self.state = "DOCKING"

        elif self.state == "REVERSING":
            if (rospy.Time.now() - self.state_start_time).to_sec() < 1.5 and self.rear_val > 0.15:
                cmd.linear.x = -0.08
            else:
                rospy.loginfo("Reverse complete. Pivoting left to put obstacle on the right.")
                turn_rads = 1.3 if self.wall_to_follow == "right" else -1.3
                self.pivot_goal_yaw = self.normalize_angle(self.current_yaw + turn_rads)
                self.state = "REALIGNING"

        elif self.state == "REALIGNING":
            error = self.normalize_angle(self.pivot_goal_yaw - self.current_yaw)

            if abs(error) <= 0.08:
                rospy.loginfo("Realignment perfect. Resuming wall tracking.")
                self.prev_dist_error = self.desired_dist - self.side_val
                self.cooldown_end_time = rospy.Time.now() + rospy.Duration(1.0)
                self.state = "SEARCHING"
            else:
                cmd.angular.z = max(min(1.2 * error, 0.5), -0.5)

        elif self.state == "RECOVERY":
            elapsed = (rospy.Time.now() - self.state_start_time).to_sec()
            
            if elapsed < 1.5:
                cmd.linear.x = -0.06 if self.rear_val > 0.15 else 0.0
                cmd.angular.z = 0.0
            elif elapsed < 4.0:
                cmd.linear.x = 0.0
                cmd.angular.z = -1.0 
            else:
                rospy.loginfo("Recovery complete. Resuming search.")
                self.state = "SEARCHING"
                self.cooldown_end_time = rospy.Time.now() + rospy.Duration(1.0)

        self.cmd_pub.publish(cmd)

if __name__ == "__main__":
    try:
        AutonomousExplorerDock()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
