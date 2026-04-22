#!/usr/bin/env python3
import rospy
import tf
import json
import math
import os

from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray

class AirportTaxiwayDocking:
    def __init__(self):
        rospy.init_node('airport_taxiway_docking')

        # ---------------- SETTINGS ----------------
        # File paths with ROS parameters and default fallbacks
        self.json_path = rospy.get_param("~waypoint_file", os.path.expanduser("~/bnus_ws/src/cam_aprtag/scripts/lab_waypoints.json"))
        self.snapshot_dir = rospy.get_param("~snapshot_dir", os.path.expanduser("~/bnus_ws/src/cam_aprtag/scripts/snapshots/"))

        self.taxiway_distance = rospy.get_param("~taxiway_distance", 0.50)
        self.staging_distance = rospy.get_param("~staging_distance", 0.35)
        self.desired_distance = rospy.get_param("~desired_distance", 0.25)
        
        self.k_ang = rospy.get_param("~k_ang", 1.0)
        self.k_lin = rospy.get_param("~k_lin", 0.5)

        # Minimum speeds to overcome physical floor friction
        self.min_lin_speed = rospy.get_param("~min_lin_speed", 0.08)
        self.min_ang_speed = rospy.get_param("~min_ang_speed", 0.15)

        # LOWER HUB: Safely clears the right-side posts
        self.hub_x = rospy.get_param("~hub_x", -2.51)
        self.hub_y = rospy.get_param("~hub_y", -0.80)

        # Tag tracking state variables (formerly globals)
        self.tag_detected = False
        self.last_tag_x = 0.0
        self.last_tag_z = 999.0
        self.current_target_id = None
        self.detection_memory_timer = 0
        self.search_spin_dir = 1.0

        self.listener = tf.TransformListener()
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)

    # ---------------- LOAD WAYPOINTS ----------------
    def load_waypoints(self):
        if not os.path.exists(self.json_path):
            rospy.logerr("[ATC ERROR] Waypoints file not found at " + self.json_path)
            return None
        with open(self.json_path, 'r') as f:
            return json.load(f)

    # ---------------- TAG CALLBACK ----------------
    def tag_callback(self, msg):
        for det in msg.detections:
            if det.id[0] == self.current_target_id:
                pose = det.pose.pose.pose
                self.last_tag_x = pose.position.x
                self.last_tag_z = pose.position.z
                self.tag_detected = True
                self.detection_memory_timer = 2
                return
        
        if self.detection_memory_timer > 0:
            self.detection_memory_timer -= 1
        else:
            self.tag_detected = False

    # ---------------- CALCULATE TAXIWAY NODES ----------------
    def get_gate_nodes(self, tx, ty, tyaw, center_x, center_y):
        cand_a_x = tx + self.taxiway_distance * math.cos(tyaw)
        cand_a_y = ty + self.taxiway_distance * math.sin(tyaw)
        cand_b_x = tx - self.taxiway_distance * math.cos(tyaw)
        cand_b_y = ty - self.taxiway_distance * math.sin(tyaw)

        dist_a = math.hypot(cand_a_x - center_x, cand_a_y - center_y)
        dist_b = math.hypot(cand_b_x - center_x, cand_b_y - center_y)

        vector_sign = 1.0 if dist_a < dist_b else -1.0

        taxi_x = tx + vector_sign * self.taxiway_distance * math.cos(tyaw)
        taxi_y = ty + vector_sign * self.taxiway_distance * math.sin(tyaw)

        stage_x = tx + vector_sign * self.staging_distance * math.cos(tyaw)
        stage_y = ty + vector_sign * self.staging_distance * math.sin(tyaw)

        return (taxi_x, taxi_y), (stage_x, stage_y)

    # ---------------- GPS NAVIGATION HELPER (SMOOTH UPGRADE) ----------------
    def navigate_to_coordinate(self, goal_x, goal_y, tolerance=0.10, allow_early_lock=False, stop_at_target=True):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if allow_early_lock and self.tag_detected:
                rospy.loginfo(f"[APPROACH] Visual contact with Gate {self.current_target_id}...")
                self.vel_pub.publish(Twist())
                return True

            try:
                (trans, rot) = self.listener.lookupTransform("odom", "base_footprint", rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn_throttle(2.0, "[TF ERROR] Waiting for transform from odom to base_footprint...")
                continue

            cx, cy = trans[0], trans[1]
            cyaw_robot = tf.transformations.euler_from_quaternion(rot)[2]

            dx = goal_x - cx
            dy = goal_y - cy
            dist = math.hypot(dx, dy)

            # THE SMOOTH CORNER FIX: Only brake if commanded to
            if dist < tolerance:
                if stop_at_target:
                    self.vel_pub.publish(Twist())
                return False

            angle_err = math.atan2(dy, dx) - cyaw_robot
            while angle_err > math.pi: angle_err -= 2 * math.pi
            while angle_err < -math.pi: angle_err += 2 * math.pi

            cmd = Twist()
            if abs(angle_err) > 0.25:
                cmd.angular.z = 0.6 * angle_err
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = 0.15
                cmd.angular.z = 0.5 * angle_err

            cmd.angular.z = max(min(cmd.angular.z, 0.6), -0.6)
            self.vel_pub.publish(cmd)
            rate.sleep()
        return False

    # ---------------- AUTO-AIM HELPER (RADAR SWEEP) ----------------
    def auto_aim(self):
        rate = rospy.Rate(10)
        rospy.loginfo("[SEARCH] Odometry aim failed. Initiating visual radar sweep...")
        
        while not rospy.is_shutdown():
            if self.tag_detected:
                self.vel_pub.publish(Twist())
                rospy.sleep(0.5)
                if self.tag_detected:
                    rospy.loginfo("[SEARCH] Target Locked! Handing off to visual glide slope.")
                    return
                    
            cmd = Twist()
            cmd.angular.z = 0.25
            self.vel_pub.publish(cmd)
            rate.sleep()

    # ---------------- FINAL VISUAL DOCK (PANIC FIX) ----------------
    def visual_dock(self):
        rate = rospy.Rate(10)
        rospy.loginfo("[FINAL APPROACH] Glideslope active...")

        while not rospy.is_shutdown():
            cmd = Twist()

            if not self.tag_detected:
                if self.last_tag_z < 0.35:
                    rospy.loginfo("[DOCK] Gate reached. Parking brakes set.")
                    self.vel_pub.publish(Twist())
                    return
                    
                cmd.linear.x = 0.0
                cmd.angular.z = 0.20 * self.search_spin_dir
            else:
                angle_error = math.atan2(self.last_tag_x, self.last_tag_z)
                dist_error = self.last_tag_z - self.desired_distance
                
                if angle_error > 0: self.search_spin_dir = -1.0
                else: self.search_spin_dir = 1.0
                
                cmd.angular.z = -self.k_ang * angle_error
                
                if abs(angle_error) > 0.15:
                    cmd.linear.x = self.min_lin_speed
                else:
                    base_speed = self.k_lin * dist_error
                    if 0 < base_speed < self.min_lin_speed: cmd.linear.x = self.min_lin_speed
                    elif 0 > base_speed > -self.min_lin_speed: cmd.linear.x = -self.min_lin_speed
                    else: cmd.linear.x = base_speed

                if abs(dist_error) < 0.03 and abs(angle_error) < 0.05:
                    rospy.loginfo("[DOCK] Gate reached. Parking brakes set.")
                    self.vel_pub.publish(Twist())
                    return

            cmd.angular.z = max(min(cmd.angular.z, 0.25), -0.25)
            cmd.linear.x = max(min(cmd.linear.x, 0.15), -0.15)
            self.vel_pub.publish(cmd)
            rate.sleep()

    # ---------------- PUSHBACK ----------------
    def pushback(self):
        rospy.loginfo("[PUSHBACK] Commencing pushback...")
        backup_cmd = Twist()
        backup_cmd.linear.x = -0.15
        rate = rospy.Rate(10)
        for _ in range(45):
            self.vel_pub.publish(backup_cmd)
            rate.sleep()
        self.vel_pub.publish(Twist())
        rospy.sleep(1.0)

    # ---------------- MAIN ATC LOOP ----------------
    def start_mission(self):
        waypoints = self.load_waypoints()
        if not waypoints: return

        rospy.sleep(2.0)
        prev_taxi_node = None
        flight_plan = ["tag_7", "tag_18", "tag_10", "tag_0", "tag_4", "tag_2"]

        for name in flight_plan:
            if name not in waypoints:
                rospy.logerr(f"[ATC ERROR] {name} not found in waypoints. Skipping.")
                continue
                
            data = waypoints[name]
            self.current_target_id = int(name.split("_")[1])
            
            self.tag_detected = False
            self.last_tag_z = 999.0
            self.search_spin_dir = 1.0
            
            rospy.loginfo("==============================================")
            rospy.loginfo(f"[ATC] FLIGHT PLAN FILED: DESTINATION GATE {self.current_target_id}")
            rospy.loginfo("==============================================")

            rospy.sleep(0.5)
            
            # --- THE DUAL ZONE ARCHITECTURE ---
            if self.current_target_id == 2:
                active_hub_x = 1.098
                active_hub_y = 1.157
            else:
                active_hub_x = self.hub_x
                active_hub_y = self.hub_y

            taxi_node, stage_node = self.get_gate_nodes(data['x'], data['y'], data['yaw'], active_hub_x, active_hub_y)

            if self.tag_detected:
                rospy.loginfo(f"[VISUAL OVERRIDE] Gate {self.current_target_id} already in sight! Ignoring drifted odometry.")
                self.visual_dock()
                rospy.sleep(1.0)
                self.pushback()
                prev_taxi_node = taxi_node
                continue

            # 1. Drive out to the local taxiway
            if prev_taxi_node:
                rospy.loginfo("[TAXI] Proceeding to local taxiway intersect...")
                self.navigate_to_coordinate(prev_taxi_node[0], prev_taxi_node[1], allow_early_lock=True)

                # --- THE CLEAN PATH FIX (SMOOTH CORNERING) ---
                dist_between_gates = math.hypot(taxi_node[0] - prev_taxi_node[0], taxi_node[1] - prev_taxi_node[1])
                
                if dist_between_gates > 0.7:
                    rospy.loginfo("[TAXI] Long distance jump detected. Rounding the corner at Safe Hub...")
                    self.navigate_to_coordinate(active_hub_x, active_hub_y, tolerance=0.25, allow_early_lock=True, stop_at_target=False)
                else:
                    rospy.loginfo("[TAXI] Next gate is close. Staying on local wall taxiway...")

            # 3. Drive to the destination taxiway
            rospy.loginfo(f"[TAXI] Proceeding to Gate {self.current_target_id} approach vector...")
            self.navigate_to_coordinate(taxi_node[0], taxi_node[1], allow_early_lock=True)

            # 4. Turn into the gate and drive to the approach marker
            rospy.loginfo(f"[APPROACH] Turning into Gate {self.current_target_id}...")
            found_early = self.navigate_to_coordinate(stage_node[0], stage_node[1], allow_early_lock=True)
            
            if not found_early:
                rospy.loginfo("[APPROACH] Holding at marker. Auto-aiming...")
                self.auto_aim()

            self.visual_dock()
            rospy.sleep(1.0)
            self.pushback()

            prev_taxi_node = taxi_node

if __name__ == '__main__':
    try:
        navigator = AirportTaxiwayDocking()
        navigator.start_mission()
    except rospy.ROSInterruptException:
        pass