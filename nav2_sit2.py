#!/usr/bin/env python3

import rospy
import json
import math
import os
import sys
import actionlib

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from apriltag_ros.msg import AprilTagDetectionArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class Nav2Navigator:
    def __init__(self):
        rospy.init_node('nav2_navigator', anonymous=True)

        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.json_path = rospy.get_param("~waypoint_file", os.path.expanduser(os.path.join(script_dir, "lab_waypoints.json")))
        self.staging_distance = 0.85
        self.desired_distance = 0.25
        self.wall_safety_threshold = 0.18
        self.server_wait_timeout = rospy.Duration(10.0)
        self.nav_goal_timeout = rospy.Duration(60.0)
        self.reacquire_timeout = rospy.Duration(8.0)
        self.docking_timeout = rospy.Duration(20.0)
        self.tag_detection_timeout = rospy.Duration(1.0)

        self.k_ang = 1.2
        self.k_lin = 0.6
        self.min_lin_speed = 0.08

        self.tag_detected = False
        self.last_tag_x = 0.0
        self.last_tag_z = 999.0
        self.current_target_id = None
        self.detection_memory_timer = 0
        self.search_spin_dir = 1.0
        self.last_detection_time = rospy.Time(0)

        self.front_val = 10.0
        self.corner_val = 10.0
        self.opp_corner_val = 10.0

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("[ATC] Waiting for move_base...")
        if not self.nav_client.wait_for_server(self.server_wait_timeout):
            rospy.logerr("[ATC] move_base server did not become available within %.1fs.", self.server_wait_timeout.to_sec())
            raise rospy.ROSException("move_base unavailable")
        rospy.loginfo("[ATC] Online.")

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

        side_window = int((20.0 / 360.0) * num)
        self.corner_val = self.get_min_range(ranges, int(7*num/8), side_window)
        self.opp_corner_val = self.get_min_range(ranges, int(num/8), side_window)

    def tag_callback(self, msg):
        for det in msg.detections:
            if not det.id:
                continue
            if det.id[0] == self.current_target_id:
                pose = det.pose.pose.pose
                self.last_tag_x = pose.position.x
                self.last_tag_z = pose.position.z
                self.tag_detected = True
                self.detection_memory_timer = 10
                self.last_detection_time = rospy.Time.now()
                return
        if self.detection_memory_timer > 0: self.detection_memory_timer -= 1
        else: self.tag_detected = False

    def has_recent_detection(self):
        return self.tag_detected and (rospy.Time.now() - self.last_detection_time) <= self.tag_detection_timeout

    def load_target_waypoint(self, target_tag):
        try:
            prefix, tag_suffix = target_tag.split("_", 1)
            if prefix != "tag":
                raise ValueError
            target_id = int(tag_suffix)
        except (AttributeError, ValueError):
            rospy.logerr("[ATC] Invalid target tag '%s'. Expected format tag_<id>.", target_tag)
            return None, None

        try:
            with open(self.json_path, 'r') as f:
                data = json.load(f)
        except FileNotFoundError:
            rospy.logerr("[ATC] Waypoint file not found: %s", self.json_path)
            return None, None
        except json.JSONDecodeError as exc:
            rospy.logerr("[ATC] Failed to parse waypoint file: %s", exc)
            return None, None

        tag_data = data.get(target_tag)
        if not isinstance(tag_data, dict):
            rospy.logerr("[ATC] Target %s not found in waypoint file.", target_tag)
            return None, None

        try:
            tx = float(tag_data['x'])
            ty = float(tag_data['y'])
            tyaw = float(tag_data['yaw'])
        except (KeyError, TypeError, ValueError) as exc:
            rospy.logerr("[ATC] Invalid waypoint data for %s: %s", target_tag, exc)
            return None, None

        return target_id, {'x': tx, 'y': ty, 'yaw': tyaw}

    def force_backup(self, reason="Stuck"):
        rospy.logwarn(f"[RECOVERY] {reason}. Clearing wall inflation...")
        cmd = Twist()
        cmd.linear.x = -0.10
        cmd.angular.z = 0.2 * self.search_spin_dir
        for _ in range(25):
            if rospy.is_shutdown(): break
            self.vel_pub.publish(cmd)
            rospy.sleep(0.1)
        self.vel_pub.publish(Twist())

    def wait_for_handover(self):
        deadline = rospy.Time.now() + self.nav_goal_timeout
        rate = rospy.Rate(5)

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            state = self.nav_client.get_state()
            if (self.has_recent_detection() and self.last_tag_z < 0.9) or state == actionlib.GoalStatus.SUCCEEDED:
                return True
            if state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED, actionlib.GoalStatus.PREEMPTED]:
                self.nav_client.cancel_all_goals()
                self.force_backup("Nav2 Stuck")
                return False
            rate.sleep()

        rospy.logwarn("[ATC] Timed out waiting for navigation handoff.")
        self.nav_client.cancel_all_goals()
        return False

    def reacquire_tag(self):
        deadline = rospy.Time.now() + self.reacquire_timeout
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.has_recent_detection():
                self.vel_pub.publish(Twist())
                return True

            cmd = Twist()
            cmd.angular.z = 0.35
            self.vel_pub.publish(cmd)
            rate.sleep()

        self.vel_pub.publish(Twist())
        rospy.logwarn("[SEARCH] Timed out while trying to reacquire tag.")
        return False

    def visual_dock(self):
        rate = rospy.Rate(10)
        rospy.loginfo("[DOCK] Precision docking engaged with obstacle avoidance.")
        deadline = rospy.Time.now() + self.docking_timeout

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            cmd = Twist()

            if self.front_val < self.wall_safety_threshold:
                rospy.logerr("[DOCK] Front obstacle threshold breached! Reversing.")
                self.force_backup("Wall Proximity")
                return False

            if not self.has_recent_detection():
                cmd.angular.z = 0.25 * self.search_spin_dir
            else:
                angle_error = math.atan2(self.last_tag_x, self.last_tag_z)
                dist_error = self.last_tag_z - self.desired_distance
                self.search_spin_dir = -1.0 if angle_error > 0 else 1.0

                if self.corner_val < 0.30:
                    cmd.linear.x = 0.03
                    cmd.angular.z = 0.6
                elif self.opp_corner_val < 0.30:
                    cmd.linear.x = 0.05
                    cmd.angular.z = -0.3
                else:
                    cmd.angular.z = -self.k_ang * angle_error
                    if abs(angle_error) > 0.2:
                        cmd.linear.x = 0.03
                    else:
                        cmd.linear.x = self.k_lin * dist_error

                cmd.linear.x = max(min(cmd.linear.x, 0.12), -0.12)

                if abs(dist_error) < 0.04 and abs(angle_error) < 0.06:
                    self.vel_pub.publish(Twist())
                    return True

            self.vel_pub.publish(cmd)
            rate.sleep()

        self.vel_pub.publish(Twist())
        return False

    def execute(self, target_tag):
        target_id, tag_data = self.load_target_waypoint(target_tag)
        if tag_data is None:
            return False

        offset_forward = 0.1078
        offset_left = 0.0856

        tx = tag_data['x'] + (offset_forward * math.cos(tag_data['yaw'])) - (offset_left * math.sin(tag_data['yaw']))
        ty = tag_data['y'] + (offset_forward * math.sin(tag_data['yaw'])) + (offset_left * math.cos(tag_data['yaw']))
        tyaw = tag_data['yaw']

        sx = tx - 0.40 * math.cos(tyaw)
        sy = ty - 0.40 * math.sin(tyaw)
        syaw = tyaw

        if sx < -1.5:
             sx, sy, syaw = tx + 0.40 * math.cos(tyaw), ty + 0.40 * math.sin(tyaw), tyaw + math.pi

        self.current_target_id = target_id
        self.tag_detected = False
        self.detection_memory_timer = 0
        self.last_tag_x = 0.0
        self.last_tag_z = 999.0
        self.last_detection_time = rospy.Time(0)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = sx, sy
        q = quaternion_from_euler(0, 0, syaw)
        goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w = q[2], q[3]

        self.nav_client.send_goal(goal)

        handover_ready = self.wait_for_handover()

        self.nav_client.cancel_all_goals()
        self.vel_pub.publish(Twist())
        rospy.sleep(0.6)

        if not handover_ready and not self.has_recent_detection():
            rospy.logwarn("[ATC] Navigation handoff failed.")
            return False

        if not self.has_recent_detection() and not self.reacquire_tag():
            return False

        return self.visual_dock()

if __name__ == '__main__':
    try:
        nav = Nav2Navigator()
        if len(sys.argv) > 1:
            nav.execute(sys.argv[1])
        else:
            rospy.logerr("[ATC] Missing target tag argument.")
    except rospy.ROSInterruptException:
        pass
