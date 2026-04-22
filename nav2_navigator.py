#!/usr/bin/env python3

import rospy
import json
import math
import os
import sys
import actionlib

from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class Nav2Navigator:
    def __init__(self):
        rospy.init_node('nav2_navigator', anonymous=True)

        self.json_path = os.path.expanduser("~/bnus_ws/src/cam_aprtag/scripts/lab_waypoints.json")
        self.staging_distance = 0.55 
        self.desired_distance = 0.25 
        
        self.k_ang = 1.2
        self.k_lin = 0.6
        self.min_lin_speed = 0.08

        self.tag_detected = False
        self.last_tag_x = 0.0
        self.last_tag_z = 999.0
        self.current_target_id = None
        self.detection_memory_timer = 0
        self.search_spin_dir = 1.0

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)

        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("[ATC] Waiting for move_base...")
        self.nav_client.wait_for_server()
        rospy.loginfo("[ATC] Online.")

    def tag_callback(self, msg):
        for det in msg.detections:
            if det.id[0] == self.current_target_id:
                pose = det.pose.pose.pose
                self.last_tag_x = pose.position.x
                self.last_tag_z = pose.position.z
                self.tag_detected = True
                self.detection_memory_timer = 5 
                return
        if self.detection_memory_timer > 0: self.detection_memory_timer -= 1
        else: self.tag_detected = False

    def force_backup(self):
        """Manually backup if the planner gets stuck near the wall."""
        rospy.logwarn("[RECOVERY] Stuck near wall. Manually backing up...")
        cmd = Twist()
        cmd.linear.x = -0.1  # Move backward
        for _ in range(25): # 2.5 seconds at 10Hz
            if rospy.is_shutdown(): break
            self.vel_pub.publish(cmd)
            rospy.sleep(0.1)
        self.vel_pub.publish(Twist()) # Stop

    def visual_dock(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            cmd = Twist()
            if not self.tag_detected:
                if self.last_tag_z < 0.35: return 
                cmd.angular.z = 0.2 * self.search_spin_dir
            else:
                angle_error = math.atan2(self.last_tag_x, self.last_tag_z)
                dist_error = self.last_tag_z - self.desired_distance
                self.search_spin_dir = -1.0 if angle_error > 0 else 1.0
                cmd.angular.z = -self.k_ang * angle_error
                cmd.linear.x = self.min_lin_speed if abs(angle_error) > 0.15 else self.k_lin * dist_error
                cmd.linear.x = max(min(cmd.linear.x, 0.12), -0.12)
                if abs(dist_error) < 0.04 and abs(angle_error) < 0.06:
                    self.vel_pub.publish(Twist())
                    return
            self.vel_pub.publish(cmd)
            rate.sleep()

    def execute(self, target_tag):
        with open(self.json_path, 'r') as f:
            data = json.load(f)
        tag_data = data.get(target_tag)
        self.current_target_id = int(target_tag.split("_")[1])
        
        # Staging Pose Calculation
        tx, ty, tyaw = tag_data['x'], tag_data['y'], tag_data['yaw']
        sx = tx - self.staging_distance * math.cos(tyaw)
        sy = ty - self.staging_distance * math.sin(tyaw)
        syaw = tyaw
        if sx < -0.5: # Boundary logic for Tag 7
            sx, sy, syaw = tx + self.staging_distance * math.cos(tyaw), ty + self.staging_distance * math.sin(tyaw), tyaw + math.pi

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = sx, sy
        q = quaternion_from_euler(0, 0, syaw)
        goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w = q[2], q[3]

        self.nav_client.send_goal(goal)
        
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            state = self.nav_client.get_state()
            if (self.tag_detected and self.last_tag_z < 1.0) or state == 3:
                break
            if state in [4, 5]: # If Aborted or Rejected
                self.nav_client.cancel_all_goals()
                self.force_backup()
                break
            rate.sleep()

        self.nav_client.cancel_all_goals()
        self.vel_pub.publish(Twist())
        rospy.sleep(0.8) # Wait for path clearing in RViz

        if not self.tag_detected:
            # Simple rotation to find tag
            while not self.tag_detected and not rospy.is_shutdown():
                cmd = Twist(); cmd.angular.z = 0.3; self.vel_pub.publish(cmd)
                rospy.sleep(0.1)
        
        self.visual_dock()

if __name__ == '__main__':
    try:
        nav = Nav2Navigator()
        if len(sys.argv) > 1: nav.execute(sys.argv[1])
    except rospy.ROSInterruptException: pass