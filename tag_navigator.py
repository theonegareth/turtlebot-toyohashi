#!/usr/bin/env python3
import rospy
import actionlib
import json
import os
import math
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from apriltag_ros.msg import AprilTagDetectionArray

DESIRED_DISTANCE = 0.20
K_ANG = 1.0
K_LIN = 0.5
MIN_LIN_SPEED = 0.08

tag_detected = False
last_tag_x = 0.0
last_tag_z = 999.0
current_target_id = None

def load_waypoints():
    file_path = os.path.expanduser("~/catkin_ws/src/lab_waypoints.json")
    if not os.path.exists(file_path):
        print("[ERROR] lab_waypoints.json not found.")
        return None
    with open(file_path, 'r') as f:
        return json.load(f)

def tag_callback(msg):
    global tag_detected, last_tag_x, last_tag_z
    for det in msg.detections:
        if current_target_id is not None and det.id[0] == current_target_id:
            pose = det.pose.pose.pose
            last_tag_x = pose.position.x
            last_tag_z = pose.position.z
            tag_detected = True
            return
    tag_detected = False

def navigate_to_point(client, x, y, yaw):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    q = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_state() == actionlib.GoalStatus.SUCCEEDED

def visual_center_and_dock(vel_pub, listener):
    global tag_detected, last_tag_x, last_tag_z
    rate = rospy.Rate(10)
    
    print("[CENTER] Centering on AprilTag...")
    
    while not rospy.is_shutdown():
        cmd = Twist()
        
        if not tag_detected:
            cmd.angular.z = 0.3
            vel_pub.publish(cmd)
            rate.sleep()
            continue
        
        angle_error = math.atan2(last_tag_x, last_tag_z)
        dist_error = last_tag_z - DESIRED_DISTANCE
        
        cmd.angular.z = -K_ANG * angle_error
        
        if abs(angle_error) > 0.15:
            cmd.linear.x = MIN_LIN_SPEED
        else:
            base_speed = K_LIN * dist_error
            if 0 < base_speed < MIN_LIN_SPEED:
                cmd.linear.x = MIN_LIN_SPEED
            elif 0 > base_speed > -MIN_LIN_SPEED:
                cmd.linear.x = -MIN_LIN_SPEED
            else:
                cmd.linear.x = base_speed
        
        if abs(dist_error) < 0.03 and abs(angle_error) < 0.05:
            print("[DOCK] Parked 20cm in front of tag!")
            vel_pub.publish(Twist())
            return True
        
        cmd.angular.z = max(min(cmd.angular.z, 0.4), -0.4)
        cmd.linear.x = max(min(cmd.linear.x, 0.15), -0.15)
        vel_pub.publish(cmd)
        rate.sleep()
    
    return False

def run_navigator():
    global current_target_id, tag_detected
    
    rospy.init_node('tag_navigator')
    
    waypoints = load_waypoints()
    if not waypoints:
        return
    
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base_client.wait_for_server()
    
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_callback)
    
    listener = tf.TransformListener()
    rospy.sleep(2.0)
    
    print("Tag Navigator ready. Loading flight plan...")
    
    for tag_name, coords in waypoints.items():
        tag_id = int(tag_name.split("_")[1])
        current_target_id = tag_id
        tag_detected = False
        
        print(f"\n{'='*50}")
        print(f"NAVIGATING TO: {tag_name}")
        print(f"{'='*50}")
        
        rospy.sleep(0.5)
        
        print(f"[APPROACH] Driving to waypoint...")
        navigate_to_point(move_base_client, coords['x'], coords['y'], coords['yaw'])
        
        print(f"[CENTER] Visual centering and docking...")
        visual_center_and_dock(vel_pub, listener)
        
        rospy.sleep(2.0)
    
    print("\nAll waypoints completed!")

if __name__ == '__main__':
    try:
        run_navigator()
    except rospy.ROSInterruptException:
        pass