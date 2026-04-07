#!/usr/bin/env python3
import rospy
import tf
import json
import math
import os
from geometry_msgs.msg import Twist

def load_waypoints():
    # Pointing to the local path on your Raspberry Pi
    file_path = os.path.expanduser("~/catkin_ws/src/lab_waypoints.json")
    if not os.path.exists(file_path):
        print(f"\n[ERROR] File not found at: {file_path}")
        return None
    with open(file_path, 'r') as f:
        return json.load(f)

def drive_to_target(target_name, target_x, target_y, listener, vel_pub):
    print(f"\n[NAVIGATING] Target: {target_name.upper()} (X: {target_x}, Y: {target_y})")
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform("odom", "base_footprint", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
            
        current_x, current_y = trans[0], trans[1]
        current_yaw = tf.transformations.euler_from_quaternion(rot)[2]
        
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        angle_error = target_angle - current_yaw
        while angle_error > math.pi: angle_error -= 2.0 * math.pi
        while angle_error < -math.pi: angle_error += 2.0 * math.pi
        
        cmd = Twist()
        
        if distance < 0.15: # Stop within 15cm
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            vel_pub.publish(cmd)
            print(f"[ARRIVED] Reached {target_name}!")
            return
            
        # P-Controller: Adjust speed based on how far off we are
        if abs(angle_error) > 0.3:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 * angle_error # Turn in place
        else:
            cmd.linear.x = 0.12 # Slow, safe forward speed
            cmd.angular.z = 0.5 * angle_error # Slight steering adjustments
            
        vel_pub.publish(cmd)
        rate.sleep()

def start_patrol():
    rospy.init_node('local_pi_navigator')
    waypoints = load_waypoints()
    if not waypoints: return
        
    listener = tf.TransformListener()
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    rospy.sleep(2.0)
    print("Local Navigation Active. Ensure floor is clear!")
    
    for tag_name, coords in waypoints.items():
        drive_to_target(tag_name, coords['x'], coords['y'], listener, vel_pub)
        print("Waiting 3 seconds...")
        rospy.sleep(3.0)

if __name__ == '__main__':
    try:
        start_patrol()
    except rospy.ROSInterruptException:
        pass