#!/usr/bin/env python3
import rospy
import tf
import math
import os
from apriltag_ros.msg import AprilTagDetectionArray

listener = None
known_tags = {}

def get_yaw_degrees(rotation_quaternion):
    """Converts quaternion to Euler yaw, then translates radians to degrees."""
    yaw_rad = tf.transformations.euler_from_quaternion(rotation_quaternion)[2]
    return round(math.degrees(yaw_rad), 1)

def calculate_distance(x, y):
    """Calculates straight-line (Euclidean) distance using Pythagorean theorem."""
    return round(math.sqrt(x**2 + y**2), 3)

def clear_screen():
    """Clears the terminal to create a static dashboard effect."""
    os.system('clear' if os.name == 'posix' else 'cls')

def detection_callback(msg):
    # Update our memory dictionary with any currently visible tags
    for detection in msg.detections:
        tag_id = detection.id[0] 
        tag_name = f"tag_{tag_id}"
        
        try:
            (trans, rot) = listener.lookupTransform("base_footprint", tag_name, rospy.Time(0))
            
            # Save or overwrite the tag's data with the newest information
            known_tags[tag_name] = {
                'dist': calculate_distance(trans[0], trans[1]),
                'bearing': get_yaw_degrees(rot),
                'x': round(trans[0], 3),
                'y': round(trans[1], 3),
                'last_seen': rospy.get_time()
            }
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

def display_radar(event):
    """Redraws the radar screen on a fixed timer, regardless of camera input."""
    clear_screen()
    print("=" * 60)
    print("                 TURTLEBOT APRILTAG RADAR                 ")
    print("=" * 60)
    
    if not known_tags:
        print("\n Scanning environment... No tags detected yet.")
    else:
        current_time = rospy.get_time()
        
        for tag, data in known_tags.items():
            time_since_seen = round(current_time - data['last_seen'], 1)
            
            # If the tag was seen in the last 0.5 seconds, it is actively in view
            if time_since_seen < 0.5:
                status = "ACTIVE"
            else:
                status = f"LAST KNOWN ({time_since_seen}s ago)"
            
            print(f"\n [ {tag.upper()} | Status: {status} ]")
            print("-" * 40)
            print(f" Direct Distance: {data['dist']} meters")
            print(f" Bearing (Angle): {data['bearing']} degrees")
            print(f" Grid Position:   X: {data['x']}m | Y: {data['y']}m")
            
    print("\n" + "=" * 60)
    print(" Press Ctrl+C to exit radar mode.")

def run_radar():
    global listener
    rospy.init_node('tag_radar_monitor')
    listener = tf.TransformListener()
    rospy.sleep(1.0) 
    
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, detection_callback)
    
    # Update the dashboard at 5 Hz (every 0.2 seconds)
    rospy.Timer(rospy.Duration(0.2), display_radar)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        run_radar()
    except rospy.ROSInterruptException:
        pass