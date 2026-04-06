#!/usr/bin/env python3
import rospy
import tf
import json
import os

# Import the exact message type broadcasted by the camera
from apriltag_ros.msg import AprilTagDetectionArray

# Global variables so our callback function can access them
saved_waypoints = {}
file_path = os.path.expanduser("~/catkin_ws/src/lab_waypoints.json")
listener = None

def detection_callback(msg):
    # This function triggers instantly whenever the camera sees something

    # If the camera sees nothing, just ignore and return
    if not msg.detections:
        return

    # Loop through whatever tags the camera is currently looking at
    for detection in msg.detections:
        # The node stores IDs as a list, we grab the first one
        tag_id = detection.id[0]
        tag_name = f"tag_{tag_id}"

        # If we haven't saved this specific tag yet, calculate its permanent location
        if tag_name not in saved_waypoints:
            try:
                # Ask TF for the absolute path from 'odom' to this newly spotted tag
                (trans, rot) = listener.lookupTransform("odom", tag_name, rospy.Time(0))

                yaw = tf.transformations.euler_from_quaternion(rot)[2]

                saved_waypoints[tag_name] = {
                    "x": round(trans[0], 3),
                    "y": round(trans[1], 3),
                    "yaw": round(yaw, 3)
                }

                print(f"\n[SUCCESS] New tag detected and locked: {tag_name}!")
                print(f"X: {saved_waypoints[tag_name]['x']}m | Y: {saved_waypoints[tag_name]['y']}m")
                print("Updating lab_waypoints.json...")

                with open(file_path, 'w') as f:
                    json.dump(saved_waypoints, f, indent=4)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # The camera saw it, but the 3D math tree needs another millisecond to catch up
                pass

def map_the_tags():
    global listener
    rospy.init_node('dynamic_tag_mapper')

    listener = tf.TransformListener()

    # Give the TF listener one second to build the initial math tree
    rospy.sleep(1.0)

    print("Drive the TurtleBot around. Listening directly to the camera for ANY tag...")

    # Subscribe directly to the vision pipeline
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, detection_callback)

    # Keep the script running forever, waiting for the camera to trigger the callback
    rospy.spin()

if __name__ == '__main__':
    try:
        map_the_tags()
    except rospy.ROSInterruptException:
        pass
