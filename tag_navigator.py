#!/usr/bin/env python3
import rospy
import tf
import json
import math
import os

from apriltag_ros.msg import AprilTagDetectionArray

# ---------------- SETTINGS ----------------
FRAMES_REQUIRED = 20  # Robot must see the tag for 20 frames before saving

saved_waypoints = {}
tag_buffers = {}  # Temporary storage to hold data for averaging
file_path = os.path.expanduser("~/catkin_ws/src/lab_waypoints.json")
listener = None

def get_average(data_list):
    return sum(data_list) / len(data_list)

def detection_callback(msg):
    if not msg.detections:
        return

    for detection in msg.detections:
        tag_id = detection.id[0]
        tag_name = f"tag_{tag_id}"

        # If it's already permanently saved, ignore it
        if tag_name in saved_waypoints:
            continue

        try:
            (trans, rot) = listener.lookupTransform("odom", tag_name, rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]

            # Initialize the buffer if this is the first time seeing it
            if tag_name not in tag_buffers:
                tag_buffers[tag_name] = {"x": [], "y": [], "yaw": []}
                print(f"[MAPPER] Spotted {tag_name}. Gathering telemetry...")

            # Add this frame's math to the buffer
            tag_buffers[tag_name]["x"].append(trans[0])
            tag_buffers[tag_name]["y"].append(trans[1])
            tag_buffers[tag_name]["yaw"].append(yaw)

            # If we have collected enough clean frames, average and save!
            if len(tag_buffers[tag_name]["x"]) >= FRAMES_REQUIRED:
                
                final_x = round(get_average(tag_buffers[tag_name]["x"]), 3)
                final_y = round(get_average(tag_buffers[tag_name]["y"]), 3)
                
                # Averaging angles can be tricky due to pi wrap-around, but for a stationary
                # or slow approach, a simple mean over 20 frames is highly effective at killing noise.
                final_yaw = round(get_average(tag_buffers[tag_name]["yaw"]), 3)

                saved_waypoints[tag_name] = {
                    "x": final_x,
                    "y": final_y,
                    "yaw": final_yaw
                }

                print(f"\n[SUCCESS] Tag Locked and Averaged: {tag_name}")
                print(f"X: {final_x}m | Y: {final_y}m | YAW: {final_yaw}rad")
                
                with open(file_path, 'w') as f:
                    json.dump(saved_waypoints, f, indent=4)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

def map_the_tags():
    global listener
    rospy.init_node('robust_tag_mapper')

    listener = tf.TransformListener()
    rospy.sleep(1.0)

    print("--- ROBUST MAPPING MODE ACTIVE ---")
    print("Drive slowly toward a tag. Keep it centered in the camera for 2 seconds to lock.")

    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, detection_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        map_the_tags()
    except rospy.ROSInterruptException:
        pass