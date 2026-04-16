#!/usr/bin/env python3
import rospy
import tf
import json
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray

class TagMapper:
    def __init__(self):
        rospy.init_node('dynamic_tag_mapper')

        # Configuration limits
        self.distance_limit = 1.0  # Max distance in meters to accept a reading
        self.max_count = 100       # Cap the running average weight
        
        # Confidence threshold to filter out ghost tags
        self.required_detections = 5
        self.pending_tags = {}

        # File paths with ROS parameters and default fallbacks
        self.json_path = rospy.get_param("~waypoint_file", os.path.expanduser("~/bnus_ws/src/cam_aprtag/scripts/lab_waypoints.json"))
        self.snapshot_dir = rospy.get_param("~snapshot_dir", os.path.expanduser("~/bnus_ws/src/cam_aprtag/scripts/snapshots/"))

        if not os.path.exists(self.snapshot_dir):
            os.makedirs(self.snapshot_dir)

        # State variables
        self.saved_waypoints = {}
        self.latest_image = None
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        self.load_waypoints()
        rospy.on_shutdown(self.save_waypoints)

        rospy.sleep(1.0)

        # Subscribers
        rospy.Subscriber("/tag_detections_image", Image, self.image_callback)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_callback)

        rospy.loginfo("Tag Mapper running. Exploring tags...")

    def load_waypoints(self):
        if os.path.exists(self.json_path):
            try:
                with open(self.json_path, 'r') as f:
                    self.saved_waypoints = json.load(f)
                rospy.loginfo("Loaded {} existing waypoints.".format(len(self.saved_waypoints)))
            except json.JSONDecodeError:
                rospy.logwarn("JSON corrupted. Starting fresh.")
                self.saved_waypoints = {}
        else:
            rospy.loginfo("No existing JSON found. Starting new session.")

    def save_waypoints(self):
        rospy.loginfo("Saving waypoints to JSON...")
        with open(self.json_path, 'w') as f:
            json.dump(self.saved_waypoints, f, indent=4)
        rospy.loginfo("Save complete.")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CV Bridge Error: {}".format(e))

    def tag_callback(self, msg):
        if not msg.detections:
            return

        for detection in msg.detections:
            tag_id = detection.id[0]
            tag_name = "tag_{}".format(tag_id)

            # Get depth distance from camera to tag
            dist = detection.pose.pose.pose.position.z
            if dist > self.distance_limit:
                continue

            try:
                timestamp = msg.header.stamp
                self.listener.waitForTransform("odom", tag_name, timestamp, rospy.Duration(0.05))
                (trans, rot) = self.listener.lookupTransform("odom", tag_name, timestamp)

                curr_x, curr_y = trans[0], trans[1]
                curr_yaw = tf.transformations.euler_from_quaternion(rot)[2]

                if tag_name not in self.saved_waypoints:
                    # Increment pending count instead of saving immediately
                    self.pending_tags[tag_name] = self.pending_tags.get(tag_name, 0) + 1
                    
                    if self.pending_tags[tag_name] >= self.required_detections:
                        self.saved_waypoints[tag_name] = {
                            "x": curr_x,
                            "y": curr_y,
                            "yaw": curr_yaw,
                            "count": 1,
                            "closest_distance": round(dist, 3)
                        }
                        self.save_snapshot(tag_name, dist)
                        self.save_waypoints() # Save immediately to prevent data loss
                        rospy.loginfo("New tag officially locked: {}".format(tag_name))

                else:
                    # Update existing tag
                    data = self.saved_waypoints[tag_name]
                    n = data.get("count", 1)

                    data["x"] += (curr_x - data["x"]) / (n + 1)
                    data["y"] += (curr_y - data["y"]) / (n + 1)
                    data["yaw"] += (curr_yaw - data["yaw"]) / (n + 1)

                    if n < self.max_count:
                        data["count"] = n + 1

                    # Check if we have a better view for a snapshot
                    record_dist = data.get("closest_distance", 99.0)
                    if dist < (record_dist - 0.05):
                        data["closest_distance"] = round(dist, 3)
                        self.save_snapshot(tag_name, dist)

            except (tf.Exception):
                pass

    def save_snapshot(self, tag_name, distance):
        if self.latest_image is not None:
            image_filename = os.path.join(self.snapshot_dir, "{}.jpg".format(tag_name))
            cv2.imwrite(image_filename, self.latest_image)
            rospy.loginfo("Saved better snapshot for {} at {}m".format(tag_name, round(distance, 2)))

if __name__ == '__main__':
    try:
        mapper = TagMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass