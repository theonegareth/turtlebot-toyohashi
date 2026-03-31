#!/usr/bin/env python3
import rospy
import tf
from apriltag_ros.msg import AprilTagDetectionArray

listener = None

def get_yaw(rotation_quaternion):
    """Helper function to convert 3D quaternion to a flat 2D compass heading."""
    return round(tf.transformations.euler_from_quaternion(rotation_quaternion)[2], 3)

def detection_callback(msg):
    # If the camera sees nothing, ignore
    if not msg.detections:
        return
        
    for detection in msg.detections:
        tag_id = detection.id[0] 
        tag_name = f"tag_{tag_id}"
        
        try:
            # 1. Tag location relative to odometry (Global Tag Location)
            (trans_global_tag, rot_global_tag) = listener.lookupTransform("odom", tag_name, rospy.Time(0))
            
            # 2. Tag location relative to turtlebot (Local Distance)
            (trans_local_tag, rot_local_tag) = listener.lookupTransform("base_footprint", tag_name, rospy.Time(0))
            
            # 3. Turtlebot location relative to odometry (Global Robot Location)
            (trans_robot, rot_robot) = listener.lookupTransform("odom", "base_footprint", rospy.Time(0))
            
            # --- Print the Dashboard ---
            print(f"\n" + "="*40)
            print(f" TARGET ACQUIRED: {tag_name.upper()} ")
            print("="*40)
            
            print("\n[3] TURTLEBOT CURRENT LOCATION (odom -> base_footprint)")
            print(f"    X: {round(trans_robot[0], 3)}m | Y: {round(trans_robot[1], 3)}m | Facing (Yaw): {get_yaw(rot_robot)} rad")
            
            print("\n[2] LOCAL DISTANCE TO TAG (base_footprint -> tag)")
            print(f"    X (Forward): {round(trans_local_tag[0], 3)}m | Y (Left/Right): {round(trans_local_tag[1], 3)}m")
            
            print("\n[1] ABSOLUTE TAG LOCATION (odom -> tag)")
            print(f"    X: {round(trans_global_tag[0], 3)}m | Y: {round(trans_global_tag[1], 3)}m | Yaw: {get_yaw(rot_global_tag)} rad")
            print("-" * 40)
                
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # The math tree is slightly behind the camera frame, just skip and catch it on the next millisecond
            pass

def run_radar():
    global listener
    rospy.init_node('tag_radar_monitor')
    listener = tf.TransformListener()
    rospy.sleep(1.0) # Let the TF tree initialize
    
    print("Spatial Radar Active. Waiting for visual contact with any AprilTag...")
    
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, detection_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        run_radar()
    except rospy.ROSInterruptException:
        pass
