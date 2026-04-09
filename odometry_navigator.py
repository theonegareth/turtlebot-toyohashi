#!/usr/bin/env python3
import rospy
import tf
import json
import math
import os

from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray

# ---------------- SETTINGS ----------------
TAXIWAY_DISTANCE = 0.50  
STAGING_DISTANCE = 0.35  
DESIRED_DISTANCE = 0.25  

K_ANG = 1.0 
K_LIN = 0.5

# Minimum speeds to overcome physical floor friction
MIN_LIN_SPEED = 0.08
MIN_ANG_SPEED = 0.15

# LOWER HUB: Safely clears the right-side posts
HUB_X = 0.51 
HUB_Y = 0.50 

# Tag tracking globals
tag_detected = False
last_tag_x = 0.0
last_tag_z = 999.0 
current_target_id = None
detection_memory_timer = 0 
search_spin_dir = 1.0 

# ---------------- LOAD WAYPOINTS ----------------
def load_waypoints():
    file_path = os.path.expanduser("~/catkin_ws/src/lab_waypoints.json")
    if not os.path.exists(file_path):
        print("[ATC ERROR] Waypoints file not found")
        return None
    with open(file_path, 'r') as f:
        return json.load(f)

# ---------------- TAG CALLBACK ----------------
def tag_callback(msg):
    global tag_detected, last_tag_x, last_tag_z, current_target_id, detection_memory_timer

    for det in msg.detections:
        if det.id[0] == current_target_id:
            pose = det.pose.pose.pose
            last_tag_x = pose.position.x
            last_tag_z = pose.position.z
            tag_detected = True
            detection_memory_timer = 2 
            return
            
    if detection_memory_timer > 0:
        detection_memory_timer -= 1
    else:
        tag_detected = False

# ---------------- CALCULATE TAXIWAY NODES ----------------
def get_gate_nodes(tx, ty, tyaw, center_x, center_y):
    cand_a_x = tx + TAXIWAY_DISTANCE * math.cos(tyaw)
    cand_a_y = ty + TAXIWAY_DISTANCE * math.sin(tyaw)
    cand_b_x = tx - TAXIWAY_DISTANCE * math.cos(tyaw)
    cand_b_y = ty - TAXIWAY_DISTANCE * math.sin(tyaw)

    dist_a = math.hypot(cand_a_x - center_x, cand_a_y - center_y)
    dist_b = math.hypot(cand_b_x - center_x, cand_b_y - center_y)

    vector_sign = 1.0 if dist_a < dist_b else -1.0

    taxi_x = tx + vector_sign * TAXIWAY_DISTANCE * math.cos(tyaw)
    taxi_y = ty + vector_sign * TAXIWAY_DISTANCE * math.sin(tyaw)

    stage_x = tx + vector_sign * STAGING_DISTANCE * math.cos(tyaw)
    stage_y = ty + vector_sign * STAGING_DISTANCE * math.sin(tyaw)

    return (taxi_x, taxi_y), (stage_x, stage_y)

# ---------------- GPS NAVIGATION HELPER (SMOOTH UPGRADE) ----------------
def navigate_to_coordinate(goal_x, goal_y, listener, vel_pub, tolerance=0.10, allow_early_lock=False, stop_at_target=True):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if allow_early_lock and tag_detected:
            print(f"[APPROACH] Visual contact with Gate {current_target_id}...")
            vel_pub.publish(Twist())
            return True 

        try:
            (trans, rot) = listener.lookupTransform("odom", "base_footprint", rospy.Time(0))
        except:
            continue

        cx, cy = trans[0], trans[1]
        cyaw_robot = tf.transformations.euler_from_quaternion(rot)[2]

        dx = goal_x - cx
        dy = goal_y - cy
        dist = math.hypot(dx, dy)

        # THE SMOOTH CORNER FIX: Only brake if commanded to
        if dist < tolerance:
            if stop_at_target:
                vel_pub.publish(Twist())
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
        vel_pub.publish(cmd)
        rate.sleep()
    return False

# ---------------- AUTO-AIM HELPER (RADAR SWEEP) ----------------
def auto_aim(listener, vel_pub):
    rate = rospy.Rate(10)
    print("[SEARCH] Odometry aim failed. Initiating visual radar sweep...")
    
    while not rospy.is_shutdown():
        if tag_detected:
            vel_pub.publish(Twist()) 
            rospy.sleep(0.5) 
            if tag_detected: 
                print("[SEARCH] Target Locked! Handing off to visual glide slope.")
                return
                
        cmd = Twist()
        cmd.angular.z = 0.25 
        vel_pub.publish(cmd)
        rate.sleep()

# ---------------- FINAL VISUAL DOCK (PANIC FIX) ----------------
def visual_dock(vel_pub):
    global tag_detected, last_tag_x, last_tag_z, search_spin_dir

    rate = rospy.Rate(10)
    print("[FINAL APPROACH] Glideslope active...")

    while not rospy.is_shutdown():
        cmd = Twist()

        if not tag_detected:
            if last_tag_z < 0.35:
                print("[DOCK] Gate reached. Parking brakes set.")
                vel_pub.publish(Twist())
                return
                
            cmd.linear.x = 0.0 
            cmd.angular.z = 0.20 * search_spin_dir 
        else:
            angle_error = math.atan2(last_tag_x, last_tag_z)
            dist_error = last_tag_z - DESIRED_DISTANCE
            
            if angle_error > 0: search_spin_dir = -1.0 
            else: search_spin_dir = 1.0  
            
            cmd.angular.z = -K_ANG * angle_error
            
            if abs(angle_error) > 0.15:
                cmd.linear.x = MIN_LIN_SPEED 
            else:
                base_speed = K_LIN * dist_error
                if 0 < base_speed < MIN_LIN_SPEED: cmd.linear.x = MIN_LIN_SPEED
                elif 0 > base_speed > -MIN_LIN_SPEED: cmd.linear.x = -MIN_LIN_SPEED
                else: cmd.linear.x = base_speed

            if abs(dist_error) < 0.03 and abs(angle_error) < 0.05:
                print("[DOCK] Gate reached. Parking brakes set.")
                vel_pub.publish(Twist())
                return

        cmd.angular.z = max(min(cmd.angular.z, 0.25), -0.25)
        cmd.linear.x = max(min(cmd.linear.x, 0.15), -0.15)
        vel_pub.publish(cmd)
        rate.sleep()

# ---------------- PUSHBACK ----------------
def pushback(vel_pub):
    print("[PUSHBACK] Commencing pushback...")
    backup_cmd = Twist()
    backup_cmd.linear.x = -0.15 
    rate = rospy.Rate(10)
    for _ in range(45): 
        vel_pub.publish(backup_cmd)
        rate.sleep()
    vel_pub.publish(Twist())
    rospy.sleep(1.0)

# ---------------- MAIN ATC LOOP ----------------
def start_mission():
    global current_target_id, tag_detected, last_tag_z, search_spin_dir

    rospy.init_node('airport_taxiway_docking')
    waypoints = load_waypoints()
    if not waypoints: return

    listener = tf.TransformListener()
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_callback)

    rospy.sleep(2.0)
    prev_taxi_node = None

    flight_plan = ["tag_7", "tag_18", "tag_10", "tag_0", "tag_4", "tag_2"]

    for name in flight_plan:
        if name not in waypoints: 
            print(f"[ATC ERROR] {name} not found in waypoints. Skipping.")
            continue
            
        data = waypoints[name]
        current_target_id = int(name.split("_")[1])
        
        tag_detected = False
        last_tag_z = 999.0
        search_spin_dir = 1.0 
        
        print(f"\n==============================================")
        print(f"[ATC] FLIGHT PLAN FILED: DESTINATION GATE {current_target_id}")
        print(f"==============================================")

        rospy.sleep(0.5) 
        
        # --- THE DUAL ZONE ARCHITECTURE ---
        if current_target_id == 2:
            active_hub_x = 1.098
            active_hub_y = 1.157
        else:
            active_hub_x = HUB_X
            active_hub_y = HUB_Y

        taxi_node, stage_node = get_gate_nodes(data['x'], data['y'], data['yaw'], active_hub_x, active_hub_y)

        if tag_detected:
            print(f"[VISUAL OVERRIDE] Gate {current_target_id} already in sight! Ignoring drifted odometry.")
            visual_dock(vel_pub)
            rospy.sleep(1.0)
            pushback(vel_pub)
            prev_taxi_node = taxi_node 
            continue 

        # 1. Drive out to the local taxiway
        if prev_taxi_node:
            print(f"[TAXI] Proceeding to local taxiway intersect...")
            navigate_to_coordinate(prev_taxi_node[0], prev_taxi_node[1], listener, vel_pub, allow_early_lock=True) 

            # --- THE CLEAN PATH FIX (SMOOTH CORNERING) ---
            dist_between_gates = math.hypot(taxi_node[0] - prev_taxi_node[0], taxi_node[1] - prev_taxi_node[1])
            
            if dist_between_gates > 0.7:
                print(f"[TAXI] Long distance jump detected. Rounding the corner at Safe Hub...")
                navigate_to_coordinate(active_hub_x, active_hub_y, listener, vel_pub, tolerance=0.25, allow_early_lock=True, stop_at_target=False) 
            else:
                print(f"[TAXI] Next gate is close. Staying on local wall taxiway...")

        # 3. Drive to the destination taxiway
        print(f"[TAXI] Proceeding to Gate {current_target_id} approach vector...")
        navigate_to_coordinate(taxi_node[0], taxi_node[1], listener, vel_pub, allow_early_lock=True)

        # 4. Turn into the gate and drive to the approach marker
        print(f"[APPROACH] Turning into Gate {current_target_id}...")
        found_early = navigate_to_coordinate(stage_node[0], stage_node[1], listener, vel_pub, allow_early_lock=True)
        
        if not found_early:
            print("[APPROACH] Holding at marker. Auto-aiming...")
            auto_aim(listener, vel_pub)

        visual_dock(vel_pub)
        rospy.sleep(1.0)
        pushback(vel_pub)

        prev_taxi_node = taxi_node

if __name__ == '__main__':
    try:
        start_mission()
    except rospy.ROSInterruptException:
        pass