#!/usr/bin/env python3

import cmd

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraAvoidance:

    def __init__(self):
        rospy.init_node('camera_low_obstacle_avoidance')

        self.bridge = CvBridge()
        self.prev_turn = 0   # -1 = right, 1 = left

        self.sub = rospy.Subscriber(
            '/tag_detections_image',
            Image,
            self.callback
        )

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.debug_pub = rospy.Publisher('/debug_image', Image, queue_size=1)
        self.mask_pub = rospy.Publisher('/debug_mask', Image, queue_size=1)
        self.edges_pub = rospy.Publisher('/debug_edges', Image, queue_size=1)
        self.color_pub = rospy.Publisher('/debug_color', Image, queue_size=1)
        
        rospy.loginfo("Low obstacle avoidance started")

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.resize(frame, (320, 240))

        # 1. Preprocessing
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        blur = cv2.GaussianBlur(gray, (3,3), 0)

        # 2. Edge detection (sensitive for small objects)
        v = np.median(blur)

        lower = int(max(0, 0.9 * v))
        upper = int(min(255, 1.8 * v))

        edges = cv2.Canny(blur, lower, upper)
        
        #try hough lines
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=30, maxLineGap=10)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(edges, (x1, y1), (x2, y2), 0, 7)
        edges = cv2.dilate(edges, None, iterations=1)

        
        #edges = cv2.erode(edges, None, iterations=1)
        kernel = np.ones((3,3), np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_OPEN, kernel)
        '''
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(edges)

        min_area = 30  # tune this

        filtered = np.zeros_like(edges)
        
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]

            if area > min_area:
                filtered[labels == i] = 255

        edges = filtered
        horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25,1))
        horizontal_lines = cv2.morphologyEx(edges, cv2.MORPH_OPEN, horizontal_kernel)

        edges = cv2.subtract(edges, horizontal_lines)
        '''
        height, width = edges.shape
        #experimental: using color as well
        floor_sample = frame[int(height*0.92):height, :]
        floor_mean = np.mean(floor_sample, axis=(0,1))

        diff = np.linalg.norm(frame.astype(float) - floor_mean, axis=2)

        diff = diff.astype(np.uint8)

        _, color_mask = cv2.threshold(diff, 50, 255, cv2.THRESH_BINARY)
        
        
        kernel = np.ones((5,5), np.uint8)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel)
        
        #combined = cv2.bitwise_or(edges, color_mask)

        # 3. Focus ONLY on bottom strip (ground area)
        
        #try calculating separately
        
        height_limit = 0.73
        
        #roi = edges[int(height*height_limit):height, :]
        #roi = combined[int(height*height_limit):height, :]
        
        roi_edges = edges[int(height*height_limit):height, :]
        roi_color = color_mask[int(height*height_limit):height, :]
        roi_h, roi_w = roi_edges.shape
        '''
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(roi)

        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] < 40:
                roi[labels == i] = 0
        '''
        # Split regions
        '''
        left = roi[:, :roi_w//3]
        center = roi[:, roi_w//3:2*roi_w//3]
        right = roi[:, 2*roi_w//3:]
        '''
        left_edges = roi_edges[:, :roi_w//3]
        center_edges = roi_edges[:, roi_w//3:2*roi_w//3]
        right_edges = roi_edges[:, 2*roi_w//3:]
        left_color = roi_color[:, :roi_w//3]
        center_color = roi_color[:, roi_w//3:2*roi_w//3]
        right_color = roi_color[:, 2*roi_w//3:]
        # Create vertical weights (closer = bigger weight)
        weights = np.linspace(1.0, 3.0, roi_h).reshape(-1, 1)
        '''
        left_score = np.sum((left > 0) * weights[:, :left.shape[1]])
        center_score = np.sum((center > 0) * weights[:, :center.shape[1]])
        right_score = np.sum((right > 0) * weights[:, :right.shape[1]])
        '''
        left_score_edges = np.sum((left_edges > 0) * weights[:, :left_edges.shape[1]])
        center_score_edges = np.sum((center_edges > 0) * weights[:, :center_edges.shape[1]])
        right_score_edges = np.sum((right_edges > 0) * weights[:, :right_edges.shape[1]])
        left_score_color = np.sum((left_color > 0) * weights[:, :left_color.shape[1]])
        center_score_color = np.sum((center_color > 0) * weights[:, :center_color.shape[1]])
        right_score_color = np.sum((right_color > 0) * weights[:, :right_color.shape[1]])
        
        left_score = left_score_edges + 0.3 * left_score_color
        center_score = center_score_edges + 0.3 * center_score_color
        right_score = right_score_edges + 0.3 * right_score_color
        # 6. Decision making (LOW threshold for small objects)
        cmd = Twist()
        threshold = 800
        
        if center_score < threshold:
            cmd.linear.x = 0.15
            cmd.angular.z = 0.0
            decision = "FORWARD"
        else:
            cmd.linear.x = 0.05

            difference = abs(left_score - right_score)

            if difference < 200:
                # If similar, keep previous direction
                turn = self.prev_turn if self.prev_turn != 0 else 1
            elif left_score < right_score:
                turn = 1
            else:
                turn = -1
            # Smooth turning (prevent flipping)
            if self.prev_turn != 0 and turn != self.prev_turn:
                turn = self.prev_turn  # keep previous direction

            self.prev_turn = turn

            turn_strength = max(2.0, 0.0007 * center_score)
            cmd.angular.z = turn * turn_strength
            if turn == 1:
                decision = "LEFT"
            elif turn == -1:
                decision = "RIGHT"

        self.pub.publish(cmd)
        
        
        #try using state logic to smooth out turning
        # DEBUG LOGS
        '''
        rospy.loginfo("----- LOW OBSTACLE DEBUG -----")
        rospy.loginfo(f"Edges total: {cv2.countNonZero(edges)}")
        rospy.loginfo(f"ROI edges: {cv2.countNonZero(roi)}")
        rospy.loginfo(f"L:{left_score} C:{center_score} R:{right_score}")
        rospy.loginfo(f"Decision: {decision}")
        '''
        
        vis = frame.copy()
        h,w,_ = vis.shape
        
        cv2.rectangle(vis, (0, int(h*height_limit)), (w, h), (255,0,0), 2)
        
        cv2.line(vis, (w//3, int(h*height_limit)), (w//3, h), (0,255,0), 2)
        cv2.line(vis, (2*w//3, int(h*height_limit)), (2*w//3, h), (0,255,0), 2)
        
        cv2.putText(vis, f"L:{left_score:.1f}", (10, int(h*height_limit)+30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.putText(vis, f"C:{center_score:.1f}", (w//3 + 10, int(h*height_limit)+30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.putText(vis, f"R:{right_score:.1f}", (2*w//3 + 10, int(h*height_limit)+30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.putText(vis, f"Decision: {decision}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
        
        arrow_start = (w//2, h-10)
        
        if decision == "FORWARD":
            arrow_end = (w//2, h-60)
        elif decision == "LEFT":
            arrow_end = (w//2 - 40, h-60)
        else:
            arrow_end = (w//2 + 40, h-60)
            
        cv2.arrowedLine(vis, arrow_start, arrow_end, (0,255,255), 5)
        cv2.putText(vis, f"Decision: {decision}", (10, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)
        

        
        debug_msg = self.bridge.cv2_to_imgmsg(vis, "bgr8")
        self.debug_pub.publish(debug_msg)
        
        #debug_mask = self.bridge.cv2_to_imgmsg(combined, "mono8")
        #self.mask_pub.publish(debug_mask)
        
        debug_edges = self.bridge.cv2_to_imgmsg(edges, "mono8")
        self.edges_pub.publish(debug_edges)
        
        debug_color = self.bridge.cv2_to_imgmsg(color_mask, "mono8")
        self.color_pub.publish(debug_color)

if __name__ == '__main__':
    try:
        CameraAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
