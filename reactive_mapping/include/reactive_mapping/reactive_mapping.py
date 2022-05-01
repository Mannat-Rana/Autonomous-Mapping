#!/usr/bin/env python
'''
IMPORTS
'''
import rospy 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import actionlib                                                # For move_base client
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal     # Need to create move_base client and publish move_base goal
from visualization_msgs.msg import Marker                       # Need to publish goal pose marker
from nav_msgs.msg import Odometry                               # Listen to /odom topic
from sensor_msgs.msg import LaserScan                           # Listen to /scan topic
from geometry_msgs.msg import Twist
import math
import numpy as np
import random
import time

node_name = "Reactive_Mapping_Node"
scan_topic = "/scan"
pose_topic = "/odom"
vel_topic = "/cmd_vel"
vis_topic = "/marker"
move_ns = "move_base"
safe_distance = 2

class ReactiveMapping:
    
    def __init__(self):
        rospy.init_node(node_name, anonymous=True)
        self.vel_pub = rospy.Publisher(vel_topic, Twist, queue_size=10)
        self.scan_pub = rospy.Publisher('/my_scan', LaserScan, queue_size=10)
        self.vis_pub = rospy.Publisher(vis_topic, Marker, queue_size=10)
        self.pose_estimate = {"x" : 0, "y" : 0, "z" : 0, "t" : 0, "v" : 0, "w" : 0}
        self.goal_client = actionlib.SimpleActionClient(move_ns, MoveBaseAction)
        self.goal_client.wait_for_server()
        self.goal = MoveBaseGoal()
        self.vel_msg = Twist()
        self.left_turn = True
        self.goal.target_pose.header.frame_id = "map"
        self.marker = Marker()
        self.marker.header.frame_id = "odom"
        self.marker.type = 1
        self.marker.id = 0
        self.marker.header.stamp = rospy.Time.now();
        self.marker.ns = "reactive_mapping";
        self.marker.action = 0
        self.marker.pose.orientation.x = 0.0;
        self.marker.pose.orientation.y = 0.0;
        self.marker.pose.orientation.z = 0.0;
        self.marker.pose.orientation.w = 1.0;
        self.marker.scale.x = 0.15;
        self.marker.scale.y = 0.15;
        self.marker.scale.z = 0.15;
        self.marker.color.a = 1.0; 
        self.marker.color.r = 0.5;
        self.marker.color.g = 0.0;
        self.marker.color.b = 0.5;
        self.found_safe_point = False
        self.setpoint_counter = 0
        self.stopped_time = 0
        self.time_since_stopped = time.time()
        self.reached_goal = False
        self.safe_point = [None, None, None, None]
        self.pose_sub = rospy.Subscriber(pose_topic, Odometry, self.pose_callback)
        self.scan_sub = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)
        while not rospy.is_shutdown():
            if self.found_safe_point:
                self.send_goal(self.safe_point)
            else:
                if self.left_turn:
                    turn_dir = 1
                else:
                    turn_dir = -1
                self.vel_msg.angular.z = turn_dir
                while (not self.found_safe_point) and (not rospy.is_shutdown()):
                    rospy.loginfo_throttle(5, "Searching for safe point...")
                    if (self.stopped_time > 10):
                        self.vel_msg.linear.x = 0
                        self.vel_msg.angular.z = turn_dir
                        rospy.loginfo_throttle(1, "Stuck more than 10s")
                    elif (self.stopped_time > 5):
                        self.vel_msg.linear.x = -1
                        self.vel_msg.angular.z = 0
                        rospy.loginfo_throttle(1, "Stuck more than 5s")
                    else:
                        self.vel_msg.linear.x = 0
                    self.vel_pub.publish(self.vel_msg)
        
    def pose_callback(self, pose_msg):
        """Updates pose estimate from the pose msgs coming from /odom topic
        Args:
            pose_msg (Odometry): Contains info regarding angular/linear positions and velocity
        """
        rospy.loginfo_once("Got pose message")
        self.pose_estimate["x"] = pose_msg.pose.pose.position.x
        self.pose_estimate["y"] = pose_msg.pose.pose.position.y
        self.pose_estimate["z"] = pose_msg.pose.pose.position.z
        self.pose_estimate["v"] = pose_msg.twist.twist.linear.x
        if (self.pose_estimate["v"] < 0.02):
            self.stopped_time = time.time() - self.time_since_stopped
        else:
            self.time_since_stopped = time.time()
            self.stopped_time = 0
        self.pose_estimate["w"] = pose_msg.twist.twist.angular.z
        orientation = pose_msg.pose.pose.orientation
        angle = math.degrees(euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2])
        if angle < 0:
            # Need to bring angle into 0-360 degree scope here from -180-180
            self.pose_estimate["t"] = 360.0 + angle
        else:
            self.pose_estimate["t"] = angle
        
    def scan_callback(self, scan_msg):
        rospy.loginfo_once("Got scan message")
        self.left_turn = max(scan_msg.ranges[0:309]) < max(scan_msg.ranges[409:719])
        rospy.loginfo_throttle(5, self.left_turn)
        if self.stopped_time < 5:
            center_scan = scan_msg.ranges[309:409]
            temp_msg = scan_msg
            temp_msg.angle_min = scan_msg.angle_min + (scan_msg.angle_increment * 309)
            temp_msg.ranges = center_scan
            self.scan_pub.publish(temp_msg)
            straight_min = min(center_scan)
            if straight_min >= safe_distance:
                self.found_safe_point = True
                self.safe_point[0] = self.pose_estimate["x"] + ((0.5 * straight_min) * np.cos(math.radians(self.pose_estimate["t"])))
                self.safe_point[1] = self.pose_estimate["y"] + ((0.5 * straight_min) * np.sin(math.radians(self.pose_estimate["t"])))
                self.safe_point[2] = self.pose_estimate["z"]
                self.safe_point[3] = self.pose_estimate["t"]
                self.marker.pose.position.x = self.safe_point[0]
                self.marker.pose.position.y = self.safe_point[1]
                self.marker.pose.position.z = self.safe_point[2]
                self.vis_pub.publish(self.marker)
            else:
                self.found_safe_point = False  
        else:
            self.found_safe_point = False          
            
    def send_goal(self, goal_pose):
        t = math.radians(goal_pose[3])
        q = quaternion_from_euler(0, 0, t)
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = goal_pose[0]
        self.goal.target_pose.pose.position.y = goal_pose[1]
        self.goal.target_pose.pose.position.z = goal_pose[2]
        self.goal.target_pose.pose.orientation.x = q[0]
        self.goal.target_pose.pose.orientation.y = q[1]
        self.goal.target_pose.pose.orientation.z = q[2]
        self.goal.target_pose.pose.orientation.w = q[3]
        self.goal_client.send_goal(self.goal)
        while (abs(self.pose_estimate["v"])) > 0.02 and (not rospy.is_shutdown()):
            rospy.loginfo_throttle(5, "Driving towards goal")
        rospy.loginfo_throttle(5, "Stopped, reached goal or got stuck")

'''
TO DO'S
- Develop safety
- Take a video
'''