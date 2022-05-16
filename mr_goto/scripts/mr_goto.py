#!/usr/bin/env python3
from __future__ import print_function
from calendar import c
from re import T
import sys
import math
from turtle import left, speed

#ROS Imports
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import tf


def angle_difference(alpha0, angle1):
    return math.atan2(math.sin(alpha0-angle1), math.cos(alpha0-angle1))

def angle_normalize(angle):
    while angle > math.pi:
        angle -= 2.*math.pi
    while angle < -math.pi:
        angle += 2.*math.pi
    return angle

class GoTo:
    def __init__(self):
        rospy.loginfo("Hello from wall_follow node")

        #Topics & Subs, Pubs
        drive_topic = '/cmd_vel'
        goal_topic = '/move_base_simple/goal'
        initial_pose_topic = '/initialpose'
        odom_topic = '/odom'
        pose_estimated_topic = '/pose_estimated'

        self.goal_pose = None
        self.movement_pattern = 0
        
        self.goal_sub = rospy.Subscriber(goal_topic, PoseStamped, self.goal_callback)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.init_pose_sub = rospy.Subscriber(initial_pose_topic, PoseWithCovarianceStamped, self.init_pose_callback)
        self.drive_pub = rospy.Publisher(drive_topic, Twist, queue_size=10)
        self.pose_estimated_sub = rospy.Subscriber(pose_estimated_topic, PoseWithCovarianceStamped, self.pose_est_callback)

        self.listener = tf.TransformListener()


    def odom_callback(self, data):
        self.move(data)
        pass

    def pose_est_callback(self, data):
        print("Pose Estimated")
        print(data.pose.pose.position)
        print(data.pose.pose.orientation)

   
    def goal_callback(self, data):
        rospy.logdebug("goal callback data: %f", data.pose.position.x)
        
        self.goal_pose = data
        print("goal callback:", data)
        self.movement_pattern = 1
    
    
    def init_pose_callback(self, data):
        rospy.logdebug("init pose callback data: %f, %f", data.pose.pose.position.x, data.pose.pose.position.y)

        
    def move(self, data):

        try:
        # if listener.frameExists('odom'):
            (trans, rota) = self.listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('tf exception thrown')
            return

        print("Odometry:")
        print(data.pose.pose.position)
        print(data.pose.pose.orientation)
        print("TF:")
        print(trans)
        print(rota)

            
        cmd = Twist()
        cmd.linear.x = 2.0
        cmd.angular.z = 2.0
        print(cmd)
        self.drive_pub.publish(cmd)

        #if goal_pose: 
            #rospy.loginfo("I like to move it")
            #rospy.loginfo("goal: %fx, %fy", goal_pose.pose.position.x, goal_pose.pose.position.y)
            #rospy.loginfo("goal-orientation: %fx, %fy, %fz, %fw", goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w)
        #if data:
            #rospy.loginfo("I have a position")
            #rospy.loginfo("pose: %fx, %fy", data.pose.pose.position.x, data.pose.pose.position.y)
            #rospy.loginfo("pose-orientation: %fx, %fy, %fz, %fw", data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        # if self.movement_pattern == 0:
        #     rospy.loginfo("Waiting for instructions")
        # elif self.movement_pattern == 1:

        #     rospy.loginfo("Movement initialized - turning to goal")
        # elif self.movement_pattern == 2:
        #     rospy.loginfo("Moving in goal direction")
        #     pass
        # elif self.movement_pattern == 3:
        #     rospy.loginfo("Goal reached - turning in goal direction")
        #     movement_pattern = 0
        
        
 
def main(args):
    rospy.init_node("mr_goto_node", anonymous=True)
    gt = GoTo()

    rospy.sleep(0.1)
    rospy.spin()


if __name__=='__main__':
	main(sys.argv)
