#!/usr/bin/env python3
from __future__ import print_function
from calendar import c
from re import T
import sys
import math

#ROS Imports
import rospy
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

def norm_angle_difference(alpha0, angle1):
    a0 = angle_normalize(alpha0)
    a1 = angle_normalize(angle1)
    return math.atan2(math.sin(a0-a1), math.cos(a0-a1))

def norm_orientation(z, w):
    return angle_normalize(2 * math.atan2(z, w))

class GoTo:
    def __init__(self):
        rospy.loginfo("Hello from goto node")

        #Topics & Subs, Pubs
        drive_topic = '/cmd_vel'
        goal_topic = '/move_base_simple/goal'
        odom_topic = '/odom'

        self.goal_pose = None
        self.movement_pattern = 0
        
        self.goal_sub = rospy.Subscriber(goal_topic, PoseStamped, self.goal_callback)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.drive_pub = rospy.Publisher(drive_topic, Twist, queue_size=10)

        self.listener = tf.TransformListener()


    def odom_callback(self, data):
        self.move(data)

   
    def goal_callback(self, data):
        rospy.logdebug("goal callback data: %f", data.pose.position.x)
        
        self.goal_pose = data
        print("goal callback:", data)
        self.movement_pattern = 1
    
    
    def init_pose_callback(self, data):
        rospy.logdebug("init pose callback data: %f, %f", data.pose.pose.position.x, data.pose.pose.position.y)

        
    def move(self, data):

        if not self.goal_pose:
            return

        try:
        # if listener.frameExists('odom'):
            (trans, rota) = self.listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('tf exception thrown')
            return

        velocity = 0.0
        rotation = 0.0

        goal_direction = angle_normalize(math.atan2(self.goal_pose.pose.position.y - trans[1], self.goal_pose.pose.position.x - trans[0]))
        # print("Goal direction ", goal_direction)
        current_orientation = norm_orientation(rota[2], rota[3])
        # print("Current orientation ", current_orientation)
        angle_diff = angle_difference(goal_direction, current_orientation)
    
        position_diff = math.sqrt(math.pow(self.goal_pose.pose.position.x - trans[0], 2) + math.pow(self.goal_pose.pose.position.y - trans[1], 2))

        goal_angle = norm_orientation(self.goal_pose.pose.orientation.z, self.goal_pose.pose.orientation.w)

        goal_angle_diff = angle_difference(goal_angle, current_orientation)

        if self.movement_pattern == 1:
            if angle_diff > 0.2:
                rotation = 0.1
            elif angle_diff < -0.2:
                rotation = -0.1
            else:
                print("Rotation matched")
                self.movement_pattern = 2
        if self.movement_pattern == 2:
            if angle_diff > 0.2 or angle_diff < -0.2:
                self.movement_pattern = 1
            if position_diff > 0.2:
                velocity = 0.2
            else:
                print("Position matched")
                self.movement_pattern = 3
        if self.movement_pattern == 3:
            if goal_angle_diff > 0.1:
                rotation = 0.05
            elif goal_angle_diff < -0.1:
                rotation = -0.05
            else:
                print("Target position reached")
                self.movement_pattern = 0

        cmd = Twist()
        cmd.linear.x = velocity
        cmd.angular.z = rotation
        self.drive_pub.publish(cmd)
        
 
def main(args):
    rospy.init_node("mr_goto_node", anonymous=True)
    gt = GoTo()

    rospy.sleep(0.1)
    rospy.spin()


if __name__=='__main__':
	main(sys.argv)
