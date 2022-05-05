#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Float64

class GlobalPlanner(object):
    def __init__(self):
        rospy.loginfo("GlobalPlanner init")
        
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size = 1)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initialpose_callback, queue_size = 1)
        self.path_publisher = rospy.Publisher('/waypoints', Path, queue_size=1)
        
        self.initpose = PoseWithCovarianceStamped()
        self.initpose.header.stamp = rospy.get_rostime()
        self.initpose.header.frame_id = "unknown"
        self.initpose.pose.pose.position.x = 0
        self.initpose.pose.pose.position.y = 0

        self.N_STEPS = 5

    def goal_callback(self, goal_msg):
        if(self.initpose.header.frame_id == "unknown"):
            rospy.loginfo("inital pose was not set before")
        else:
            rospy.loginfo("goal_callback received")
            msg = Path()
            msg.header.frame_id = "map"
            msg.header.stamp = rospy.get_rostime()
            for i in range(self.N_STEPS):
                pose = PoseStamped()
                pose.pose.position.x = self.initpose.pose.pose.position.x + (goal_msg.pose.position.x - self.initpose.pose.pose.position.x)/(self.N_STEPS-1)*i
                pose.pose.position.y = self.initpose.pose.pose.position.y + (goal_msg.pose.position.y - self.initpose.pose.pose.position.y)/(self.N_STEPS-1)*i
                pose.pose.position.z = 0
                if(i == 0):
                    pose.pose.orientation = self.initpose.pose.pose.orientation
                if(i >= self.N_STEPS-1):
                    pose.pose.orientation = goal_msg.pose.orientation
                #quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
                #pose.pose.orientation.x = quaternion[0]
                #pose.pose.orientation.y = quaternion[1]
                #pose.pose.orientation.z = quaternion[2]
                #pose.pose.orientation.w = quaternion[3]
                msg.poses.append(pose)
            
            self.path_publisher.publish(msg)

    def initialpose_callback(self, pose_msg):
        rospy.loginfo("initialpose_callback received")
        self.initpose = pose_msg
        #rospy.loginfo(self.initpose)

def main():
    rospy.init_node('global_planner_node')
    gp = GlobalPlanner()
    rospy.spin()
if __name__ == '__main__':
    main()

