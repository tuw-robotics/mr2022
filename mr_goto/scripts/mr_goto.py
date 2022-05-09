from turtle import distance
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from math import sqrt, atan2, sin, cos

pub_cmd = None
odom_ = None # type: Pose2D
goal_ = None # type: Pose2D
cmd_vel = Twist()
action_state = 0 # 0: idle, 1: init, 2: turn_to_goal, 3: straight, 4: final_orientation

def angle_difference(a, b):
    return atan2(sin(a-b), cos(a-b))

def planner_1():
    global action_state
    global odom_
    global goal_
    global cmd_vel
    v = 0.0
    w = 0.0

    if action_state == 0:
        return
    
    goal_direction = atan2(goal_.y - odom_.y, goal_.x - odom_.x)
    distance_to_goal = sqrt( (goal_.x - odom_.x)**2 + (goal_.y - odom_.y)**2 )
    current_orientation = odom_.theta
    goal_orientation = goal_.theta

    if action_state == 1:
        rospy.loginfo("Goal received, start planning, distance to goal: " + str(distance_to_goal))
        action_state = 2
    
    if action_state == 2:
        if (angle_difference(goal_direction, current_orientation) > 0.2):
            w = 0.2
        elif (angle_difference(goal_direction, current_orientation) < -0.2):
            w = -0.2
        else:
            rospy.loginfo("Goal direction reached, start moving forward")
            action_state = 3
    
    if action_state == 3: # straight
        if (distance_to_goal > 0.2):
            v = 0.2
        else:
            rospy.loginfo("Goal reached, start final orientation")
            action_state = 4
    
    if action_state == 4: # final orientation
        if (angle_difference(goal_orientation, current_orientation) > 0.2):
            w = 0.2
        elif (angle_difference(goal_orientation, current_orientation) < -0.2):
            w = -0.2
        else:
            rospy.loginfo("Goal orientation reached, stop")
            action_state = 0
    
    cmd_vel.linear.x = v
    cmd_vel.angular.z = w

def callbackOdometry(data):
    global odom_
    # rospy.loginfo("Odometry received: " + str(data))

    # transform orientation from quaternion to euler
    euler = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])

    odom_ = Pose2D(data.pose.pose.position.x, data.pose.pose.position.y, euler[2])

def callbackGoal(data: Pose2D):
    global goal_

    goal_ = Pose2D(data.position.x, data.position.y, data.theta)

    rospy.loginfo(f"Goal received:  ({goal_.x:.2f}, {goal_.y:.2f}, {goal_.theta:.2f})")

def callbackMoveBaseSimpleGoal(data):
    global action_state
    global goal_

    euler = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])

    goal_ = Pose2D(data.pose.position.x, data.pose.position.y, euler[2])
    rospy.loginfo(f"MBS-Goal received: ({goal_.x:.2f}, {goal_.y:.2f}, {goal_.theta:.2f})")
    action_state = 1

def listener():
    global cmd_vel

    rospy.init_node('mr_goto', anonymous=True)

    rate = rospy.Rate(10)  # 10hz

    pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rospy.Subscriber("odom", Odometry, callbackOdometry)
    rospy.Subscriber("goal", Pose2D, callbackGoal)
    rospy.Subscriber("move_base_simple/goal", PoseStamped, callbackMoveBaseSimpleGoal)

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        planner_1()
        pub_cmd.publish(cmd_vel)
        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()



if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass


