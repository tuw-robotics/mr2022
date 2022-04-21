#include "monitor_node.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/bind.hpp>
#include <numeric>

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "multirobot_monitor" );  /// initializes the ros node with default name
    ros::NodeHandle n;
    MonitorNode node ( n );
    return 0;
}

/**
 * Constructor
 **/
MonitorNode::MonitorNode ( ros::NodeHandle & n )
    : n_ ( n ),
      n_param_ ( "~" ) {

    init();
    sub_ground_truth0_ = n.subscribe ( "robot_0/base_pose_ground_truth", 1, &MonitorNode::callback_ground_truth0, this );
    sub_ground_truth1_ = n.subscribe ( "robot_1/base_pose_ground_truth", 1, &MonitorNode::callback_ground_truth1, this );
    sub_ground_truth2_ = n.subscribe ( "robot_2/base_pose_ground_truth", 1, &MonitorNode::callback_ground_truth2, this );
    sub_ground_truth3_ = n.subscribe ( "robot_3/base_pose_ground_truth", 1, &MonitorNode::callback_ground_truth3, this );
    sub_ground_truth4_ = n.subscribe ( "robot_4/base_pose_ground_truth", 1, &MonitorNode::callback_ground_truth4, this );
    sub_ground_truth5_ = n.subscribe ( "robot_5/base_pose_ground_truth", 1, &MonitorNode::callback_ground_truth5, this );
    sub_ground_truth6_ = n.subscribe ( "robot_6/base_pose_ground_truth", 1, &MonitorNode::callback_ground_truth6, this );
    //sub_ground_truth_ = n.subscribe ( "robot_0/base_pose_ground_truth", 1, boost::bind(&MonitorNode::callback_ground_truth0, this, _1));

    cv::namedWindow ( "Monitor", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_NORMAL );
    figure_.create ( 200, 400, CV_8UC3 );

    ros::Rate rate ( 10 );  /// ros loop frequency synchronized with the wall time (simulated time)

    while ( ros::ok() ) {

        figure_.setTo ( 0xFF );
	plot();

        cv::imshow ( "Monitor", figure_ );
        cv::waitKey ( 1 );

        /// calls all callbacks waiting in the queue
        ros::spinOnce();
        /// sleeps for the time remaining to let us hit our publish rate
        rate.sleep();

    }
}

void MonitorNode::init() {
    visited.resize ( nr_of_robots );
    for ( size_t r = 0; r < nr_of_robots; r++ ) {
        visited[r].resize ( enviroment_size*enviroment_size, 0 );
    }
    colors.resize(nr_of_robots);
    colors[0] = cv::Scalar ( 0,   0, 255 );
    colors[1] = cv::Scalar ( 47,   255, 173 );
    colors[2] = cv::Scalar ( 47,   107, 85 );
    colors[3] = cv::Scalar ( 255,   0, 255 );
    colors[4] = cv::Scalar ( 255,   255, 0 );
    colors[5] = cv::Scalar ( 0,   255, 255 );
    colors[6] = cv::Scalar ( 143,   188, 143 );
}
void MonitorNode::update ( int robot, double x, double y ) {
    //ROS_INFO ( "updat %i", robot );
    int c = round(x) + enviroment_size/2;
    int r = round(y) + enviroment_size/2;
    int idx = r * enviroment_size + c;
    visited[robot][idx] = 1;
}

void MonitorNode::callback_ground_truth0 ( const nav_msgs::OdometryPtr& odom ) {
    update ( 0, odom->pose.pose.position.x, odom->pose.pose.position.y );

}
void MonitorNode::callback_ground_truth1 ( const nav_msgs::OdometryPtr& odom ) {
    update ( 1, odom->pose.pose.position.x, odom->pose.pose.position.y );

}
void MonitorNode::callback_ground_truth2 ( const nav_msgs::OdometryPtr& odom ) {
    update ( 2, odom->pose.pose.position.x, odom->pose.pose.position.y );

}
void MonitorNode::callback_ground_truth3 ( const nav_msgs::OdometryPtr& odom ) {
    update ( 3, odom->pose.pose.position.x, odom->pose.pose.position.y );

}
void MonitorNode::callback_ground_truth4 ( const nav_msgs::OdometryPtr& odom ) {
    update ( 4, odom->pose.pose.position.x, odom->pose.pose.position.y );

}
void MonitorNode::callback_ground_truth5 ( const nav_msgs::OdometryPtr& odom ) {
    update ( 5, odom->pose.pose.position.x, odom->pose.pose.position.y );

}
void MonitorNode::callback_ground_truth6 ( const nav_msgs::OdometryPtr& odom ) {
    update ( 6, odom->pose.pose.position.x, odom->pose.pose.position.y );

}
void MonitorNode::plot() {
  
  int space = 20;
  int bar_height = 10;
  char text[0xff];
  for(size_t robot = 0; robot < nr_of_robots; robot++){
    int sum_of_elems = std::accumulate(visited[robot].begin(), visited[robot].end(), 0);
    cv::Rect bar(70,space*(robot+1), sum_of_elems*2, bar_height);
    sprintf(text, "r%i %i", (int) robot, sum_of_elems);
    cv::putText(figure_, text, cv::Point(5,space*(robot+1)+bar_height), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0), 1, cv::LINE_AA);
    cv::rectangle(figure_, bar, colors[robot], cv::FILLED);
  }
}
