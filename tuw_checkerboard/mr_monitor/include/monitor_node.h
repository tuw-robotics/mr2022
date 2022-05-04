#ifndef MONITOR_NODE_H
#define MONITOR_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/core/core.hpp>
/**
 * class to cover the ros communication
 **/
class MonitorNode  {
public:
    MonitorNode ( ros::NodeHandle & n ); /// Constructor
    void publishMotion ();      /// publishes the motion commands 
    static const int enviroment_size = 20;
    static const int nr_of_robots = 7;
private:
    ros::NodeHandle n_;         /// node handler to the root node
    ros::NodeHandle n_param_;   /// node handler to the current node
    ros::Subscriber sub_ground_truth0_; 
    ros::Subscriber sub_ground_truth1_; 
    ros::Subscriber sub_ground_truth2_;
    ros::Subscriber sub_ground_truth3_; 
    ros::Subscriber sub_ground_truth4_; 
    ros::Subscriber sub_ground_truth5_; 
    ros::Subscriber sub_ground_truth6_; 
    void callback_ground_truth0 ( const nav_msgs::OdometryPtr& );  
    void callback_ground_truth1 ( const nav_msgs::OdometryPtr& );  
    void callback_ground_truth2 ( const nav_msgs::OdometryPtr& );  
    void callback_ground_truth3 ( const nav_msgs::OdometryPtr& );  
    void callback_ground_truth4 ( const nav_msgs::OdometryPtr& );  
    void callback_ground_truth5 ( const nav_msgs::OdometryPtr& );  
    void callback_ground_truth6 ( const nav_msgs::OdometryPtr& );  
    std::vector<std::vector<int> > visited;
    std::vector<cv::Scalar> colors;
    void update(int robot, double x, double y);
    void init();
    void plot();
    cv::Mat figure_;
    
};

#endif // MONITOR_NODE_H
