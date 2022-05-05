#include "self_localization_node.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <tf/transform_datatypes.h>
#include <boost/filesystem.hpp>
#include <nav_msgs/OccupancyGrid.h>

using namespace moro;

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "self_localization" );
    ros::NodeHandle n;
    SelfLocalizationNode self_localization ( n );
    self_localization.init();
    ros::Rate rate ( 10 );

    int pub_map_cnt = 0;

    while ( ros::ok() ) {

        /// localization
        self_localization.localization();

        /// publishes the estimated pose
        self_localization.publishPoseEstimated();

        if ( pub_map_cnt == 10 ){
            self_localization.publishMap();
            pub_map_cnt = 0;
        }

        /// plots measurements
        self_localization.plot();

        /// calls all callbacks waiting in the queue
        ros::spinOnce();

        /// sleep for the time remaining to let us hit our publish rate
        rate.sleep();
    }
    return 0;
}

/**
 * Constructor
 **/
SelfLocalizationNode::SelfLocalizationNode ( ros::NodeHandle & n )
    : SelfLocalization ( ros::NodeHandle ( "~" ).getNamespace() ),
      n_ ( n ),
      n_param_ ( "~" ) {

    // reads shared parameter on the operation mode
    int mode;
    if ( ! n_param_.getParam ( "mode", mode ) ) {
        ROS_ERROR ( "mode is not set" );
        return;
    }
    if ( mode == PoseFilter::PARTICLE_FILTER ) pose_filter_ = std::make_shared<moro::ParticleFilter>( n );  // to publish particles, we pass the node handle to the constructor
    if ( mode == PoseFilter::KALMAN_FILTER )   pose_filter_ = std::make_shared<moro::KalmanFilter>();

    ROS_INFO ( "mode: %s(%i)", pose_filter_->getTypeName().c_str(), ( int ) pose_filter_->getType() );
    n_param_.getParam ( "map_image", filename_map_image_ );

    if ( boost::filesystem::exists ( filename_map_image_ ) )  {
        ROS_INFO ( "map_image: %s", filename_map_image_.c_str() );
    } else {
        ROS_ERROR ( "map_image file does not exist: %s", filename_map_image_.c_str() );
        return;
    }
    n_param_.getParam ( "map_lines", filename_map_lines_ );

    if ( boost::filesystem::exists ( filename_map_lines_ ) )  {
        ROS_INFO ( "map_lines: %s", filename_map_lines_.c_str() );
    } else {
        ROS_ERROR ( "map_lines file does not exist: %s", filename_map_lines_.c_str() );
        return;
    }
    /// subscribes to transformations
    tf_listener_ = std::make_shared<tf::TransformListener>();

    n_param_.param<std::string> ( "frame_id_map", pose_.header.frame_id, "map" );

    /// subscribes to  odometry values
    sub_cmd_ = n.subscribe ( "cmd", 1, &SelfLocalizationNode::callbackCmd, this );

    /// subscribes to  odometry values
    sub_odometry_ = n.subscribe ( "odom", 1, &SelfLocalizationNode::callbackOdometry, this );

    /// two subscribers to laser sensor
    sub_initial_pose_ = n.subscribe ( "initialpose", 1, &SelfLocalizationNode::callbackInitialpose, this );

    /// two subscribers to ground truth data
    sub_ground_truth_ = n.subscribe ( "base_pose_ground_truth", 1, &SelfLocalizationNode::callbackGroundTruth, this );

    /// defines a publisher for the resulting pose
    pub_pose_estimated_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped> ( "pose_estimated", 1 );   

    pub_map_ = n.advertise<nav_msgs::OccupancyGrid> ( "map", 1 ); 

    pose_.header.seq = 0;

    /// start parameter server
    reconfigureFncSelfLocalization_ = boost::bind ( &SelfLocalizationNode::callbackConfigSelfLocalization, this,  _1, _2 );
    reconfigureServerSelfLocalization_.setCallback ( reconfigureFncSelfLocalization_ );

    /// two subscribers to laser sensor
    sub_laser_ = n.subscribe ( "scan", 10, &SelfLocalizationNode::callbackLaser, this );

    if ( pose_filter_->getType() == PoseFilter::PARTICLE_FILTER ) {

        /// start parameter server
        reconfigureServerParticleFilter_ = std::make_shared< dynamic_reconfigure::Server<mr_self_localization::ParticleFilterConfig> > ( ros::NodeHandle ( "~/particle_filter" ) );
        reconfigureFncParticleFilter_ = boost::bind ( &SelfLocalizationNode::callbackConfigParticleFilter, this,  _1, _2 );
        reconfigureServerParticleFilter_->setCallback ( reconfigureFncParticleFilter_ );
    }
    if ( pose_filter_->getType() == PoseFilter::KALMAN_FILTER ) {

        /// start parameter server
        reconfigureServerKalmanFilter_ = std::make_shared< dynamic_reconfigure::Server<mr_self_localization::KalmanFilterConfig> > ( ros::NodeHandle ( "~/kalman_filter" ) );
        reconfigureFncKalmanFilter_ = boost::bind ( &SelfLocalizationNode::callbackConfigKalmanFilter, this,  _1, _2 );
        reconfigureServerKalmanFilter_->setCallback ( reconfigureFncKalmanFilter_ );
    }
}


void SelfLocalizationNode::callbackConfigSelfLocalization ( mr_self_localization::SelfLocalizationConfig &config, uint32_t level ) {
    ROS_INFO ( "callbackConfigSelfLocalization!" );
    config_ = config;
    init();
}

void SelfLocalizationNode::callbackConfigParticleFilter ( mr_self_localization::ParticleFilterConfig &config, uint32_t level ) {
    ROS_INFO ( "callbackConfigParticleFilter!" );
    pose_filter_->setConfig ( &config );
}

void SelfLocalizationNode::callbackConfigKalmanFilter ( mr_self_localization::KalmanFilterConfig &config, uint32_t level ) {
    ROS_INFO ( "callbackConfigKalmanFilter!" );
    pose_filter_->setConfig ( &config );
}

void SelfLocalizationNode::localization() {
    if ( config_.reinitialize ) {
        pose_filter_->setPoseInit ( pose_ground_truth_ );
        pose_filter_->reset();
    }
    /// localization
    SelfLocalization::localization ();
}

/**
 * copies incoming laser messages to the base class
 * @param laser
 **/
void SelfLocalizationNode::callbackLaser ( const sensor_msgs::LaserScan &_laser ) {
    tf::StampedTransform transform;
    /**
     * @ToDo Sensor Mount
     * remove the static transformation and use the tf_listener_
     * @url http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29
     **/
    try {
#if SELF_LOCALIZATION_EXERCISE >= 10
#else
        /**
         * @node your code
         **/        
        tf_listener_->lookupTransform("/base_link", _laser.header.frame_id, ros::Time(0.0), transform);
#endif
        double roll = 0, pitch = 0, yaw = 0;
        transform.getBasis().getRPY ( roll, pitch, yaw );
        measurement_laser_->pose2d() = Pose2D ( transform.getOrigin().x(),  transform.getOrigin().y(), yaw );
    } catch ( tf::TransformException ex ) {
        ROS_ERROR ( "%s",ex.what() );
        ros::Duration ( 1.0 ).sleep();
    }

    int nr = ( _laser.angle_max - _laser.angle_min ) / _laser.angle_increment;
    measurement_laser_->range_max() = _laser.range_max;
    measurement_laser_->range_min() = _laser.range_min;
    measurement_laser_->resize ( nr );
    measurement_laser_->stamp() = _laser.header.stamp.toBoost();
    for ( int i = 0; i < nr; i++ ) {
        MeasurementLaser::Beam &beam = measurement_laser_->operator[] ( i );
        beam.length = _laser.ranges[i];
        beam.angle = _laser.angle_min + ( _laser.angle_increment * i );
        beam.end_point.x() = cos ( beam.angle ) * beam.length;
        beam.end_point.y() = sin ( beam.angle ) * beam.length;
    }
}

/**
 * copies incoming odometry messages to the base class
 * @param odom
 **/
void SelfLocalizationNode::callbackOdometry ( const nav_msgs::Odometry &odom ) {
    tf::Quaternion q;
    tf::quaternionMsgToTF ( odom.pose.pose.orientation, q );
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3 ( q ).getRPY ( roll, pitch, yaw );
    double a = yaw;
    odom_.set ( odom.pose.pose.position.x, odom.pose.pose.position.y, a );
}

/**
 * copies incoming robot pose messages to the base class
 * @param pose
 **/
void SelfLocalizationNode::callbackInitialpose ( const geometry_msgs::PoseWithCovarianceStamped &pose ) {
    tf::Quaternion q;
    tf::quaternionMsgToTF ( pose.pose.pose.orientation, q );
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3 ( q ).getRPY ( roll, pitch, yaw );
    double a = yaw;
    pose_filter_->reinitialize ( Pose2D ( pose.pose.pose.position.x, pose.pose.pose.position.y, a ) );
}


/**
 * copies incoming robot command message
 * @param cmd
 **/
void SelfLocalizationNode::callbackCmd ( const geometry_msgs::Twist& cmd ) {
    cmd_.v() = cmd.linear.x;
    cmd_.w() = cmd.angular.z;
}

/**
 * copies incoming odometry messages to the base class
 * @param odom
 **/
void SelfLocalizationNode::callbackGroundTruth ( const nav_msgs::Odometry& ground_truth ) {
    tf::Quaternion q;
    tf::quaternionMsgToTF ( ground_truth.pose.pose.orientation, q );
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3 ( q ).getRPY ( roll, pitch, yaw );
    double a = yaw;
    pose_ground_truth_.set ( ground_truth.pose.pose.position.x, ground_truth.pose.pose.position.y, a );

    if ( config_.initial_with_ground_truth ) {
        ROS_INFO ( "initial_with_ground_truth!" );
        pose_filter_->reinitialize ( pose_ground_truth_ );
        config_.initial_with_ground_truth = false;
    }
}

/**
 * Publishes the estimated pose
 **/
void SelfLocalizationNode::publishPoseEstimated () {
    if ( pose_filter_->time_last_update().is_not_a_date_time() ) return;    
    pose_.header.stamp.fromBoost ( pose_filter_->time_last_update() );
    pose_.header.seq++;
    pose_.pose.pose.position.x = pose_estimated_.x();
    pose_.pose.pose.position.y = pose_estimated_.y();
    pose_.pose.pose.position.z = 0;
    pose_.pose.pose.orientation = tf::createQuaternionMsgFromYaw ( pose_estimated_.theta() );
    for ( double &d : pose_.pose.covariance ) d = 0;
    /// publishes motion command
    pub_pose_estimated_.publish ( pose_ );
    
    // subtracting base2odom from map2base
    try
    {
        tf::Transform map2base;
        map2base.setOrigin( tf::Vector3(pose_estimated_.x(), pose_estimated_.y(), 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, pose_estimated_.theta());
        map2base.setRotation(q);
    
        tf::Stamped<tf::Pose> stamped_base2map( map2base.inverse(),
                                                ros::Time::now(),
                                                "base_link" );
        tf::Stamped<tf::Pose> tf_odom2map;
        tf_listener_->transformPose("odom", stamped_base2map, tf_odom2map);
        // send map to odom instead
        static tf::TransformBroadcaster br;
        tf::StampedTransform stamped_map2odom(tf_odom2map.inverse(),
                                                ros::Time::now(),
                                                pose_.header.frame_id, 
                                                "odom");
        br.sendTransform(stamped_map2odom);
    }
    catch(tf::TransformException)
    {
        ROS_DEBUG("Failed to subtract base to odom transform");
        return;
    }
}

/**
 * Publishes the map
 **/
void SelfLocalizationNode::publishMap(){
    
    nav_msgs::OccupancyGrid map_msg;

    // Header
    auto stamp = ros::Time::now();
    static size_t seq = 0;
    map_msg.header.stamp = stamp;
    map_msg.header.frame_id = "map";
    map_msg.header.seq = seq++;

    // Metadata
    map_msg.info.map_load_time = stamp;
    map_msg.info.width = figure_map_.width();
    map_msg.info.height = figure_map_.height();
    // scale_dim = pixel_dim / dim -> resolution = 1 / scale
    map_msg.info.resolution = 1 / figure_map_.scale_x();

    // width/height are for pixels -> divide by scale
    // y origin not negative because of rotation around x axis
    map_msg.info.origin.position.x = -( figure_map_.width() / 2 ) / figure_map_.scale_x();
    map_msg.info.origin.position.y = ( figure_map_.height() / 2 ) / figure_map_.scale_y();
    map_msg.info.origin.orientation.x = 1;

    // Map data
    // https://stackoverflow.com/questions/56233780/convert-an-image-into-an-occupancy-grid
    cv::Mat map_gray = cv::imread( filename_map_image_, cv::IMREAD_GRAYSCALE );
    cv::Mat map_bin;
    cv::threshold( map_gray, map_bin, 100, 255.0, cv::THRESH_BINARY );

    // Empty space is represented by max value, so "invert" values and use 100 as max 
    map_bin = 100 - ( ( map_bin / 255 ) * 100 );

    // https://stackoverflow.com/questions/26681713/convert-mat-to-array-vector-in-opencv
    // https://stackoverflow.com/questions/33665241/is-opencv-matrix-data-guaranteed-to-be-continuous
    cv::Mat map_flat = map_bin.reshape( 1, map_bin.total() );
    std::vector<int8_t> map_vec = map_flat.isContinuous() ? map_flat : map_flat.clone();
    map_msg.data = map_vec;
    
    pub_map_.publish( map_msg );
}
