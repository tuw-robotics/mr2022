#include "self_localization_node.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <tf/transform_datatypes.h>
#include <boost/filesystem.hpp>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace moro;

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "self_localization" );
    ros::NodeHandle n;
    SelfLocalizationNode self_localization ( n );
    self_localization.init();
    ros::Rate rate ( 10 );

    while ( ros::ok() ) {

        /// localization
        self_localization.localization();

        /// publishes the estimated pose
        self_localization.publishPoseEstimated();

        /// publishes odom to map tf
        self_localization.publishOdomToMapTf();

        /// publishes fitler particles
        self_localization.publishFilterParticles();

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
    if ( mode == PoseFilter::PARTICLE_FILTER ) pose_filter_ = std::make_shared<moro::ParticleFilter>();
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

    /// defines a publisher for the filter particles
    pub_filter_particles_ = n.advertise<geometry_msgs::PoseArray> ( "filter_particles", 1 );

    /// defines a timer_ for publishing map_ once every second
    timer_ = n.createTimer(ros::Duration(1), &SelfLocalizationNode::callbackTimer, this );

    /// defines a publisher for map_
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
        // transform.setRotation ( tf::Quaternion ( 0,0,0,1 ) );    /// Dummy to remove
        // transform.setOrigin ( tf::Vector3 ( 1,0,0 ) );           /// Dummy to remove
        tf_listener_->lookupTransform("base_link", "base_laser_link", ros::Time(0), transform);
        // soft code -> _laser.header.frame_id

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
    ROS_INFO_STREAM("callbackInitialpose:" << pose.pose.pose.position.x << ", " << pose.pose.pose.position.y << ", " << a);
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
}

/**
 * Publishes odom to map tf
 **/
void SelfLocalizationNode::publishOdomToMapTf () {

    tf::Stamped<tf::Pose> odom_to_map;

    std::string base_frame_id = "base_footprint";
    std::string odom_frame_id = "odom";
    std::string global_frame_id = "map";

    try {
        tf::Transform tmp_tf(tf::createQuaternionFromYaw(pose_estimated_.get_theta()), tf::Vector3(pose_estimated_.get_x(), pose_estimated_.get_y(), 0));
        tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(), ros::Time::now(), base_frame_id);
        tf_listener_->transformPose(odom_frame_id, tmp_tf_stamped, odom_to_map);
    } catch ( tf::TransformException ex ) {
        ROS_ERROR ( "%s",ex.what() );
        ros::Duration ( 1.0 ).sleep();
    }
       
    tf::Transform odom_to_map_tf(tf::Quaternion(odom_to_map.getRotation()), tf::Point(odom_to_map.getOrigin()));

    tf::StampedTransform tmp_tf_stamped(odom_to_map_tf.inverse(), ros::Time::now(), global_frame_id, odom_frame_id);

    br_.sendTransform(tmp_tf_stamped);
}

/**
 * Publishes filter particles
 **/
void SelfLocalizationNode::publishFilterParticles() {
    std::vector< SamplePtr > samples_temp = pose_filter_->getSamples();
    geometry_msgs::PoseArray pose_array_;
    pose_array_.header.frame_id = "map";
    pose_array_.header.stamp = ros::Time::now();
    pose_array_.header.seq++;
    geometry_msgs::Pose pose;
    for(auto& s : samples_temp) {
        pose.position.x = s->get_x();
        pose.position.y = s->get_y();
        pose.position.z = 0;
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(s->get_theta()), pose.orientation);
        pose_array_.poses.push_back(pose);
    }
    pub_filter_particles_.publish(pose_array_);
}

void SelfLocalizationNode::callbackTimer(const ros::TimerEvent& event) {

    map_.header.frame_id = "map";
    map_.header.stamp = ros::Time::now();
    map_.info.resolution = 1. / figure_map_.scale_x(); // m/cell
    map_.info.width = figure_map_.width(); // cells
    map_.info.height = figure_map_.height(); // cells
    map_.info.origin.position.x = -(figure_map_.width() / 2.) / figure_map_.scale_x(); // origin in x of the map [m]
    map_.info.origin.position.y = (figure_map_.height() / 2.) / figure_map_.scale_y(); // origin in y of the map [m]
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(-M_PI / 2.), map_.info.origin.orientation);
    map_.data.resize( 0 );
    
    cv::Mat data = cv::imread(figure_map_.backgroundFileName(), cv::IMREAD_GRAYSCALE);
    // cv::imshow("background", data);

    for(size_t i = 0; i < figure_map_.height(); ++i) {
        for(size_t j = 0; j < figure_map_.height(); ++j) {
            map_.data.push_back( ( 255 - data.at<u_int8_t>( j , i ) ) / 255 * 100 );
        }
    }

    pub_map_.publish(map_);
}