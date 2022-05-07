#include "self_localization_node.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <boost/filesystem.hpp>

using namespace moro;

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "self_localization" );
    ros::NodeHandle n;
    SelfLocalizationNode self_localization ( n );
    self_localization.init();
    ros::Rate rate ( 10 );

    int cnt = 1;

    while ( ros::ok() ) {

        /// localization
        self_localization.localization();

        /// publishes the estimated pose
        self_localization.publishPoseEstimated();

        /// publishes the estimated pose
        self_localization.broadcastEstimatedPoseAsTF();

        /// publishes the particles
        self_localization.publishParticles();

        /// publishes the map every second
        if (cnt == 10) {
            self_localization.publishMap();
            cnt = 1;
        } else {
            cnt++;
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
    
    /// defines a publisher for the resulting particles
    pub_particles_ = n.advertise<geometry_msgs::PoseArray> ( "particles", 1 );

    /// defines a publisher for the map
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
         tf_listener_->lookupTransform("/base_link", _laser.header.frame_id, ros::Time(0), transform);
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
    ROS_INFO("Got initial pose");
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
    

    // map -> odom TF:
    tf::Stamped<tf::Pose> odom_to_map;
    try {
        tf::Transform tmp_tf(tf::createQuaternionFromYaw(pose_estimated_.theta()),
                            tf::Vector3(pose_estimated_.x(),
                                pose_estimated_.y(),
                                0.0));
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                                pose_.header.stamp,
                                                "base_link");
        this->tf_listener_->transformPose("odom",
                                            tmp_tf_stamped,
                                            odom_to_map);
    }
    catch(tf::TransformException) {
        ROS_DEBUG("Failed to subtract base to odom transform");
        return;
    }

    tf::Transform latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
    tf::Point(odom_to_map.getOrigin()));

    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                ros::Time::now(),
                                "map", "odom");
    static tf::TransformBroadcaster tfb;
    tfb.sendTransform(tmp_tf_stamped);
}


/**
 * Publishes the estimated pose
 **/
void SelfLocalizationNode::broadcastEstimatedPoseAsTF() {
    if ( pose_filter_->time_last_update().is_not_a_date_time() ) return;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pose_estimated_.x(), pose_estimated_.y(), 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, pose_estimated_.theta());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::fromBoost(pose_filter_->time_last_update()), "map", "pose_estimated_tf"));
}

/**
 * Publishes the particles
 **/
void SelfLocalizationNode::publishParticles () {
    if ( pose_filter_->time_last_update().is_not_a_date_time() ) return;
    particles_.header.stamp.fromBoost ( pose_filter_->time_last_update() );
    particles_.header.seq++;
    particles_.header.frame_id = "map";
    particles_.poses.clear();
    particles_.poses.resize(pose_filter_->samples.size());
    for(size_t i = 0; i< particles_.poses.size(); i++) {
        geometry_msgs::Pose p_pose;
        particles_.poses[i].position.x = pose_filter_->samples[i]->x();
        particles_.poses[i].position.y = pose_filter_->samples[i]->y();
        particles_.poses[i].position.z = 0;
        particles_.poses[i].orientation = tf::createQuaternionMsgFromYaw ( pose_filter_->samples[i]->theta() );
    }
    /// publishes motion command
    pub_particles_.publish ( particles_ );
}

/**
 * Publishes the particles
 **/
void SelfLocalizationNode::publishMap () {
    if ( pose_filter_->time_last_update().is_not_a_date_time() ) return;
    auto map_msg_to_publish = pose_filter_->get_map_msg_to_publish();
    map_msg_to_publish.header.stamp.fromBoost ( pose_filter_->time_last_update() );
    map_msg_to_publish.header.seq++;
    map_msg_to_publish.info.map_load_time = ros::Time::now();

    pub_map_.publish ( map_msg_to_publish );
}
