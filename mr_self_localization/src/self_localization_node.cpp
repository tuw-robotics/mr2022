#include "self_localization_node.h"
#include <tf/transform_datatypes.h>
#include <boost/filesystem.hpp>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace moro;

int main(int argc, char **argv) {

    ros::init(argc, argv, "self_localization");
    ros::NodeHandle n;
    SelfLocalizationNode self_localization(n);
    self_localization.init();

    size_t frequency = 10;
    ros::Rate rate(frequency);

    size_t iteration = 0;

    while (ros::ok()) {

        /// localization
        self_localization.localization();

        /// publishes the estimated pose
        self_localization.publishPoseEstimated();

        // Only plot with 1Hz.
        if ((iteration % frequency) == 0) {
            /// Publish map
            self_localization.publishMap();
        }

        /// plots measurements
        self_localization.plot();

        /// calls all callbacks waiting in the queue
        ros::spinOnce();

        /// sleep for the time remaining to let us hit our publish rate
        rate.sleep();

        iteration += 1;
    }
    return 0;
}

/**
 * Constructor
 **/
SelfLocalizationNode::SelfLocalizationNode(ros::NodeHandle &n)
        : SelfLocalization(ros::NodeHandle("~").getNamespace()),
          n_(n),
          n_param_("~") {

    // reads shared parameter on the operation mode
    int mode;
    if (!n_param_.getParam("mode", mode)) {
        ROS_ERROR ("mode is not set");
        return;
    }
    if (mode == PoseFilter::PARTICLE_FILTER) pose_filter_ = std::make_shared<moro::ParticleFilter>();
    if (mode == PoseFilter::KALMAN_FILTER) pose_filter_ = std::make_shared<moro::KalmanFilter>();

    ROS_INFO ("mode: %s(%i)", pose_filter_->getTypeName().c_str(), (int) pose_filter_->getType());
    n_param_.getParam("map_image", filename_map_image_);

    if (boost::filesystem::exists(filename_map_image_)) {
        ROS_INFO ("map_image: %s", filename_map_image_.c_str());
    } else {
        ROS_ERROR ("map_image file does not exist: %s", filename_map_image_.c_str());
        return;
    }
    n_param_.getParam("map_lines", filename_map_lines_);

    if (boost::filesystem::exists(filename_map_lines_)) {
        ROS_INFO ("map_lines: %s", filename_map_lines_.c_str());
    } else {
        ROS_ERROR ("map_lines file does not exist: %s", filename_map_lines_.c_str());
        return;
    }
    /// subscribes to transformations
    tf_listener_ = std::make_shared<tf::TransformListener>();
    tf_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();

    n_param_.param<std::string>("frame_id_map", pose_.header.frame_id, "map");

    /// subscribes to  odometry values
    sub_cmd_ = n.subscribe("cmd", 1, &SelfLocalizationNode::callbackCmd, this);

    /// subscribes to  odometry values
    sub_odometry_ = n.subscribe("odom", 1, &SelfLocalizationNode::callbackOdometry, this);

    /// two subscribers to laser sensor
    sub_initial_pose_ = n.subscribe("initialpose", 1, &SelfLocalizationNode::callbackInitialpose, this);

    /// two subscribers to ground truth data
    sub_ground_truth_ = n.subscribe("base_pose_ground_truth", 1, &SelfLocalizationNode::callbackGroundTruth, this);

    /// defines a publisher for the resulting pose
    pub_pose_estimated_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_estimated", 1);
    pub_pose_array_ = n.advertise<geometry_msgs::PoseArray>("pose_array", 1);

    /// defines a published for the map
    pub_map_ = n.advertise<nav_msgs::OccupancyGrid>("map", 1);


    pose_.header.seq = 0;

    /// start parameter server
    reconfigureFncSelfLocalization_ = boost::bind(&SelfLocalizationNode::callbackConfigSelfLocalization, this, _1, _2);
    reconfigureServerSelfLocalization_.setCallback(reconfigureFncSelfLocalization_);

    /// two subscribers to laser sensor
    sub_laser_ = n.subscribe("scan", 10, &SelfLocalizationNode::callbackLaser, this);

    if (pose_filter_->getType() == PoseFilter::PARTICLE_FILTER) {

        /// start parameter server
        reconfigureServerParticleFilter_ = std::make_shared<dynamic_reconfigure::Server<mr_self_localization::ParticleFilterConfig> >(
                ros::NodeHandle("~/particle_filter"));
        reconfigureFncParticleFilter_ = boost::bind(&SelfLocalizationNode::callbackConfigParticleFilter, this, _1, _2);
        reconfigureServerParticleFilter_->setCallback(reconfigureFncParticleFilter_);
    }
    if (pose_filter_->getType() == PoseFilter::KALMAN_FILTER) {

        /// start parameter server
        reconfigureServerKalmanFilter_ = std::make_shared<dynamic_reconfigure::Server<mr_self_localization::KalmanFilterConfig> >(
                ros::NodeHandle("~/kalman_filter"));
        reconfigureFncKalmanFilter_ = boost::bind(&SelfLocalizationNode::callbackConfigKalmanFilter, this, _1, _2);
        reconfigureServerKalmanFilter_->setCallback(reconfigureFncKalmanFilter_);
    }
}


void SelfLocalizationNode::callbackConfigSelfLocalization(mr_self_localization::SelfLocalizationConfig &config,
                                                          uint32_t level) {
    ROS_INFO ("callbackConfigSelfLocalization!");
    config_ = config;
    init();
}

void
SelfLocalizationNode::callbackConfigParticleFilter(mr_self_localization::ParticleFilterConfig &config, uint32_t level) {
    ROS_INFO ("callbackConfigParticleFilter!");
    pose_filter_->setConfig(&config);
}

void
SelfLocalizationNode::callbackConfigKalmanFilter(mr_self_localization::KalmanFilterConfig &config, uint32_t level) {
    ROS_INFO ("callbackConfigKalmanFilter!");
    pose_filter_->setConfig(&config);
}

void SelfLocalizationNode::localization() {
    if (config_.reinitialize) {
        pose_filter_->setPoseInit(pose_ground_truth_);
        pose_filter_->reset();
    }
    /// localization
    SelfLocalization::localization();
}

/**
 * copies incoming laser messages to the base class
 * @param laser
 **/
void SelfLocalizationNode::callbackLaser(const sensor_msgs::LaserScan &_laser) {
    tf::StampedTransform transform;

    try {
#if SELF_LOCALIZATION_EXERCISE >= 10
#else
        auto base_link = n_.getNamespace().length() == 1 ? "/base_link" : n_.getNamespace() + "/base_link";
        auto laser_link = n_.getNamespace().length() == 1 ? _laser.header.frame_id : n_.getNamespace() + "/" +
                                                                                     _laser.header.frame_id;

        tf_listener_->lookupTransform(base_link, laser_link, ros::Time::now(), transform);
#endif
        double roll = 0, pitch = 0, yaw = 0;
        transform.getBasis().getRPY(roll, pitch, yaw);
        measurement_laser_->pose2d() = Pose2D(transform.getOrigin().x(), transform.getOrigin().y(), yaw);
    } catch (tf::TransformException ex) {
        ROS_ERROR ("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    int nr = (_laser.angle_max - _laser.angle_min) / _laser.angle_increment;
    measurement_laser_->range_max() = _laser.range_max;
    measurement_laser_->range_min() = _laser.range_min;
    measurement_laser_->resize(nr);
    measurement_laser_->stamp() = _laser.header.stamp.toBoost();
    for (int i = 0; i < nr; i++) {
        MeasurementLaser::Beam &beam = measurement_laser_->operator[](i);
        beam.length = _laser.ranges[i];
        beam.angle = _laser.angle_min + (_laser.angle_increment * i);
        beam.end_point.x() = cos(beam.angle) * beam.length;
        beam.end_point.y() = sin(beam.angle) * beam.length;
    }
}

/**
 * copies incoming odometry messages to the base class
 * @param odom
 **/
void SelfLocalizationNode::callbackOdometry(const nav_msgs::Odometry &odom) {
    tf::Quaternion q;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double a = yaw;
    odom_.set(odom.pose.pose.position.x, odom.pose.pose.position.y, a);
}

/**
 * copies incoming robot pose messages to the base class
 * @param pose
 **/
void SelfLocalizationNode::callbackInitialpose(const geometry_msgs::PoseWithCovarianceStamped &pose) {
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.pose.pose.orientation, q);
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double a = yaw;
    pose_filter_->reinitialize(Pose2D(pose.pose.pose.position.x, pose.pose.pose.position.y, a));
}


/**
 * copies incoming robot command message
 * @param cmd
 **/
void SelfLocalizationNode::callbackCmd(const geometry_msgs::Twist &cmd) {
    cmd_.v() = cmd.linear.x;
    cmd_.w() = cmd.angular.z;
}

/**
 * copies incoming odometry messages to the base class
 * @param odom
 **/
void SelfLocalizationNode::callbackGroundTruth(const nav_msgs::Odometry &ground_truth) {
    tf::Quaternion q;
    tf::quaternionMsgToTF(ground_truth.pose.pose.orientation, q);
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double a = yaw;
    pose_ground_truth_.set(ground_truth.pose.pose.position.x, ground_truth.pose.pose.position.y, a);

    if (config_.initial_with_ground_truth) {
        ROS_INFO ("initial_with_ground_truth!");
        pose_filter_->reinitialize(pose_ground_truth_);
        config_.initial_with_ground_truth = false;
    }
}

/**
 * Publishes the estimated pose
 **/
void SelfLocalizationNode::publishPoseEstimated() {
    if (pose_filter_->time_last_update().is_not_a_date_time()) return;
    pose_.header.stamp.fromBoost(pose_filter_->time_last_update());
    pose_.header.seq++;
    pose_.pose.pose.position.x = pose_estimated_.x();
    pose_.pose.pose.position.y = pose_estimated_.y();
    pose_.pose.pose.position.z = 0;
    pose_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose_estimated_.theta());
    for (double &d: pose_.pose.covariance) d = 0;
    /// publishes motion command
    pub_pose_estimated_.publish(pose_);

    auto odom_frame_id = n_.getNamespace().length() == 1 ? "/odom" : n_.getNamespace() + "/odom";
    auto base_frame_id = n_.getNamespace().length() == 1 ? "/base_link" : n_.getNamespace() + "/base_link";
    auto map_frame_id = "map";

    /// publish estimated pose to tf tree
    // WARNING:
    //      odom -> base_link is published by other components, eg. stage, gazebo, robot, ...
    //      localization calculates map -> base_link
    //      Thus:  subtracting odom to base from map to base and send map to odom instead
    ros::Time update_time = ros::Time::fromBoost(pose_filter_->time_last_update());

    // Build a pose from base_link to map
    tf::Quaternion q;
    q.setRPY(0, 0, pose_estimated_.theta());
    tf::Transform tf_map_to_base(q, tf::Vector3(
            pose_estimated_.get_x(),
            pose_estimated_.get_y(),
            0
    ));
    geometry_msgs::PoseStamped base_to_map;
    base_to_map.header.frame_id = base_frame_id;
    base_to_map.header.stamp = pose_.header.stamp;
    tf::poseTFToMsg(tf_map_to_base.inverse(), base_to_map.pose);

    // TODO: Use dynamic parameters
    // Pose from odom to map
    geometry_msgs::PoseStamped odom_to_map_pose;
    tf_listener_->transformPose(odom_frame_id, base_to_map, odom_to_map_pose);

    tf::Transform odom_to_map_tf;
    tf::poseMsgToTF(odom_to_map_pose.pose, odom_to_map_tf);

    geometry_msgs::TransformStamped map_to_odom;
    map_to_odom.header.frame_id = pose_.header.frame_id;
    map_to_odom.header.stamp = ros::Time::fromBoost(pose_filter_->time_last_update());
    map_to_odom.header.seq = pose_.header.seq;
    map_to_odom.child_frame_id = odom_frame_id;
    tf::transformTFToMsg(odom_to_map_tf.inverse(), map_to_odom.transform);

    tf_broadcaster_->sendTransform(map_to_odom);

    if (pose_filter_->getTypeName() == "PARTICLE_FILTER") {
        auto particleFilterPtr = (ParticleFilter *) pose_filter_.get();

        geometry_msgs::PoseArray pose_array;
        pose_array.header.frame_id = map_frame_id;
        pose_array.header.stamp = pose_.header.stamp;
        pose_array.header.seq = pose_.header.seq;

        for (const SamplePtr &sptr: particleFilterPtr->samples) {
            geometry_msgs::Pose pose;

            pose.position.x = sptr->get_x();
            pose.position.y = sptr->get_y();
            pose.position.z = 0;

            tf::Quaternion q_tf;
            q_tf.setRPY(0, 0, pose_estimated_.theta());
            tf::quaternionTFToMsg(q_tf, pose.orientation);

            pose_array.poses.push_back(pose);
        }

        pub_pose_array_.publish(pose_array);
    }
}

/**
 * Publishes the map
 **/
void SelfLocalizationNode::publishMap() {
    nav_msgs::OccupancyGrid map;

    map.header.stamp = ros::Time::now();
    map.header.frame_id = "map";

    map.info.width = figure_map_.width();
    map.info.height = figure_map_.height();
    map.info.resolution = 1.0 / figure_map_.scale_x();
    map.info.map_load_time = ros::Time::now();

    // Origin
    geometry_msgs::Pose pose;
    pose.position.x = (figure_map_.width() / 2) / figure_map_.scale_x() * -1;
    pose.position.y = (figure_map_.height() / 2) / figure_map_.scale_y() * -1;
    map.info.origin = pose;

    // Actual data
    std::vector<int8_t> data(figure_map_.width() * figure_map_.height(), -1);

    cv::Mat map_image;
    map_image.create(figure_map_.height(), figure_map_.width(), CV_8UC3);
    cv::Mat map_image_gray;
    map_image_gray.create(figure_map_.height(), figure_map_.width(), CV_8UC1);
    cv::Mat raw_image = cv::imread(figure_map_.backgroundFileName(), cv::IMREAD_COLOR);
    cv::resize(raw_image, map_image, cv::Size(map_image.cols, map_image.rows),
               cv::INTER_AREA);
    cv::cvtColor(map_image, map_image_gray, cv::COLOR_BGR2GRAY);

    for (size_t i = 0; i < figure_map_.width() * figure_map_.height(); i++) {
        // Rotate 90deg and flip.
        size_t img_x = figure_map_.height() - i / figure_map_.height();
        size_t img_y = i % figure_map_.width();

        data.at(i) = (255 - map_image_gray.at<u_int8_t>(img_x, img_y)) / 255 * 100;
    }

    map.data = data;

    pub_map_.publish(map);
}
