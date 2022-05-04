#include "mr_self_localization/self_localization.h"
#include <opencv2/core/core.hpp>
#include <boost/concept_check.hpp>

using namespace moro;

void SelfLocalization::onMouseMap ( int event, int x, int y, int flags, void* param ) {
    SelfLocalization *self_localization = ( SelfLocalization * ) param;
    self_localization->mouse_on_map_ =  self_localization->figure_map_.m2w ( Point2D ( x,y ) );
    static Pose2D pose;
    /**
     * @ToDo React on mouse clicks
     * catch the mouse and reset the filter
     * use the event CV_EVENT_LBUTTONDOWN and CV_EVENT_LBUTTONUP to define a pose with orientation
     * reset the filter with ground truth on the middle button
     * Capture the mouse events.
     * Left click -> set the pose to the mouse location
     * Left release -> reinitialize the filter on the previously selected pose
     * Middle mouse -> reinitialize the filter on the ground truth
     **/
#if SELF_LOCALIZATION_EXERCISE >= 30
#else
    /**
     * @node your code
     **/
    if ( event == cv::EVENT_LBUTTONDOWN ) {
        std::cout << self_localization->mouse_on_map_ << std::endl;
        pose = Pose2D(self_localization->mouse_on_map_);
    }
    if(event == cv::EVENT_LBUTTONUP){
        std:: cout << "L_UP"<< std::endl;
        double dx = self_localization->mouse_on_map_.x() - pose.x();
        double dy = self_localization->mouse_on_map_.y() - pose.y();
        pose.set_theta(std::atan2(dy,dx));
        self_localization->pose_filter_->reinitialize(pose);
    }
    if(event == cv::EVENT_MBUTTONDOWN){
        std:: cout << "Middle"<< std::endl;
        self_localization->pose_filter_->reinitialize(self_localization->pose_ground_truth_);
    }
#endif
}

SelfLocalization::SelfLocalization ( const std::string &ns )
    : loop_count_ ( 0 )
    , figure_map_ ( ns + ", Global View" ) {
    measurement_laser_ = std::make_shared<moro::MeasurementLaser>();   /// laser measurements
}

void SelfLocalization::init() {
    figure_map_.init ( config_.map_pix_x, config_.map_pix_y,
                       config_.map_min_x, config_.map_max_x,
                       config_.map_min_y, config_.map_max_y,
                       config_.map_rotation + M_PI,
                       config_.map_grid_x, config_.map_grid_y, filename_map_image_ );

    figure_map_.setLabel ( "x=%4.2f","y=%4.2f" );

    if ( config_.plot_data ) cv::namedWindow ( figure_map_.title(), 1 );
    if ( config_.plot_data ) cv::setMouseCallback ( figure_map_.title(), SelfLocalization::onMouseMap, this );
    if ( config_.plot_data ) {
        cv::moveWindow ( figure_map_.title(), 20, 20 );
    }

    std::string filename_map;
    if ( pose_filter_->getType() == PoseFilter::PARTICLE_FILTER ) filename_map =  filename_map_image_;
    if ( pose_filter_->getType() == PoseFilter::KALMAN_FILTER ) filename_map =  filename_map_lines_;

    pose_filter_->loadMap ( config_.map_pix_x, config_.map_pix_y,
                            config_.map_min_x, config_.map_max_x,
                            config_.map_min_y, config_.map_max_y,
                            config_.map_rotation + M_PI, filename_map );
}


void SelfLocalization::plot() {
    if ( config_.plot_data ) plotMap();
    cv::waitKey ( 10 );
}

void SelfLocalization::plotMap() {
    figure_map_.clear();
    char text[0xFF];

    //cv::Matx33d M = pose_ground_truth_.tf() * measurement_laser_->pose2d().tf();  /// for testing only
    cv::Matx33d M = pose_estimated_.tf() * measurement_laser_->pose2d().tf();


    for ( size_t i = 0; i < measurement_laser_->size(); i++ ) {
        /**
        * @ToDo plot sensor data
        * plot the laser data into the map and use the transformation measurement_laser_->pose2d().tf() as well!!
        * After you have finished the self-localization you mide want to use the pose_estimated_ instaead of pose_ground_truth_ to compute M
        **/
#if SELF_LOCALIZATION_EXERCISE >= 11
#else
        /**
         * @node your code
         **/
        figure_map_.symbol (M * ((*measurement_laser_)[i].end_point), Figure::red );

#endif

    }
    sprintf ( text, "%5lu,  <%+4.2fm, %+4.2f>", loop_count_, mouse_on_map_.x(), mouse_on_map_.y() );
    cv::putText ( figure_map_.view(), text, cv::Point ( 20,20 ), cv::FONT_HERSHEY_PLAIN, 1, Figure::white,3, cv::LINE_AA );
    cv::putText ( figure_map_.view(), text, cv::Point ( 20,20 ), cv::FONT_HERSHEY_PLAIN, 1, Figure::black,1, cv::LINE_AA );

    /**
    * @ToDo plot sensor data (your name)
    **/
    cv::putText ( figure_map_.view(), "Alexander Allacher", cv::Point ( 20,550 ), cv::FONT_HERSHEY_PLAIN, 1, Figure::black,1, cv::LINE_AA );
#if SELF_LOCALIZATION_EXERCISE >= 11
#else
    /**
     * @node your code
     **/
#endif
    figure_map_.symbol ( odom_, 0.2, Figure::cyan, 1 );
    figure_map_.symbol ( pose_ground_truth_, 0.2, Figure::orange, 1 );
    pose_filter_->plotData ( figure_map_ );
    cv::imshow ( figure_map_.title(),figure_map_.view() );
    cv::waitKey ( 10 );
}

void SelfLocalization::localization () {
    if ( measurement_laser_->empty() ) return;
    pose_estimated_ = pose_filter_->localization ( cmd_, ( MeasurementPtr ) measurement_laser_ );
    loop_count_++;
}

