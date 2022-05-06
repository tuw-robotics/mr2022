#include "mr_self_localization/kalman_filter.h"
#include <boost/lexical_cast.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <mr_geometry/linesegment2d_detector.h>
#include <iostream>

const bool DEBUG_1_1 = true;
const bool DEBUG = false;

using namespace moro;
KalmanFilter::KalmanFilter()
    : PoseFilter ( KALMAN_FILTER )
    , figure_hspace_ ( "Hough Space" ) {
}

void KalmanFilter::init ( ) {
    pose_estimated_ = pose_init_;
    P = cv::Matx<double, 3, 3> ( config_.init_sigma_location*config_.init_sigma_location, 0, 0,
                                 0, config_.init_sigma_location*config_.init_sigma_location, 0,
                                 0, 0, config_.init_sigma_orientation*config_.init_sigma_orientation );
    reset_ = false;


}

void KalmanFilter::detect_lines ( const MeasurementLaserConstPtr &z ) {

    LineSegment2DDetector linesegment_detector;
    linesegment_detector.config_.threshold_split_neighbor = config_.line_dection_split_neighbor;
    linesegment_detector.config_.threshold_split = config_.line_dection_split_threshold;
    linesegment_detector.config_.min_length = config_.line_dection_min_length;
    linesegment_detector.config_.min_points_per_line = config_.line_dection_min_points_per_line;
    linesegment_detector.config_.min_points_per_unit = config_.line_dection_min_points_per_unit;
    measurement_local_scanpoints_.resize ( z->size() );
    for ( size_t i = 0; i < z->size(); i++ ) {
        measurement_local_scanpoints_[i] = z->pose2d().tf() * z->operator[] ( i ).end_point;
    }
    measurement_linesegments_.clear();
    linesegment_detector.start ( measurement_local_scanpoints_, measurement_linesegments_ );

    measurement_match_.resize ( measurement_linesegments_.size(), -1 );
}


void KalmanFilter::plotData ( Figure &figure_map ) {
    plotMap ( figure_map );
    if ( config_.plot_hough_space ) plotHoughSpace();
}

void KalmanFilter::plotMap ( Figure &figure_map ) {
    char text[0xFF];
    cv::Scalar color;

    /// Plot known line segments (map)
    for ( size_t i = 0; i < map_linesegments_.size(); i++ ) {
        color = Figure::orange;
        figure_map.line ( map_linesegments_[i].p0(), map_linesegments_[i].p1(), color, 1 );
        figure_map.putText ( boost::lexical_cast<std::string> ( i ),  map_linesegments_[i].pc(), cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white,  3, cv::LINE_AA );
        figure_map.putText ( boost::lexical_cast<std::string> ( i ),  map_linesegments_[i].pc(), cv::FONT_HERSHEY_PLAIN, 0.6, color, 1, cv::LINE_AA );
    }
    cv::Matx33d M = pose_estimated_.tf();
    for ( size_t i = 0; i < measurement_linesegments_.size(); i++ ) {
        color = Figure::red;
        if ( measurement_match_[i] >= 0 ) color = Figure::green_dark;
        /**
        * @ToDo visualize the measurement
        * After the prediction has been computed, visualize the outcome.
        * The first step here is to visualize all line segments using opencv
        **/
#if SELF_LOCALIZATION_EXERCISE >= 40
#else
        /**
         * @node your code
         **/
        
        Point2D p0 = M * measurement_linesegments_[i].p0();
        Point2D p1 = M * measurement_linesegments_[i].p1();
        Point2D pc = ( p0 + p1 ) / 2;
        figure_map.line ( p0, p1, color );
        figure_map.putText ( boost::lexical_cast<std::string> ( i ),  pc, cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white,  3, cv::LINE_AA );
        figure_map.putText ( boost::lexical_cast<std::string> ( i ),  pc, cv::FONT_HERSHEY_PLAIN, 0.6, color, 1, cv::LINE_AA );

        ROS_INFO_STREAM_COND(DEBUG, "2.1 @ToDo visualize the measurement");
        ROS_INFO_STREAM_COND(DEBUG, std::endl);
#endif

    }

    for ( size_t i = 0; i < measurement_match_.size(); i++ ) {
        if ( measurement_match_[i] >= 0 ) {
            /**
            * @ToDo visualize the matches
            * it is up to you how you visualize the relation
            * Visualize the matching linesegments in blue
            * the distanceTo function is used to find the matching endpoints
            */
#if SELF_LOCALIZATION_EXERCISE >= 40
#else
            /**
             * @node your code
             **/
            if(measurement_match_[i] >= 0) {
                Point2D p0 = map_linesegments_[measurement_match_[i]].p0();
                Point2D p1 = map_linesegments_[measurement_match_[i]].p1();
                figure_map.line ( p0, p1, Figure::blue_dark );
            }

            ROS_INFO_STREAM_COND(DEBUG, "2.4 @ToDo visualize the matches");

#endif

        }
    }


    sprintf ( text, "%4.3fsec", duration_last_update_.total_microseconds() /1000000. );
    cv::putText ( figure_map.view(), text, cv::Point ( figure_map.view().cols-100,20 ), cv::FONT_HERSHEY_PLAIN, 1, Figure::white,3, cv::LINE_AA );
    cv::putText ( figure_map.view(), text, cv::Point ( figure_map.view().cols-100,20 ), cv::FONT_HERSHEY_PLAIN, 1, Figure::black,1, cv::LINE_AA );

    /**
    * @ToDo visualize the pose covariance
    * Compute and plot the pose covariance in x and y direction
    * take the pose covariance P and create a 2x2 matrix out of the x,y components
    * transform the matrix into the plot E = Mw2m*P(0:1,0:1)*Mw2m'
    * use the opencv to compute eigenvalues and eigen vectors to compute the size and orientation of the ellipse
    * After the prediction, the ellipse of the covariance should be plotted here
    * Eigenvalues are used to visualize the ellipse with opencv ellipse
    * atan2 is needed to always return the angle from the x-axis and eliminate ambiguity (as opposed to atan)
    **/
#if SELF_LOCALIZATION_EXERCISE >= 40
#else
    /**
     * @node your code
     **/
    constexpr double RADTODEGREE = 57.295779513;
    auto Mw2m_2x2 = figure_map.Mw2m().get_minor<2,2>(0,0);
    auto P_2x2 = P.get_minor<2,2>(0,0);
    cv::Matx<double, 2, 2> E = Mw2m_2x2 * P_2x2 * Mw2m_2x2.t();  /// must be changed
    cv::Mat_<double> eigval, eigvec;
    cv::eigen ( E, eigval, eigvec );
    cv::RotatedRect ellipse ( ( figure_map.Mw2m() * pose_estimated_.position() ).cv(), cv::Size( sqrt(eigval.at<double>(0)), sqrt(eigval.at<double>(1)) ), atan2( eigvec.at<double>(0,1) , eigvec.at<double>(0,0) ) * RADTODEGREE ); /// must be changed
    cv::ellipse ( figure_map.view(), ellipse, Figure::magenta, 1, cv::LINE_AA );

    ROS_INFO_STREAM_COND(DEBUG, "1.2 @ToDo visualize the pose covariance");
    ROS_INFO_STREAM_COND(DEBUG, "eigenvalues: "  << eigval);
    ROS_INFO_STREAM_COND(DEBUG, "eigenvectors: " << eigvec);
    ROS_INFO_STREAM_COND(DEBUG, "orientation: "  << atan2( eigvec.at<double>(0,1) , eigvec.at<double>(0,0) )* RADTODEGREE);
    ROS_INFO_STREAM_COND(DEBUG, "position: "     << ( figure_map.Mw2m() * pose_estimated_.position() ).cv() << std::endl);

#endif

    for ( size_t i = 0; i < msgs_.size(); i++ ) {
        cv::putText ( figure_map.view(), msgs_[i].c_str(), cv::Point ( 10,figure_map.view().rows - 12* ( i+1 ) ), cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white,3, cv::LINE_AA );
        cv::putText ( figure_map.view(), msgs_[i].c_str(), cv::Point ( 10,figure_map.view().rows - 12* ( i+1 ) ), cv::FONT_HERSHEY_PLAIN, 0.6, Figure::black,1, cv::LINE_AA );
    }

    ///Plot estimated pose
    figure_map.symbol ( pose_estimated_, 0.5, Figure::magenta, 1 );
}
void KalmanFilter::plotHoughSpace ( ) {

    if ( figure_hspace_.initialized() == false ) {
        figure_hspace_.setLabel ( "alpha=%4.2f","rho=%4.2f" );
        figure_hspace_.init ( config_.hough_space_pixel_alpha, config_.hough_space_pixel_rho,
                              -M_PI*1.1, +M_PI*1.1,
                              0, config_.hough_space_meter_rho,
                              M_PI,
                              1, M_PI/4 );

        if ( config_.plot_hough_space ) cv::namedWindow ( figure_hspace_.title(), 1 );
        if ( config_.plot_hough_space ) {
            cv::moveWindow ( figure_hspace_.title(), 640, 20 );
        }
    }
    figure_hspace_.clear();

    cv::Rect rectSpace ( 0,0, figure_hspace_.view().cols, figure_hspace_.view().rows );
    ROS_INFO_STREAM_COND(DEBUG, "hspace size: cols: " << figure_hspace_.view().cols << ", rows: " << figure_hspace_.view().rows);
    ROS_INFO_STREAM_COND(DEBUG, "figure_hspace_.min_x(): " << figure_hspace_.min_x() << ", figure_hspace_.max_x(): " << figure_hspace_.max_x() );
    ROS_INFO_STREAM_COND(DEBUG, "figure_hspace_.scale_x(): " << figure_hspace_.scale_x() << ", figure_hspace_.scale_y(): " << figure_hspace_.scale_y() );

    for ( unsigned int i = 0; i < measurement_local_scanpoints_.size(); i++ ) {
        Point2D p0 = measurement_local_scanpoints_[i];
        for ( double alpha = figure_hspace_.min_x(); alpha < figure_hspace_.max_x(); alpha +=1.0/figure_hspace_.scale_x() ) {
            /**
            * @ToDo EKF
            * draw a wave with angle = [-pi...pi], r = x*cos(angle) + y *sin(angle) for every laser point [x,y].
            * The function Line2D::toPolar() can be used to transform a line into polar coordinates
            * Plot the lines (measurements) in hough coordinates.
            * Each laser scan point that corresponds to a line is transformed into hough coordinates here.
            * The intersection of all of them yields the resulting value for the hough representation of each line
            **/
#if SELF_LOCALIZATION_EXERCISE >= 44
#else
            /**
             * @node your code
             **/
            auto angle = alpha / 1.1;
            auto rho = p0.x() * cos(angle) + p0.y() * sin(angle);
            
            cv::Point hspace (( figure_hspace_.max_x() * 0.1 +  angle_normalize(angle, 0, 2 * M_PI)) * figure_hspace_.scale_x(), rho * figure_hspace_.scale_y() );
            if ( hspace.inside ( rectSpace ) ) {
                figure_hspace_.view().at<cv::Vec3b> ( hspace ) -=  cv::Vec3b (  50, 10, 10 );
            }

            // ROS_INFO_STREAM_COND(DEBUG, "2.5.1 @ToDo EKF - Plot the Hough-space waves for each laser scan beam.");
#endif
        }
    }


    cv::Scalar color;
    Tf2D tf = figure_hspace_.Mw2m();
    for ( size_t i = 0; i < predicted_linesegments_.size(); i++ ) {
        color = Figure::orange;
        /**
        * @ToDo Plot measurement prediction
        * the map prediction in the hough space as a circle or dot
        * The intersection point of all hough lines of all points on the predicted lines is visualized with a circle.
        */
#if SELF_LOCALIZATION_EXERCISE >= 44
#else
        /**
         * @node your code
         **/
        Polar2D polar = predicted_linesegments_[i].toPolar();
        figure_hspace_.circle ( polar, 3, color, 1 );
        figure_hspace_.putText ( boost::lexical_cast<std::string> ( i ),  polar, cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white,  3, cv::LINE_AA );
        figure_hspace_.putText ( boost::lexical_cast<std::string> ( i ),  polar, cv::FONT_HERSHEY_PLAIN, 0.6, color, 1, cv::LINE_AA );

        // ROS_INFO_STREAM_COND(DEBUG, "2.5.2 @ToDo Plot measurement prediction");
        // ROS_INFO_STREAM_COND(DEBUG, "polar: " << polar );
#endif
    }
    cv::RotatedRect ellipse;
    ellipse.angle  = 0;
    ellipse.size.width  = config_.data_association_line_alpha  * figure_hspace_.scale_x() * 2.0;
    ellipse.size.height = config_.data_association_line_rho * figure_hspace_.scale_y() * 2.0;
    for ( size_t i = 0; i < measurement_linesegments_.size(); i++ ) {
        Polar2D  polar = measurement_linesegments_[i].toPolar();
        color = Figure::blue_dark;
        /**
         * @ToDo Plot measurement
         * Plot the measurement prediction KalmanFilter::predicted_linesegments_ with an ellipse
         * to show the data association threshold config_.data_association_line_alpha and config_.data_association_line_rho.
         * Plot the prediction as an ellipsiod
         */
#if SELF_LOCALIZATION_EXERCISE >= 44
#else
        /**
         * @node your code
         **/
        ellipse.center = cv::Point2f(( figure_hspace_.max_x() * 0.1 +  angle_normalize(polar.alpha(), 0, 2*M_PI)) * figure_hspace_.scale_x(), polar.rho() * figure_hspace_.scale_y() );
        cv::ellipse ( figure_hspace_.view(), ellipse, color, 1, cv::LINE_AA );
        figure_hspace_.putText ( boost::lexical_cast<std::string> ( i ),  polar, cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white,  3, cv::LINE_AA );
        figure_hspace_.putText ( boost::lexical_cast<std::string> ( i ),  polar, cv::FONT_HERSHEY_PLAIN, 0.6, color, 1, cv::LINE_AA );

        // ROS_INFO_STREAM_COND(DEBUG, "2.5.3 @ToDo Plot measurement");
#endif
    }
    cv::imshow ( figure_hspace_.title(),figure_hspace_.view() );
}

void KalmanFilter::data_association ( ) {
    if ( config_.enable_data_association == false ) return;

    predicted_linesegments_.resize ( map_linesegments_.size() );
    Tf2D M = pose_predicted_.tf().inv();
    for ( size_t i = 0; i < predicted_linesegments_.size(); i++ ) {
        /**
        * @ToDo compute the measurement prediction
        * predicted_linesegments_[i].set ( .... )
        * Fix the data association, first the predicted linesegments are simply a copy of the map linesegments.
        * But the predicted ones reside within the coordinate frame of the prediction to ease the matching with the measurement (next step)
        */
#if SELF_LOCALIZATION_EXERCISE >= 43
#else
        /**
         * @node your code
         **/
        Point2D point0 = M * Point2D(map_linesegments_[i].x0(), map_linesegments_[i].y0());
        Point2D point1 = M * Point2D(map_linesegments_[i].x1(), map_linesegments_[i].y1());
        predicted_linesegments_[i].set( point0.x(), point0.y(), point1.x(), point1.y() );

        ROS_INFO_STREAM_COND(DEBUG, "2.2 @ToDo compute the measurement prediction");
        ROS_INFO_STREAM_COND(DEBUG, "point0:" << point0);
        ROS_INFO_STREAM_COND(DEBUG, "point1:" << point1 << std::endl);

#endif
    }

    /// Match line segments in polar coordinates which are near to the robot
    Tf2D Mp2h = figure_hspace_.Mw2m();
    for ( size_t i = 0; i < measurement_linesegments_.size(); i++ ) {
        Polar2D measurement = measurement_linesegments_[i].toPolar();
        float dMin = FLT_MAX;
        measurement_match_[i] = -1;
        for ( size_t j = 0; j < predicted_linesegments_.size(); j++ ) {
            Polar2D prediction = predicted_linesegments_[j].toPolar();
            /**
            * @ToDo matching measurement with prediction
            * find the best measurement prediction idx j and store it in measurement_match_[i]
            * Here the hough transform is used to match the lines as seen from the prediction (predicted_linesegments)
            * with the currently measured linesegments. Both are in the same coordinate space (prev step) and are transformed
            * into hough space (lines are points here and can be easily matched).
            * rqt allows for defining a matching threshold (distance)
            */
#if SELF_LOCALIZATION_EXERCISE >= 43
#else
            /**
             * @node your code
             **/
            double drho = abs(prediction.rho() - measurement.rho()), dalpha = angle_difference(prediction.alpha(), measurement.alpha());
            if( drho < config_.data_association_line_rho && dalpha < config_.data_association_line_alpha ) {
                if(measurement_match_[i] == -1) {
                    measurement_match_[i] = j;
                } else {
                    // compare with predicted_linesegments_[j] with predicted_linesegments_[measurement_match_[i]]
                    auto sum1 = 0, sum2 = 0;
                    Point2D p1, p2;
                    measurement_linesegments_[].alpha();
                    for(int a = 0; a < measurement_linesegments_[i].length(); i++) {
                        if( predicted_linesegments_[j].x0() < predicted_linesegments_[j].x1() ) {
                            if( predicted_linesegments_[j].y0() < predicted_linesegments_[j].y1() ) {
                                p1 = {predicted_linesegments_[j].x0() + i*dx, predicted_linesegments_[j].y0() + i*dy}; 
                                p2 = {predicted_linesegments_[measurement_match_[i]].x0() + i*dx, predicted_linesegments_[measurement_match_[i]].y0() + i*dy}; 
                            } else {
                                p1 = {predicted_linesegments_[j].x0() + i*dx, predicted_linesegments_[j].y0() - i*dy};
                                p2 = {predicted_linesegments_[measurement_match_[i]].x0() + i*dx, predicted_linesegments_[measurement_match_[i]].y0() - i*dy}; 
                            }
                        } else {
                            if( predicted_linesegments_[j].y0() < predicted_linesegments_[j].y1() ) {
                                p1 = {predicted_linesegments_[j].x0() - i*dx, predicted_linesegments_[j].y0() + i*dy}; 
                                p2 = {predicted_linesegments_[measurement_match_[i]].x0() - i*dx, predicted_linesegments_[measurement_match_[i]].y0() + i*dy}; 
                            
                            } else {
                                p1 = {predicted_linesegments_[j].x0() - i*dx, predicted_linesegments_[j].y0() - i*dy};
                                p2 = {predicted_linesegments_[measurement_match_[i]].x0() - i*dx, predicted_linesegments_[measurement_match_[i]].y0() - i*dy}; 
                            }
                        }
                        sum1 += predicted_linesegments_[j].distanceTo(p);
                        sum2 += predicted_linesegments_[measurement_match_[i]].distanceTo(p);
                    }
                    sum1 /= measurement_linesegments_[j].length();
                    sum2 /= measurement_linesegments_[j].length();

                    if(sum1 < sum2) {
                        measurement_match_[i] = j;
                    }
                }
            }        
            ROS_INFO_STREAM_COND(DEBUG, "2.3 @ToDo matching measurement with prediction");
            ROS_INFO_STREAM_COND(DEBUG, "Match: idx:" << i << ", match_idx: " << j);
#endif
        }
    }


}

void KalmanFilter::reinitialize ( const Pose2D &p ) {
    setPoseInit ( p );
    reset();
}


void KalmanFilter::loadMap ( int width_pixel, int height_pixel, double min_y, double max_y, double min_x, double max_x, double roation, const std::string &file ) {
    init();
    cv::FileStorage fs ( file, cv::FileStorage::READ );
    cv::Mat_<double> l;
    fs["line segments"] >> l;
    map_linesegments_.resize ( l.rows );
    for ( size_t i = 0; i < map_linesegments_.size(); i++ ) {
        map_linesegments_[i].set ( l ( i,0 ),l ( i,1 ), l ( i,2 ),l ( i,3 ) );
    }
}

Pose2D KalmanFilter::localization ( const Command &u, const MeasurementConstPtr &z ) {
    detect_lines ( ( const MeasurementLaserConstPtr& ) z );
    if ( updateTimestamp ( z->stamp() ) ) {
        if ( reset_ ) init();
        prediction ( u );
        data_association ( );
        correction ( );
    }
    return pose_estimated_;
}

void KalmanFilter::setConfig ( const void *config ) {
    config_ = * ( ( mr_self_localization::KalmanFilterConfig* ) config );
}

void KalmanFilter::prediction ( const Command &u ) {
    x = pose_estimated_.state_vector();
    if ( config_.enable_prediction ) {

        /**
        * @ToDo predict pose and covariance
        * compute KalmanFilter::xp and KalmanFilter::Pp as predicted pose and Covariance
        * Computes the prediction step of the Kalman filter as in thrun.
        **/
#if SELF_LOCALIZATION_EXERCISE >= 41
#else
        /**
         * @node your code
         **/

        double dt = duration_last_update_.total_microseconds() /1000000.;
        double theta = x[2];
        
        ROS_INFO_STREAM_COND(DEBUG_1_1, "1.1 @ToDo predict pose and covariance");
        ROS_INFO_STREAM_COND(DEBUG_1_1, "x: " << pose_estimated_.x());
        ROS_INFO_STREAM_COND(DEBUG_1_1, "y: " << pose_estimated_.y());
        ROS_INFO_STREAM_COND(DEBUG_1_1, "theta: " << pose_estimated_.theta());
        ROS_INFO_STREAM_COND(DEBUG_1_1, "v: " << u.v());
        ROS_INFO_STREAM_COND(DEBUG_1_1, "w: " << u.w() << std::endl);

        if( abs( u.w() ) > 0.00001  ) {
            double r = u.v() / u.w();
            G = {   1,  0,  -r * cos(theta) + r * cos(theta + u.w() * dt ),
                    0,  1,  -r * sin(theta) + r * sin(theta + u.w() * dt ),
                    0,  0,  1   };
            V = {   ( -sin(theta) + sin(theta + u.w() * dt ) ) / u.w(), (   u.v() * ( sin(theta) - sin(theta + u.w() * dt ) )) / pow( u.w(), 2 ) + ( u.v() * cos(theta + u.w() * dt ) * dt ) / u.w(), 
                    (  cos(theta) - cos(theta + u.w() * dt ) ) / u.w(), ( - u.v() * ( cos(theta) - cos(theta + u.w() * dt ) )) / pow( u.w(), 2 ) + ( u.v() * sin(theta + u.w() * dt ) * dt ) / u.w(), 
                    0,  dt    };
            M = {   config_.alpha_1 * pow( u.v(), 2 ) + config_.alpha_2 * pow( u.w(), 2 ),    0,
                    0,     config_.alpha_3 * pow( u.v(), 2 ) + config_.alpha_4 * pow( u.w(), 2 )  };
            xp = x + cv::Vec<double, 3>{ -r * sin(theta) + r * sin(theta + u.w() * dt ), r * cos(theta) - r * cos(theta + u.w() * dt ),  u.w() * dt };
            ROS_INFO_STREAM_COND(DEBUG_1_1, "w: " << u.w() << std::endl);
        } else {
            G = {   1,  0,  - u.v() * sin(theta) * dt,
                    0,  1,    u.v() * cos(theta) * dt,
                    0,  0,  1   };
            V = {   cos(theta) * dt, - u.v() * sin(theta) * pow(dt, 2) / 2, 
                    sin(theta) * dt,   u.v() * cos(theta) * pow(dt, 2) / 2, 
                    0,  dt    };
            M = {   config_.alpha_1 * pow( u.v(), 2 ) ,    0,
                    0,      config_.alpha_3 * pow( u.v(), 2 )  };
            xp = x + cv::Vec<double, 3>{ u.v() * cos(theta) * dt, u.v() * sin(theta) * dt, 0 };
        }
        R = V * M * V.t();
        Pp = G * P * G.t() + R;

        // xp = x;
        // Pp = P;

#endif
    } else {
        xp = x;
        Pp = P;
    }
    pose_predicted_ = xp;
}
void KalmanFilter::correction () {

    xc = pose_predicted_.state_vector();
    Pc = Pp;

    double dalpha, drho;
    Q = cv::Matx<double, 2,2> ( config_.sigma_alpha, 0, 0, config_.sigma_rho );
    char msg[0x1FF];
    msgs_.clear();
    for ( size_t idx_measurement = 0; idx_measurement < measurement_match_.size(); idx_measurement++ ) {
        int idx_map = measurement_match_[idx_measurement];
        cv::Matx<double, 2,3> H;   /// Check slides
        cv::Matx<double, 2, 1> v;  /// Measurement error between prediction (known data) and detection --> Siegwart;
        cv::Matx<double, 2,2> Si;  /// Check slides
        cv::Matx<double, 1,1> d_mahalanobis; // just for debugging reasons, not needed;
        cv::Matx<double, 3,2> K;   /// Kalman gain
        cv::Matx<double, 3,1> dx;  /// State change
        /**
        * @ToDo correction
        * Pose correction must update the KalmanFilter::xc and KalmanFilter::Pc which represents the corrected pose with covaraiance
        * have a look into Siegwart 2011 section 5.6.8.5 Case study: Kalman filter localization with line feature extraction
        * Siegwart correction implementation
        */
#if SELF_LOCALIZATION_EXERCISE >= 42
#else
        /**
         * @node your code 
         **/
        /// first the prediction and the measurement into polar space and compute the distance
        // cv::Matx<double, 2, 1> zp = { predicted_linesegments_[idx_measurement].toPolar().alpha(),   predicted_linesegments_[idx_measurement].toPolar().rho() }; 
        // cv::Matx<double, 2, 1> z  = { measurement_linesegments_[idx_measurement].toPolar().alpha(), measurement_linesegments_[idx_measurement].toPolar().rho() };
        // if (idx_map != -1) {
        //     auto mu_x = xc[0], mu_y = xc[1], mu_theta = xc[2];
        //     auto w_rho = map_linesegments_[idx_map].toPolar().rho();
        //     auto w_alpha = map_linesegments_[idx_map].toPolar().alpha();
        //     if(w_rho > mu_x * cos(w_alpha) + mu_y * sin(w_alpha)) {
        //         H = {0,0,-1, -cos(w_alpha), -sin(w_alpha), 0};
        //     } else {
        //         H = {0,0,-1, cos(w_alpha), sin(w_alpha), 0};
        //     }
        //     v = z - zp;
        //     Si = H * Pc * H.t() + Q;
        //     // d_mahalanobis = ?
        //     K = Pc * H.t() * Si.inv();
        //     // dx = ?
        //     cv::Matx<double, 3, 3> I;
        //     Pc = ( I.eye() - K * H ) * Pc;
        //     xc += K * v;
        // }

#endif
    }

    if ( config_.enable_correction ) {
        pose_estimated_ = xc;
        P = Pc;
    } else {
        P = Pp;
        pose_estimated_ =  pose_predicted_.state_vector();
    }
}

std::vector< SamplePtr > KalmanFilter::getSamples() const { return samples; }