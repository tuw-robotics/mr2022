#include "mr_self_localization/kalman_filter.h"
#include <boost/lexical_cast.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <mr_geometry/linesegment2d_detector.h>
#include <iostream>

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
        Point2D p0 = M * measurement_linesegments_[ i ].p0();   // line-segments in sensor coordinates
        Point2D p1 = M * measurement_linesegments_[ i ].p1();
        Point2D pc = 0.5 * ( p0 + p1 );                         // put label at middle-point of the line segment
        figure_map.line ( p0, p1, color );
        figure_map.putText ( boost::lexical_cast<std::string> ( i ),  pc, cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white,  3, cv::LINE_AA );
        figure_map.putText ( boost::lexical_cast<std::string> ( i ),  pc, cv::FONT_HERSHEY_PLAIN, 0.6, color, 1, cv::LINE_AA );
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
            // idea: connect endpoints of matching lines
            color = Figure::blue;
            auto measurement = measurement_linesegments_[ i ];
            auto predicted = predicted_linesegments_[ measurement_match_[ i ] ];

            figure_map.line ( M * measurement.p0(), M * predicted.p0() , color );
            figure_map.line ( M * measurement.p1(), M * predicted.p1() , color );
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
    cv::Matx22d Pxy = P.get_minor<2, 2>(0, 0);
    cv::Matx22d Mw2m = figure_map.Mw2m().get_minor<2, 2>(0, 0);
    cv::Matx22d E = Mw2m * Pxy * Mw2m.t();    
    cv::Mat_<double> eigval, eigvec;
    cv::eigen ( E, eigval, eigvec );
    double eigenvec0 = eigvec(0), eigenvec1 = eigvec(1);
    double angle = atan2(eigenvec1, eigenvec0) * 180 / M_PI;
    cv::RotatedRect ellipse ( ( figure_map.Mw2m() * pose_estimated_.position() ).cv(),
                             cv::Size ( sqrt( eigval(0) ), sqrt( eigval(1) ) ),     // Hint by Eugen: the eigenvalues give the variance, we visualize the std dev
                             angle);
    cv::ellipse ( figure_map.view(), ellipse, Figure::magenta, 1, cv::LINE_AA );
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
            double r = p0.x() * cos( alpha ) + p0.y() * sin( alpha );            
            size_t i_alpha = ( alpha - figure_hspace_.min_x() ) / ( figure_hspace_.max_x() - figure_hspace_.min_x() ) * figure_hspace_.view().rows;
            size_t i_rho= ( 1 - ( r - figure_hspace_.min_y() ) / ( figure_hspace_.max_y() - figure_hspace_.min_y() ) ) * figure_hspace_.view().cols;
            cv::Point hspace(i_alpha , i_rho);
            if ( hspace.inside ( rectSpace ) ) {
                figure_hspace_.view().at<cv::Vec3b> ( hspace ) -=  cv::Vec3b ( 50,10,10 );
            }          
            // TODO: visualize lines
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
        auto line = predicted_linesegments_[ i ];
        Polar2D polar = line.toPolar();
        figure_hspace_.circle ( polar, 3, color, 1 );
        figure_hspace_.putText ( boost::lexical_cast<std::string> ( i ),  polar, cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white,  3, cv::LINE_AA );
        figure_hspace_.putText ( boost::lexical_cast<std::string> ( i ),  polar, cv::FONT_HERSHEY_PLAIN, 0.6, color, 1, cv::LINE_AA );        
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
        ellipse.center = (tf * polar).cv();
        cv::ellipse ( figure_hspace_.view(), ellipse, color, 1, cv::LINE_AA );
        figure_hspace_.putText ( boost::lexical_cast<std::string> ( i ),  polar, cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white,  3, cv::LINE_AA );
        figure_hspace_.putText ( boost::lexical_cast<std::string> ( i ),  polar, cv::FONT_HERSHEY_PLAIN, 0.6, color, 1, cv::LINE_AA );
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
        Point2D p0 = M * map_linesegments_[i].p0();   // transform from world coordinates to sensor coordinates
        Point2D p1 = M * map_linesegments_[i].p1();
        predicted_linesegments_[i].set( p0, p1 );
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
            // matching strategy: check if it is in the ellipsis delimited by the rho/alpha parameters
            // if this condition is met, then select the one which is closer w.r.t. dist in hough space
            double dRho= fabs( measurement.rho() - prediction.rho() );
            double dAlpha= fabs( measurement.alpha() - prediction.alpha() );
            double checkInsideEllipsis = pow(dRho, 2) / pow(config_.data_association_line_rho, 2) + pow(dAlpha, 2) / pow(config_.data_association_line_alpha, 2);
            double distance = pow( dRho, 2 ) + pow( dAlpha, 2 );
            if (checkInsideEllipsis <= 1.0 && distance < dMin){
                dMin = distance;
                measurement_match_[ i ] = j;
            }            
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
        int ms = config_.forward_prediction_time * 1000;
        boost::posix_time::time_duration duration = duration_last_update_ + boost::posix_time::millisec ( ms );
        double dt = duration.total_microseconds() /1000000.;

        double theta = x[2];

        cv::Vec<double, 3> dx;
        double v = u.v(), w = u.w(), ratio = u.v() / u.w();
        const double epsilon = 0.001;
        if ( fabs(w) < epsilon ) {
            // thanks wolfram: 1. compute limit on motion model for w->0. 2. compute jacobian w.r.t. state x and input u            
            // update motion matrices - G
            G = cv::Matx33d({
                1.0, 0.0, ( - dt * v * sin( theta ) ),
                0.0, 1.0, ( - dt * v * cos( theta ) ),
                0.0, 0.0, 1.0                
            });                        
            // update motion matrices - V
            V = cv::Matx32d({
                dt * cos ( theta ), pow(dt, 2) * v * sin(theta) / 2,
                dt * sin ( theta ), pow(dt, 2) * v * cos(theta) / 2,
                0.0, dt
            });            
            // update motion matrices - M
            M = cv::Matx22d({
                config_.alpha_1 * v * v, 0.0,
                0.0, config_.alpha_3 * v * v
            });
            // prediction step - update mean
            dx = cv::Vec<double, 3> ({
                dt * v * cos( theta ), 
                dt * v * sin( theta ),
                0.0
            });
        } else {            
            // update motion matrices - G
            G = cv::Matx33d({
                1.0, 0.0, ratio * (cos( theta + w * dt) - cos ( theta ) ),
                0.0, 1.0, ratio * (sin( theta + w * dt) - sin ( theta ) ),
                0.0, 0.0, 1.0                
            });                        
            // update motion matrices - V
            V = cv::Matx32d({
                1.0 / w * ( - sin( theta ) + sin ( theta + w * dt ) ),
                (v * (sin(theta) - sin(theta + w * dt))) / pow(w, 2) + ratio * cos(theta + w * dt) * dt,
                1.0 / w * ( cos ( theta ) - cos( theta + w * dt ) ),
                (- v * (cos(theta) - cos( theta + w*dt))) / pow(w, 2) + ratio * sin ( theta + w*dt) * dt,
                0.0, dt
            });            
            // update motion matrices - M
            M = cv::Matx22d({
                config_.alpha_1 * v * v + config_.alpha_2 * w * w, 0.0,
                0.0, config_.alpha_3 * v * v + config_.alpha_4 * w * w
            });
            // prediction step - update mean
            dx = cv::Vec<double, 3> ({
                - ratio * sin (theta) + ratio * sin (theta + w * dt), 
                + ratio * cos (theta) - ratio * cos (theta + w * dt),
                w * dt
            });
        }
        xp = x + dx;
        // prediction step - update covariance
        Pp = G * P * G.t() + V * M * V.t();
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
        * have alpha_world look into Siegwart 2011 section 5.6.8.5 Case study: Kalman filter localization with line feature extraction
        * Siegwart correction implementation
        */
#if SELF_LOCALIZATION_EXERCISE >= 42
#else
        if (idx_map < 0)
            continue;
        /// first the prediction and the measurement into polar space and compute the distance
        Polar2D z_world = map_linesegments_[idx_map].toPolar();
        Polar2D z_m_robot =  measurement_linesegments_[idx_measurement].toPolar();

        // unpack pose
        double x = xc[0], y = xc[1], theta = xc[2];
        // unpack measurement
        double alpha_m_robot = z_m_robot.alpha();
        double r_m_robot = z_m_robot.rho();

        cv::Matx<double, 2, 1> z_robot;
        double r_world = z_world.rho();
        double alpha_world = z_world.alpha();
        double r_proj_r = x * cos(alpha_world) + y * sin(alpha_world);  // line projected to pass through robot pose

        if (r_world > r_proj_r){
            z_robot = cv::Matx<double, 2, 1>(
                    angle_difference( alpha_world, theta),
                    r_world - r_proj_r
                    );
            H = cv::Matx<double, 2, 3>(
                    0, 0, -1,
                    -cos(alpha_world), -sin(alpha_world), 0
            );
        } else {
            z_robot = cv::Matx<double, 2, 1>(
                angle_difference(alpha_world,theta) + M_PI,
                    r_proj_r - r_world
            );
            H = cv::Matx<double, 2, 3>(
                    0, 0, -1,
                    cos(alpha_world), sin(alpha_world), 0
            );
        }

        v = cv::Matx<double, 2, 1>(angle_difference(alpha_m_robot, z_robot(0)),
                                   r_m_robot - z_robot(1));
        ///
        Si = H * Pp * H.t() + Q;
        d_mahalanobis = v.t() * Si * v;
        std::cout << "d mahalanobis: " << (double) d_mahalanobis(0) << "\n";
        ///
        K = Pp * H.t() * Si.inv();
        dx = K * v;
        /// update covariance and mean
        Pc = (cv::Matx33d::eye() - K * H) * Pp;
        xc(0) += dx(0);
        xc(1) += dx(1);
        xc(2) += dx(2);
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
