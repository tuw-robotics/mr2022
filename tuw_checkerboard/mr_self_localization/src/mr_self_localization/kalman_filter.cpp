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
        /**
         * @node your code
         **/
        Point2D p0 = M * measurement_linesegments_[i].p0();
        Point2D p1 = M * measurement_linesegments_[i].p1();
        Point2D pc = M * measurement_linesegments_[i].pc();
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
            /**
             * @node your code
             **/
            int idx = measurement_match_[i];
            auto measurement = moro::LineSegment2D(M * measurement_linesegments_[i].p0(), M * measurement_linesegments_[i].p1());
            auto prediction = moro::LineSegment2D(M * predicted_linesegments_[idx].p0(), M * predicted_linesegments_[idx].p1());
            /*double m_vec_x = measurement.p1().get_x() - measurement.p0().get_x();
            double m_vec_y = measurement.p1().get_y() - measurement.p0().get_y();
            double m_vec_norm_x = m_vec_y;
            double m_vec_norm_y = -m_vec_x;
            double m_vec_length = sqrt(pow(m_vec_norm_x, 2) + pow(m_vec_norm_y, 2));
            double m_vec_unit_x = m_vec_norm_x/m_vec_length;
            double m_vec_unit_y = m_vec_norm_y/m_vec_length;*/

            double x0, y0, x1, y1;
            double dist_0 = prediction.distanceTo(measurement.p0(), x0, y0);
            double dist_1 = prediction.distanceTo(measurement.p1(), x1, y1);

            /*moro::Point2D measurement_intersect_p0(
                prediction.p0().get_x() + m_vec_unit_x * dist_0,
                prediction.p0().get_y() + m_vec_unit_y * dist_0);*/
            figure_map.line(measurement.p0(), moro::Point2D(measurement.p0().get_x() + x0, measurement.p0().get_y() + y0), Figure::blue);

            /*moro::Point2D measurement_intersect_p1(
                prediction.p1().get_x() + m_vec_unit_x * dist_1,
                prediction.p1().get_y() + m_vec_unit_y * dist_1);*/
            figure_map.line(measurement.p1(), moro::Point2D(measurement.p1().get_x() + x1, measurement.p1().get_y() + y1), Figure::blue);
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
    auto Mw2mVals = figure_map.Mw2m().val;
    cv::Matx<double, 2, 2> Mw2mSub (Mw2mVals[0], Mw2mVals[1], Mw2mVals[3], Mw2mVals[4]);
    cv::Matx<double, 2, 2> PSub (P.val[0], P.val[1], P.val[3], P.val[4]);
    cv::Matx<double, 2, 2> E = Mw2mSub * PSub * Mw2mSub.t(); 
    cv::Mat_<double> eigval, eigvec;
    cv::eigen ( E, eigval, eigvec );
    auto eigvecy = eigvec.at<double>(0, 1);
    auto eigvecx = eigvec.at<double>(0, 0);
    double angle = atan2(eigvecy, eigvecx) * 180. / M_PI;
    cv::Size size(sqrt(eigval.at<double>(0)), sqrt(eigval.at<double>(1)));
    cv::RotatedRect ellipse ( ( figure_map.Mw2m() * pose_estimated_.position() ).cv(), size, angle); /// must be changed
    cv::ellipse ( figure_map.view(),ellipse, Figure::magenta, 1, cv::LINE_AA );
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
            /**
             * @node your code
             **/
            moro::Point2D p1(p0.x() + cos(alpha), p0.y() + sin(alpha));
            moro::Polar2D polar = moro::Line2D(p0, p1).toPolar();
            cv::Point hspace = figure_hspace_.w2m(polar).cv();
            if ( hspace.inside ( rectSpace ) ) {
                figure_hspace_.view().at<cv::Vec3b> ( hspace ) -=  cv::Vec3b ( 50,10,10 );
            }
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
        /**
         * @node your code
         **/
        auto segm = map_linesegments_[i];
        predicted_linesegments_[i].set(M * segm.p0(), M * segm.p1());
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
            double dist0 = predicted_linesegments_[j].distanceTo(measurement_linesegments_[i].p0());
            double dist1 = predicted_linesegments_[j].distanceTo(measurement_linesegments_[i].p1());
            double dist = sqrt(pow(dist0,2) + pow(dist1,2));
            double angle_diff = abs(angle_difference(angle_normalize(measurement.alpha()), angle_normalize(prediction.alpha())));
            double rho_diff = abs(measurement.rho() - prediction.rho());
            if(dist < dMin 
                && dist < config_.data_association_distance_to_endpoints
                && rho_diff < config_.data_association_line_rho
                && angle_diff <= config_.data_association_line_alpha){
                dMin = dist;
                measurement_match_[i] = j;
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
        /**
         * @node your code
         **/
        double dt = duration_last_update_.total_microseconds() /1000000.;
        double theta = x[2];
        double v = u.v();
        double w = u.w();
        if(abs(w) < 0.001){
            double Gtx = -v * dt * sin(theta);
            double Gty = v * dt * cos(theta);
            cv::Matx<double, 3, 3 > Gt (1, 0, Gtx, 
                                        0, 1, Gty, 
                                        0, 0, 1 );

            double Vt1 = dt * cos(theta);
            double Vt2 = - pow(dt, 2) / 2 * v * sin(theta);
            double Vt3 = dt * sin(theta);
            double Vt4 = pow(dt, 2) / 2 * v * cos(theta);

            cv::Matx<double, 3, 2 > Vt (Vt1, Vt2, 
                                        Vt3, Vt4, 
                                        0,  dt);

            double Mt1 = config_.alpha_1 * pow(v, 2);
            double Mt4 = config_.alpha_3 * pow(v, 2);
            cv::Matx<double, 2, 2 > Mt ( Mt1,   0, 
                                        0, Mt4);    

            double xpbar1 = v * dt * cos(theta);    
            double xpbar2 = v * dt * sin(theta); 
            double xpbar3 = 0;
            cv::Vec3d xpbartmp (xpbar1, xpbar2, xpbar3);       
            xp = x + xpbartmp;
        
            Pp = Gt * P * Gt.t() + Vt * Mt * Vt.t();
        }
        else {
            double Gtx = -v/w * cos(theta) + v/w * cos(theta + w * dt);
            double Gty = -v/w * sin(theta) + v/w * sin(theta + w * dt);
            cv::Matx<double, 3, 3 > Gt (1, 0, Gtx, 
                                        0, 1, Gty, 
                                        0, 0, 1 );

            double Vt1 = (-sin(theta) + sin(theta + w*dt)) / w;
            double Vt2 = (v * (sin(theta) - sin(theta + w*dt))) / pow(w, 2) + (v * (cos(theta + w*dt) * dt)) / w;
            double Vt3 = (cos(theta) - cos(theta + w*dt)) / w;
            double Vt4 = -(v * (cos(theta) - cos(theta + w*dt))) / pow(w, 2) + (v * (sin(theta + w*dt) * dt)) / w;

            cv::Matx<double, 3, 2 > Vt (Vt1, Vt2, 
                                        Vt3, Vt4, 
                                          0,  dt);

            double Mt1 = config_.alpha_1 * pow(v, 2) + config_.alpha_2 * pow(w, 2);
            double Mt4 = config_.alpha_3 * pow(v, 2) + config_.alpha_4 * pow(w, 2);
            cv::Matx<double, 2, 2 > Mt (Mt1,   0, 
                                          0, Mt4);    

            double xpbar1 = -v/w * sin(theta) + v/w * sin(theta + w * dt);    
            double xpbar2 = v/w * cos(theta) - v/w * cos(theta + w * dt);  
            double xpbar3 = w * dt;
            cv::Vec3d xpbartmp (xpbar1, xpbar2, xpbar3);       
            xp = x + xpbartmp;
        
            Pp = Gt * P * Gt.t() + Vt * Mt * Vt.t();
        }
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
        if(idx_map != -1){
            double x = xc[0];
            double y = xc[1];
            double theta = xc[2];
            auto prediction = map_linesegments_[idx_map].toPolar();
            auto measurement = measurement_linesegments_[idx_measurement].toPolar();
            double alpha = prediction.alpha();
            double distance = x * cos(alpha) + y * sin(alpha);
            moro::Polar2D zhat;
            moro::Polar2D z = measurement;
            if(prediction.rho() > distance){
                H = cv::Matx<double, 2,3> (          0,           0, -1,
                                           -cos(alpha), -sin(alpha),  0);

                zhat = moro::Polar2D(prediction.alpha()-theta, prediction.rho() - distance);
            }
            else {
                H = cv::Matx<double, 2,3> (         0,          0, -1,
                                           cos(alpha), sin(alpha),  0);

                zhat = moro::Polar2D(prediction.alpha()-theta + M_PI, distance - prediction.rho());
            }
            
            v = cv::Matx<double, 2, 1>(angle_difference(z.alpha(), zhat.alpha()), z.rho() - zhat.rho());
            Si = H * Pc * H.t() + Q;
            // d_mahalanobis = ?
            K = Pc * H.t() * Si.inv();
            // dx = ?
            cv::Matx33d I;
            Pc = (I.eye() - K * H)*Pc;
            xc += K*v;
        }
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
