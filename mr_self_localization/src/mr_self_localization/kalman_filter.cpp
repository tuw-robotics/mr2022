#include "mr_self_localization/kalman_filter.h"
#include <boost/lexical_cast.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <mr_geometry/linesegment2d_detector.h>
#include <iostream>
#include <rosconsole/macros_generated.h>

using namespace moro;

KalmanFilter::KalmanFilter()
        : PoseFilter(KALMAN_FILTER), figure_hspace_("Hough Space") {
}

void KalmanFilter::init() {
    pose_estimated_ = pose_init_;
    P = cv::Matx<double, 3, 3>(config_.init_sigma_location * config_.init_sigma_location, 0, 0,
                               0, config_.init_sigma_location * config_.init_sigma_location, 0,
                               0, 0, config_.init_sigma_orientation * config_.init_sigma_orientation);
    reset_ = false;


}

void KalmanFilter::detect_lines(const MeasurementLaserConstPtr &z) {
    LineSegment2DDetector linesegment_detector;
    linesegment_detector.config_.threshold_split_neighbor = config_.line_dection_split_neighbor;
    linesegment_detector.config_.threshold_split = config_.line_dection_split_threshold;
    linesegment_detector.config_.min_length = config_.line_dection_min_length;
    linesegment_detector.config_.min_points_per_line = config_.line_dection_min_points_per_line;
    linesegment_detector.config_.min_points_per_unit = config_.line_dection_min_points_per_unit;
    measurement_local_scanpoints_.resize(z->size());
    for (size_t i = 0; i < z->size(); i++) {
        measurement_local_scanpoints_[i] = z->pose2d().tf() * z->operator[](i).end_point;
    }
    measurement_linesegments_.clear();
    linesegment_detector.start(measurement_local_scanpoints_, measurement_linesegments_);

    measurement_match_.resize(measurement_linesegments_.size(), -1);
}


void KalmanFilter::plotData(Figure &figure_map) {
    plotMap(figure_map);
    if (config_.plot_hough_space) plotHoughSpace();
}

void KalmanFilter::plotMap(Figure &figure_map) {
    char text[0xFF];
    cv::Scalar color;

    /// Plot known line segments (map)
    for (size_t i = 0; i < map_linesegments_.size(); i++) {
        color = Figure::orange;
        figure_map.line(map_linesegments_[i].p0(), map_linesegments_[i].p1(), color, 1);
        figure_map.putText(boost::lexical_cast<std::string>(i), map_linesegments_[i].pc(), cv::FONT_HERSHEY_PLAIN, 0.6,
                           Figure::white, 3, cv::LINE_AA);
        figure_map.putText(boost::lexical_cast<std::string>(i), map_linesegments_[i].pc(), cv::FONT_HERSHEY_PLAIN, 0.6,
                           color, 1, cv::LINE_AA);
    }
    cv::Matx33d M = pose_estimated_.tf();

    for (size_t i = 0; i < measurement_linesegments_.size(); i++) {
        color = Figure::red;
        if (measurement_match_[i] >= 0) color = Figure::green_dark;
#if SELF_LOCALIZATION_EXERCISE >= 40
#else
        Point2D p0 = M * measurement_linesegments_[i].p0();
        Point2D p1 = M * measurement_linesegments_[i].p1();
        Point2D pc = M * measurement_linesegments_[i].pc();
        figure_map.line(p0, p1, color);
        figure_map.putText(boost::lexical_cast<std::string>(i), pc, cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white, 3,
                           cv::LINE_AA);
        figure_map.putText(boost::lexical_cast<std::string>(i), pc, cv::FONT_HERSHEY_PLAIN, 0.6, color, 1, cv::LINE_AA);
#endif
    }

    for (size_t i = 0; i < measurement_match_.size(); i++) {
        if (measurement_match_[i] >= 0) {
#if SELF_LOCALIZATION_EXERCISE >= 40
#else
            LineSegment2D measured = measurement_linesegments_[i];
            size_t match_i = measurement_match_[i];
            LineSegment2D predicted = predicted_linesegments_[match_i];

            Point2D p0_0 = M * predicted.closestPointTo(measured.p0());
            Point2D p0_1 = M * measured.p0();
            Point2D p1_0 = M * predicted.closestPointTo(measured.p1());
            Point2D p1_1 = M * measured.p1();

            figure_map.line(p0_0, p0_1, Figure::blue);
            figure_map.line(p1_0, p1_1, Figure::blue);
#endif
        }
    }


    sprintf(text, "%4.3fsec", duration_last_update_.total_microseconds() / 1000000.);
    cv::putText(figure_map.view(), text, cv::Point(figure_map.view().cols - 100, 20), cv::FONT_HERSHEY_PLAIN, 1,
                Figure::white, 3, cv::LINE_AA);
    cv::putText(figure_map.view(), text, cv::Point(figure_map.view().cols - 100, 20), cv::FONT_HERSHEY_PLAIN, 1,
                Figure::black, 1, cv::LINE_AA);
#if SELF_LOCALIZATION_EXERCISE >= 40
#else
    cv::Matx<double, 2, 2> E =
            figure_map.Mw2m().get_minor<2, 2>(0, 0) * P.get_minor<2, 2>(0, 0) * figure_map.Mm2w().get_minor<2, 2>(0, 0);
    cv::Mat_<double> eigval, eigvec;
    cv::eigen(E, eigval, eigvec);
    cv::RotatedRect ellipse(
            (figure_map.Mw2m() * pose_estimated_.position()).cv(),
            cv::Size(eigval(0) * figure_map.scale_x(), eigval(1) * figure_map.scale_y()),
            atan2(eigvec(0, 0), eigvec(1, 0)) * (180 / M_PI)
    );
    cv::ellipse(figure_map.view(), ellipse, Figure::magenta, 1, cv::LINE_AA);
#endif

    for (size_t i = 0; i < msgs_.size(); i++) {
        cv::putText(figure_map.view(), msgs_[i].c_str(), cv::Point(10, figure_map.view().rows - 12 * (i + 1)),
                    cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white, 3, cv::LINE_AA);
        cv::putText(figure_map.view(), msgs_[i].c_str(), cv::Point(10, figure_map.view().rows - 12 * (i + 1)),
                    cv::FONT_HERSHEY_PLAIN, 0.6, Figure::black, 1, cv::LINE_AA);
    }

    ///Plot estimated pose
    figure_map.symbol(pose_estimated_, 0.5, Figure::magenta, 1);
}

void KalmanFilter::plotHoughSpace() {
    if (figure_hspace_.initialized() == false) {
        figure_hspace_.setLabel("alpha=%4.2f", "rho=%4.2f");
        figure_hspace_.init(config_.hough_space_pixel_alpha, config_.hough_space_pixel_rho,
                            -M_PI * 1.1, +M_PI * 1.1,
                            -0.1, config_.hough_space_meter_rho,
                            M_PI,
                            1, M_PI / 4);

        if (config_.plot_hough_space) cv::namedWindow(figure_hspace_.title(), 1);
        if (config_.plot_hough_space) {
            cv::moveWindow(figure_hspace_.title(), 640, 20);
        }
    }
    figure_hspace_.clear();


    cv::Rect rectSpace(0, 0, figure_hspace_.view().cols, figure_hspace_.view().rows);
    for (unsigned int i = 0; i < measurement_local_scanpoints_.size(); i++) {

        Point2D p0 = measurement_local_scanpoints_[i];
        for (double alpha = -M_PI; alpha < M_PI; alpha += 1.0 / figure_hspace_.scale_x()) {
#if SELF_LOCALIZATION_EXERCISE >= 44
#else
            double a = alpha + M_PI;

            Polar2D polar;
            double rho = p0.get_x() * cos(a) + p0.get_y() * sin(a);
            polar = {a, rho};

            if (rho < 0) {
                polar.alpha() = angle_normalize(polar.alpha());
                polar.rho() = -polar.rho();
            }

            cv::Point hspace(figure_hspace_.w2m(polar).vector());

            if (hspace.inside(rectSpace)) {
                figure_hspace_.view().at<cv::Vec3b>(hspace) -= cv::Vec3b(50, 10, 10);
            }
#endif
        }
    }


    cv::Scalar color;
    Tf2D tf = figure_hspace_.Mw2m();
    for (size_t i = 0; i < predicted_linesegments_.size(); i++) {
        color = Figure::orange;
#if SELF_LOCALIZATION_EXERCISE >= 44
#else
        Polar2D polar = predicted_linesegments_[i].toPolar();

        polar.alpha() = angle_normalize(polar.alpha() + M_PI);

        figure_hspace_.circle(polar, 3, color, 1);
        figure_hspace_.putText(boost::lexical_cast<std::string>(i), polar, cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white,
                               3, cv::LINE_AA);
        figure_hspace_.putText(boost::lexical_cast<std::string>(i), polar, cv::FONT_HERSHEY_PLAIN, 0.6, color, 1,
                               cv::LINE_AA);
#endif
    }
    cv::RotatedRect ellipse;
    ellipse.angle = 0;
    ellipse.size.width = config_.data_association_line_alpha * figure_hspace_.scale_x() * 2.0;
    ellipse.size.height = config_.data_association_line_rho * figure_hspace_.scale_y() * 2.0;
    for (size_t i = 0; i < measurement_linesegments_.size(); i++) {
        Polar2D polar = measurement_linesegments_[i].toPolar();
        color = Figure::blue_dark;
#if SELF_LOCALIZATION_EXERCISE >= 44
#else
        polar.alpha() = angle_normalize(polar.alpha() + M_PI);

        ellipse.center = cv::Point(figure_hspace_.w2m(polar).vector());
        cv::ellipse(figure_hspace_.view(), ellipse, color, 1, cv::LINE_AA);
        figure_hspace_.putText(boost::lexical_cast<std::string>(i), polar, cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white,
                               3, cv::LINE_AA);
        figure_hspace_.putText(boost::lexical_cast<std::string>(i), polar, cv::FONT_HERSHEY_PLAIN, 0.6, color, 1,
                               cv::LINE_AA);
#endif
    }
    cv::imshow(figure_hspace_.title(), figure_hspace_.view());
}

void KalmanFilter::data_association() {
    if (config_.enable_data_association == false) return;

    predicted_linesegments_.resize(map_linesegments_.size());
    Tf2D M = pose_predicted_.tf().inv();
    for (size_t i = 0; i < predicted_linesegments_.size(); i++) {
#if SELF_LOCALIZATION_EXERCISE >= 43
#else
        auto p0 = M * map_linesegments_[i].p0();
        auto p1 = M * map_linesegments_[i].p1();

        predicted_linesegments_[i].set(p0, p1);
#endif
    }

    /// Match line segments in polar coordinates which are near to the robot
    Tf2D Mp2h = figure_hspace_.Mw2m();
    for (size_t i = 0; i < measurement_linesegments_.size(); i++) {
        Polar2D measurement = measurement_linesegments_[i].toPolar();
        float dMin = FLT_MAX;
        measurement_match_[i] = -1;
        for (size_t j = 0; j < predicted_linesegments_.size(); j++) {
            Polar2D prediction = predicted_linesegments_[j].toPolar();
#if SELF_LOCALIZATION_EXERCISE >= 43
#else
            double d = pow(measurement.distanceTo(prediction), 2);

            double alpha = abs(measurement.alpha() - prediction.alpha());
            double rho = abs(measurement.rho() - prediction.rho());

            if (d < dMin && alpha < config_.data_association_line_alpha && rho < config_.data_association_line_rho) {
                measurement_match_[i] = j;
                dMin = d;
            }
#endif
        }
    }


}

void KalmanFilter::reinitialize(const Pose2D &p) {
    setPoseInit(p);
    reset();
}


void KalmanFilter::loadMap(int width_pixel, int height_pixel, double min_y, double max_y, double min_x, double max_x,
                           double roation, const std::string &file) {
    init();
    cv::FileStorage fs(file, cv::FileStorage::READ);
    cv::Mat_<double> l;
    fs["line segments"] >> l;
    map_linesegments_.resize(l.rows);
    for (size_t i = 0; i < map_linesegments_.size(); i++) {
        map_linesegments_[i].set(l(i, 0), l(i, 1), l(i, 2), l(i, 3));
    }
}

Pose2D KalmanFilter::localization(const Command &u, const MeasurementConstPtr &z) {
    detect_lines((const MeasurementLaserConstPtr &) z);
    if (updateTimestamp(z->stamp())) {
        if (reset_) init();
        prediction(u);
        data_association();
        correction();
    }
    return pose_estimated_;
}

void KalmanFilter::setConfig(const void *config) {
    config_ = *((mr_self_localization::KalmanFilterConfig *) config);
}

void KalmanFilter::prediction(const Command &u) {
    x = pose_estimated_.state_vector();
    if (config_.enable_prediction) {
#if SELF_LOCALIZATION_EXERCISE >= 41
#else
        int ms = config_.forward_prediction_time * 1000;
        boost::posix_time::time_duration duration = duration_last_update_ + boost::posix_time::millisec(ms);
        double dt = duration.total_microseconds() / 1000000.;

        double th = x[2]; // Theta
        double v = u.v(); // Forward Velocity
        double w = u.w(); // Rotational Velocity
        if (abs(w) < 0.0001) { // TODO: Div. by zero. De L'Hospital?
            w = 0.0001;
        }

        double r = v / w;

        G = {
                1, 0, -r * cos(th) + r * cos(th + w * dt),
                0, 1, -r * sin(th) + r * sin(th + w * dt),
                0, 0, 1
        };

        V = {
                (-sin(th) + sin(th + w * dt)) / w,
                (v * (sin(th) - sin(th + w * dt))) / (w * w) + (v * cos(th + w * dt)) / w * dt,
                (cos(th) - cos(th + w * dt)) / w,
                (-v * (cos(th) - cos(th + w * dt))) / (w * w) + (v * sin(th + w * dt)) / w * dt,
                0, dt
        };

        M = {
                config_.alpha_1 * (v * v) + config_.alpha_2 * (w * w), 0,
                0, config_.alpha_3 * (v * v) + config_.alpha_4 * (w * w)
        };

        xp = x + cv::Vec<double, 3>(
                -r * sin(th) + r * sin(th + w * dt),
                r * cos(th) - r * cos(th + w * dt),
                w * dt
        );
        Pp = G * P * G.t() + V * M * V.t();
#endif
    } else {
        xp = x;
        Pp = P;
    }
    pose_predicted_ = xp;
}

void KalmanFilter::correction() {

    xc = pose_predicted_.state_vector();
    Pc = Pp;

    double dalpha, drho;
    Q = cv::Matx<double, 2, 2>(config_.sigma_alpha, 0, 0, config_.sigma_rho);
    char msg[0x1FF];
    msgs_.clear();
    for (size_t idx_measurement = 0; idx_measurement < measurement_match_.size(); idx_measurement++) {
        int idx_map = measurement_match_[idx_measurement];
        cv::Matx<double, 2, 3> H;   /// Check slides
        cv::Matx<double, 2, 1> v;  /// Measurement error between prediction (known data) and detection --> Siegwart;
        cv::Matx<double, 2, 2> Si;  /// Check slides
        cv::Matx<double, 1, 1> d_mahalanobis; // just for debugging reasons, not needed;
        cv::Matx<double, 3, 2> K;   /// Kalman gain
        cv::Matx<double, 3, 1> dx;  /// State change
#if SELF_LOCALIZATION_EXERCISE >= 42
#else
        if (idx_map == -1) {
            continue;
        }

        // TF from robot to world frame
        Tf2D Mr2w = pose_predicted_.tf();

        // Prediction in robot coordinate frame
        LineSegment2D prediction_robot = predicted_linesegments_[idx_map];
        LineSegment2D prediction_world = LineSegment2D(Mr2w * prediction_robot.p0(), Mr2w * prediction_robot.p1());
        Polar2D prediction_world_p = prediction_world.toPolar();

        // Measurement in robot coordinate frame
        LineSegment2D observation_robot = measurement_linesegments_[idx_measurement];
        LineSegment2D observation_world = LineSegment2D(Mr2w * observation_robot.p0(), Mr2w * observation_robot.p1());
        Polar2D observation_world_p = observation_world.toPolar();

        Polar2D z_hat;

        if (prediction_world_p.rho() >
            pose_predicted_.get_x() * cos(prediction_world_p.alpha()) +
            pose_predicted_.get_y() * sin(prediction_world_p.alpha())
                ) {
            z_hat.alpha() = prediction_world_p.alpha() - pose_predicted_.get_theta();
            z_hat.alpha() = angle_normalize(z_hat.alpha());
            z_hat.rho() = prediction_world_p.rho() - (
                    pose_predicted_.get_x() * cos(prediction_world_p.alpha()) +
                    pose_predicted_.get_y() * sin(prediction_world_p.alpha())
            );

            H = {
                    0, 0, -1,
                    -cos(prediction_world_p.alpha()), -sin(prediction_world_p.alpha()), 0
            };
        } else {
            z_hat.alpha() = prediction_world_p.alpha() + M_PI - pose_predicted_.get_theta();
            z_hat.alpha() = angle_normalize(z_hat.alpha());
            z_hat.rho() = (
                                  pose_predicted_.get_x() * cos(prediction_world_p.alpha()) +
                                  pose_predicted_.get_y() * sin(prediction_world_p.alpha())
                          ) - prediction_world_p.rho();

            H = {
                    0, 0, -1,
                    cos(prediction_world_p.alpha()), sin(prediction_world_p.alpha()), 0
            };
        }

        v = observation_robot.toPolar().vector() - z_hat.vector();

        Si = H * Pp * H.t() + Q;
        K = Pp * H.t() * Si.inv();
        dx = K * v;
        ROS_DEBUG("Correction: %f %f %f (x y th)", dx(0), dx(1), dx(2));

        cv::Matx<double, 3, 3> I = cv::Matx<double, 3, 3>::eye();

        xc = cv::Vec<double, 3>(xp(0) + dx(0), xp(1) + dx(1), xp(2) + dx(2));
        Pc = (I - K * H) * Pp;
#endif
    }

    if (config_.enable_correction) {
        pose_estimated_ = xc;
        P = Pc;
    } else {
        P = Pp;
        pose_estimated_ = pose_predicted_.state_vector();
    }
}
