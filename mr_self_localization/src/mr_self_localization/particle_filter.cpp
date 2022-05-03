#include <mr_self_localization/particle_filter.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/math/distributions/normal.hpp> // for normal_distribution
#include <random>
#include <iostream>

using namespace moro;

std::random_device ParticleFilter::rd_;
std::mt19937 ParticleFilter::generator_(rd_());
std::uniform_real_distribution<double> ParticleFilter::uniform_distribution_x_;
std::uniform_real_distribution<double> ParticleFilter::uniform_distribution_y_;
std::uniform_real_distribution<double> ParticleFilter::uniform_distribution_theta_;
std::normal_distribution<double> ParticleFilter::normal_distribution_;

ParticleFilter::ParticleFilter() : PoseFilter(PARTICLE_FILTER) {
    normal_distribution_ = std::normal_distribution<double>();
    sigma_likelihood_field_ = 1.0;
}

SamplePtr &
ParticleFilter::normal(SamplePtr &sample, const Pose2D &mean, double sigma_position, double sigma_orientation) const {
    sample->set(mean.x() + normal_distribution_(generator_) * sigma_position,
                mean.y() + normal_distribution_(generator_) * sigma_position,
                mean.theta() + normal_distribution_(generator_) * sigma_orientation);
    sample->normalizeOrientation();
    return sample;
}

SamplePtr &ParticleFilter::uniform(SamplePtr &sample, std::uniform_real_distribution<double> distribution_x,
                                   std::uniform_real_distribution<double> distribution_y,
                                   std::uniform_real_distribution<double> distribution_theta) const {
    sample->set(distribution_x(generator_), distribution_y(generator_), distribution_theta(generator_));
    return sample;
}

void ParticleFilter::init() {
    samples.resize(config_.nr_of_samples);
    switch (config_.initial_distribution) {
        case NORMAL_DISTRIBUTION:
            initNormal();
            break;
        case UNIFORM_DISTRIBUTION:
            initUniform();
            break;
        case GRID_DISTRIBUTION:
            initGrid();
            break;
        default:
            initUniform();
    };
    reset_ = false;
}

void ParticleFilter::initNormal() {
    for (SamplePtr &s: samples) {
        s = std::make_shared<Sample>();
        normal(s, pose_init_, config_.sigma_init_position, config_.sigma_init_orientation);
    }
}

void ParticleFilter::initUniform() {
    for (SamplePtr &s: samples) {
        s = std::make_shared<Sample>();
        uniform(s, uniform_distribution_x_, uniform_distribution_y_, uniform_distribution_theta_);
    }
}

void ParticleFilter::reinitialize(const Pose2D &p) {
    setPoseInit(p);
    config_.initial_distribution = NORMAL_DISTRIBUTION;
    reset_ = true;
}

void ParticleFilter::initGrid() {
    float angle_division = 16;
    int i = 0;
    double samples_per_angle = config_.nr_of_samples / angle_division;
    double A = (max_x_ - min_x_) * (max_y_ - min_y_);
    double samples_per_m2 = samples_per_angle / A;
    double d = 1.0 / sqrt(samples_per_m2);
    samples.clear();
    samples.reserve(config_.nr_of_samples * 2);
    for (double x = min_x_ + d / 2.; x < max_x_; x += d) {
        for (double y = min_y_ + d / 2.; y < max_y_; y += d) {
            for (double theta = -M_PI; theta < M_PI; theta += (2. * M_PI) / angle_division) {
                samples.push_back(std::make_shared<Sample>());
                samples.back()->set(x, y, theta);
                samples.back()->idx() = i++;
            }
        }
    }
    config_.nr_of_samples = samples.size();

}

void ParticleFilter::update(const Command &u) {

    int ms = config_.forward_prediction_time * 1000;
    boost::posix_time::time_duration duration = duration_last_update_ + boost::posix_time::millisec(ms);
    double dt = duration.total_microseconds() / 1000000.;

    double v_sig = sqrt(config_.alpha1 * pow(u.v(), 2) + config_.alpha2 * pow(u.w(), 2));
    double w_sig = sqrt(config_.alpha3 * pow(u.v(), 2) + config_.alpha4 * pow(u.w(), 2));
    double y_sig = sqrt(config_.alpha5 * pow(u.v(), 2) + config_.alpha6 * pow(u.w(), 2));

    for (SamplePtr s: samples) {
#if SELF_LOCALIZATION_EXERCISE >= 13
#else
        double vH = u.v() + normal_distribution_(generator_) * v_sig;
        double wH = u.w() + normal_distribution_(generator_) * w_sig;
        double yH = normal_distribution_(generator_) * y_sig;

        if (wH == 0) { // Div. by zero
            wH = 0.00001;
        }

        double dx = -(vH / wH) * sin(s->theta()) + (vH / wH) * sin(s->theta() + wH * dt);
        double dy = +(vH / wH) * cos(s->theta()) - (vH / wH) * cos(s->theta() + wH * dt);
        double dtheta = s->theta() + wH * dt + yH * dt;

        s->set(s->get_x() + dx, s->get_y() + dy, s->get_theta() + dtheta);
#endif
    }
}

Pose2D ParticleFilter::localization(const Command &u, const MeasurementConstPtr &z) {
    if (updateTimestamp(z->stamp())) {
        updateLikelihoodField();
        if (reset_) init();
        if (config_.enable_resample) resample();
        if (config_.enable_update) update(u);
        if (config_.enable_weighting) weighting((const MeasurementLaserConstPtr &) z);
        pose_estimated_ = *samples[0];
    }
    return pose_estimated_;

}

void ParticleFilter::plotData(Figure &figure_map) {

#if SELF_LOCALIZATION_EXERCISE >= 20
#else
    for (int row = 0; row <= figure_map.background().rows; row++) {
        for (int col = 0; col <= figure_map.background().cols; col++) {
            float likelihood = likelihood_field_.at<float>(row, col);
            figure_map.background().at<cv::Vec3b>(row, col).val[0] = floor(255 - likelihood * 255);
        }
    }
#endif

#if SELF_LOCALIZATION_EXERCISE >= 12
#else
    std::for_each(samples.rbegin(), samples.rend(), [&](const SamplePtr& s) {
        double w = s->weight() / samples_weight_max_;
        figure_map.symbol(s->position(), 0.1, w * Figure::green + (1 - w) * Figure::blue);
    });

#endif
    char text[0xFF];

    sprintf(text, "%4.3fsec", duration_last_update_.total_microseconds() / 1000000.);
    cv::putText(figure_map.view(), text, cv::Point(figure_map.view().cols - 100, 20), cv::FONT_HERSHEY_PLAIN, 1,
                Figure::white, 3, cv::LINE_AA);
    cv::putText(figure_map.view(), text, cv::Point(figure_map.view().cols - 100, 20), cv::FONT_HERSHEY_PLAIN, 1,
                Figure::black, 1, cv::LINE_AA);

    figure_map.symbol(pose_estimated_, 0.5, Figure::magenta, 1);
}

void ParticleFilter::setConfig(const void *config) {
    config_ = *((mr_self_localization::ParticleFilterConfig *) config);
}

void ParticleFilter::loadMap(int width_pixel, int height_pixel, double min_x, double max_x, double min_y, double max_y,
                             double roation, const std::string &file) {
    width_pixel_ = width_pixel, height_pixel_ = height_pixel;
    min_y_ = min_y, max_y_ = max_y, min_x_ = min_x, max_x_ = max_x, roation_ = roation;
    double dx = max_x_ - min_x_;
    double dy = max_y_ - min_y_;
    double sy = height_pixel / dx;
    double sx = width_pixel / dy;
    double oy = height_pixel / 2.0;
    double ox = width_pixel / 2.0;
    double ca = cos(roation), sa = sin(roation);
    if (sy == sx) scale_ = sy;
    else {
        std::cerr << "loadMap: nonsymmetric scale!";
        return;
    }
    double owx = min_x_ + dx / 2.;
    double owy = min_y_ + dy / 2.;
    cv::Matx<double, 3, 3> Tw(1, 0, -owx, 0, 1, -owy, 0, 0, 1); // translation
    cv::Matx<double, 3, 3> Sc(sx, 0, 0, 0, sy, 0, 0, 0, 1); // scaling
    cv::Matx<double, 3, 3> Sp(-1, 0, 0, 0, 1, 0, 0, 0, 1);  // mirroring
    cv::Matx<double, 3, 3> R(ca, -sa, 0, sa, ca, 0, 0, 0, 1); // rotation
    cv::Matx<double, 3, 3> Tm(1, 0, ox, 0, 1, oy, 0, 0, 1); // translation
    tf_ = Tm * R * Sp * Sc * Tw;

    map_.create(height_pixel_, width_pixel_);
    distance_field_pixel_.create(height_pixel_, width_pixel_);
    likelihood_field_.create(height_pixel_, width_pixel_);
    cv::Mat image = cv::imread(file, cv::IMREAD_GRAYSCALE);
    cv::resize(image, map_, cv::Size(map_.cols, map_.rows), cv::INTER_AREA);

    uniform_distribution_x_ = std::uniform_real_distribution<double>(min_x_, max_x_);
    uniform_distribution_y_ = std::uniform_real_distribution<double>(min_y_, max_y_);
    uniform_distribution_theta_ = std::uniform_real_distribution<double>(-M_PI, M_PI);

    updateLikelihoodField();
}

void ParticleFilter::updateLikelihoodField() {

    if (sigma_likelihood_field_ == config_.sigma_hit) return;
    sigma_likelihood_field_ = config_.sigma_hit;
    boost::math::normal normal_likelihood_field;
    if (config_.sigma_hit > 0) {
        normal_likelihood_field = boost::math::normal(0, config_.sigma_hit);
    }

#if SELF_LOCALIZATION_EXERCISE >= 21
#else
    distance_field_.create(height_pixel_, width_pixel_);

    cv::distanceTransform(map_, distance_field_pixel_, cv::DIST_L2, cv::DIST_MASK_3);
    distance_field_ = distance_field_pixel_ / scale_;

    for (int row = 0; row <= likelihood_field_.rows; row++) {
        for (int col = 0; col <= likelihood_field_.cols; col++) {
            likelihood_field_.at<float>(row, col) = boost::math::pdf(normal_likelihood_field,
                                                                     distance_field_.at<float>(row, col));
        }
    }
#endif
}

void ParticleFilter::weighting(const MeasurementLaserConstPtr &z) {
    if (config_.nr_of_beams > z->size()) config_.nr_of_beams = z->size();
    std::vector<int> used_beams(config_.nr_of_beams); /// vector of beam indexes used
#if SELF_LOCALIZATION_EXERCISE >= 23
#else
    if (config_.random_beams) {
        std::vector<int> beams(z->size());
        std::iota(std::begin(beams), std::end(beams), 0);
        std::shuffle(beams.begin(), beams.end(), generator_);

        for (size_t i = 0; i < config_.nr_of_beams; i++) {
            used_beams[i] = beams[i];
        }
    } else {
        int d = floor(z->size() / config_.nr_of_beams);

        for (int i = 0; i < config_.nr_of_beams; i++) {
            used_beams[i] = i * d;
        }
    }
#endif

#if SELF_LOCALIZATION_EXERCISE >= 22
#else
    auto weight_sample = [&](SamplePtr &s) {
        double q = 1;

        for (int bi: used_beams) {
            auto b = (*z)[bi];

            cv::Matx33d endpoint_tf = tf_ * s->tf() * z->pose2d().tf();

            if (b.length < z->range_max()) {
                Pose2D end_m = endpoint_tf * b.end_point;
                int row = floor(end_m.get_y());
                int col = floor(end_m.get_x());

                float prob;
                if (row > likelihood_field_.rows ||
                    row < 0 ||
                    col > likelihood_field_.cols ||
                    col < 0
                ) {
                    prob = 0.0001;
                } else {
                    prob = likelihood_field_.at<float>(row, col);
                }

                q = q * (config_.z_hit * prob + (config_.z_rand / config_.z_max));
            }
        }

        s->weight() = q;
    };
#endif
    std::for_each(samples.begin(), samples.end(), weight_sample);
    std::sort(samples.begin(), samples.end(), Sample::greater);

    double samples_weight_sum = 0;
    for (const SamplePtr &s: samples) {
        samples_weight_sum += s->weight();
    }
    samples_weight_max_ = 0;
    for (size_t i = 0; i < samples.size(); i++) {
        SamplePtr &s = samples[i];
        s->weight() /= samples_weight_sum;
        s->idx() = i;
        if (samples_weight_max_ < s->weight()) {
            samples_weight_max_ = s->weight();
        }
        // std::cout << s->idx() << ": " << s->weight() << std::endl;
    }
}

void ParticleFilter::resample() {
    double dt = duration_last_update_.total_microseconds() / 1000000.;
    std::uniform_real_distribution<double> d(0, 1);
    std::uniform_int_distribution <size_t> uniform_idx_des(0, samples.size() - 1);

#if SELF_LOCALIZATION_EXERCISE >= 31
#else
    size_t divisor = floor(1.0 / config_.resample_rate);
    size_t M = floor(samples.size() / divisor);

    if (config_.resample_strategy == 0) {
        size_t offset = samples.size() - M;

        for (size_t i = 0; i < M; i++) {
            SamplePtr dst = samples[i + offset];
            dst->set(samples[i]);
    
            ParticleFilter::normal(dst, *dst, config_.sigma_static_position, config_.sigma_static_orientation);
        }
    } else {
        if (samples[0]->weight() == 0) {
            ROS_WARN("Cannot resample with 0 weight and low variance resampler");
        } else {
            std::vector<Sample> X;

            {
                std::uniform_real_distribution<double> rand(0, 1.0 / M);
                double r = rand(generator_);

                double c = samples[0]->weight();
                size_t i = 0;

                for (size_t m = 0; m < M; m++) {
                    double U = r + m * 1.0/M;

                    while (U > c) {
                        i = i + 1;
                        c = c + samples.at(i)->weight();
                    }

                    X.push_back(*samples.at(i));
                }
            }

            for (size_t i = 0; i < divisor; i ++) {
                for (size_t j = 0; j < X.size(); j++) {
                    SamplePtr sample = samples.at(i*X.size() + j);
                    sample->set(X.at(j));
                    ParticleFilter::normal(sample, *sample, config_.sigma_static_position, config_.sigma_static_orientation);
                }
            }
        }
    }
#endif
    /// update number of samples
    if (config_.nr_of_samples < samples.size()) samples.resize(config_.nr_of_samples);
    while (config_.nr_of_samples > samples.size()) {
        SamplePtr &parent = samples[uniform_idx_des(generator_)];
        double p = d(generator_);
        samples.push_back(std::make_shared<Sample>(*parent));
        SamplePtr &s = samples.back();
        normal(s, *s, config_.sigma_static_position * dt, config_.sigma_static_orientation * dt);
    }
}
