#include <mr_self_localization/particle_filter.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/math/distributions/normal.hpp> // for normal_distribution
#include <random>
#include <iostream>

using namespace moro;

std::random_device ParticleFilter::rd_;
std::mt19937 ParticleFilter::generator_ ( rd_() );
std::uniform_real_distribution<double> ParticleFilter::uniform_distribution_x_;
std::uniform_real_distribution<double> ParticleFilter::uniform_distribution_y_;
std::uniform_real_distribution<double> ParticleFilter::uniform_distribution_theta_;
std::normal_distribution<double> ParticleFilter::normal_distribution_;

ParticleFilter::ParticleFilter() :PoseFilter ( PARTICLE_FILTER ) {
    normal_distribution_ = std::normal_distribution<double> ();
    sigma_likelihood_field_ = 1.0;
}
SamplePtr& ParticleFilter::normal ( SamplePtr &sample, const Pose2D &mean, double sigma_position, double sigma_orientation ) const {
    sample->set ( mean.x() + normal_distribution_ ( generator_ ) * sigma_position, mean.y() + normal_distribution_ ( generator_ ) * sigma_position, mean.theta() + normal_distribution_ ( generator_ ) * sigma_orientation );
    sample->normalizeOrientation();
    return sample;
}

SamplePtr& ParticleFilter::uniform ( SamplePtr &sample, std::uniform_real_distribution<double> distribution_x, std::uniform_real_distribution<double> distribution_y, std::uniform_real_distribution<double> distribution_theta ) const {
    sample->set ( distribution_x ( generator_ ),  distribution_y ( generator_ ),  distribution_theta ( generator_ ) );
    return sample;
}
void ParticleFilter::init ( ) {
    samples.resize ( config_.nr_of_samples );
    switch ( config_.initial_distribution ) {
    case NORMAL_DISTRIBUTION:
        initNormal ();
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

void ParticleFilter::initNormal () {
    for ( SamplePtr &s: samples ) {
        s = std::make_shared<Sample>();
        normal ( s, pose_init_, config_.sigma_init_position, config_.sigma_init_orientation );
    }
}

void ParticleFilter::initUniform () {
    for ( SamplePtr &s: samples ) {
        s = std::make_shared<Sample>();
        uniform ( s, uniform_distribution_x_, uniform_distribution_y_, uniform_distribution_theta_ );
    }
}

void ParticleFilter::reinitialize ( const Pose2D &p ) {
    setPoseInit ( p );
    config_.initial_distribution = NORMAL_DISTRIBUTION;
    reset_ = true;
}

void ParticleFilter::initGrid () {
    float angle_division = 16;
    int i = 0;
    double samples_per_angle = config_.nr_of_samples / angle_division;
    double A = ( max_x_ - min_x_ ) * ( max_y_ - min_y_ );
    double samples_per_m2 = samples_per_angle / A ;
    double d =  1.0 / sqrt ( samples_per_m2 );
    samples.clear();
    samples.reserve ( config_.nr_of_samples*2 );
    for ( double x = min_x_ + d/2.; x < max_x_; x+=d ) {
        for ( double y = min_y_ + d/2.; y < max_y_; y+=d ) {
            for ( double theta = -M_PI; theta < M_PI; theta += ( 2.*M_PI ) / angle_division ) {
                samples.push_back ( std::make_shared<Sample>() );
                samples.back()->set ( x,y,theta );
                samples.back()->idx() = i++;
            }
        }
    }
    config_.nr_of_samples = samples.size();

}

void ParticleFilter::update ( const Command &u ) {

    int ms = config_.forward_prediction_time * 1000;
    boost::posix_time::time_duration duration = duration_last_update_ + boost::posix_time::millisec ( ms );
    double dx, dy, dtheta, dt = duration.total_microseconds() /1000000.;
    for ( SamplePtr s : samples ) {
        /**
        * @ToDo Motion model
        * implement the forward sample_motion_velocity algorithm and be aware that w can be zero!!
        * use the config_.alpha1 - config_.alpha6 as noise parameters
        * Executes the forward prediction step for each particle
        **/
#if SELF_LOCALIZATION_EXERCISE >= 13
#else
        /**
         * @node your code
         **/

        std::normal_distribution<double> d1{ 0, config_.alpha1*std::pow(u.v(), 2) + config_.alpha2*std::pow(u.w(), 2) };
        std::normal_distribution<double> d2{ 0, config_.alpha3*std::pow(u.v(), 2) + config_.alpha4*std::pow(u.w(), 2) };
        std::normal_distribution<double> d3{ 0, config_.alpha5*std::pow(u.v(), 2) + config_.alpha6*std::pow(u.w(), 2) };

        auto v_s = u.v() + d1(generator_);
        auto w_s = u.w() + d2(generator_);
        while( abs(w_s) < 0.000001 ) { w_s = u.w() + d2(generator_); }
        auto gamma_s = d3(generator_);

        dx = s->get_x() - ( v_s / w_s ) * sin(s->get_theta()) + ( v_s / w_s ) * sin(s->get_theta() + w_s * dt );
        dy = s->get_y() + ( v_s / w_s ) * cos(s->get_theta()) - ( v_s / w_s ) * cos(s->get_theta() + w_s * dt ); 
        dtheta = s->get_theta() + w_s * dt + gamma_s * dt;

        s->set(dx, dy, dtheta);
#endif
    }
}

Pose2D ParticleFilter::localization ( const Command &u, const MeasurementConstPtr &z ) {
    if ( updateTimestamp ( z->stamp() ) ) {
        updateLikelihoodField ();
        if ( reset_ ) init();
        if ( config_.enable_resample ) resample();
        if ( config_.enable_update ) update ( u );
        if ( config_.enable_weighting ) weighting ( ( const MeasurementLaserConstPtr& ) z );
        pose_estimated_ = *samples[0];
    }
    return pose_estimated_;

}

void ParticleFilter::plotData ( Figure &figure_map ) {
    /**
    * @ToDo Likelihood Field - Visualization
    * plot the likelihood_field_ into figure_map.background()
    * The likelihood field is visualized via opencv calls
    * figure_map.background() is the canvas and likelihood field has the values that are scaled for visualization
    * between 255 and 0
    **/
#if SELF_LOCALIZATION_EXERCISE >= 20
#else
    /**
     * @node your code
     **/   

    cv::Mat likelihood_field_scaled;
    likelihood_field_.convertTo(likelihood_field_scaled, CV_8UC1, 255.0);

    for(int r = 0; r < likelihood_field_.rows; ++r) {
        for(int c = 0; c < likelihood_field_.cols; ++c) {
            figure_map.background().at<cv::Vec3b>( r,c )[0] = ~likelihood_field_scaled.at<cv::int8_t>( r,c );
        }    
    }


#endif
    double scale =  255.0 / samples_weight_max_ ;
    char text[0xFF];
    for ( int i = samples.size()-1; i >= 0; i-- ) {
        const SamplePtr &s = samples[i];

        /**
        * @ToDo plot samples / particles
        * plot all samples use figure_map.symbol(....
        * All particles should be plotted with the symbol call.
        * @ToDo Visualize the Weights
        * Later those particles are colored by their weight (green high weight, blue low weight).
        **/
#if SELF_LOCALIZATION_EXERCISE >= 12
#else
        /**
         * @node your code
         **/
        auto diff = Figure::green - Figure::blue;
        figure_map.symbol(*s, .1, Figure::blue + diff * s->weight()/samples_weight_max_, 1, 1);
#endif
    }
    sprintf ( text, "%4.3fsec", duration_last_update_.total_microseconds() /1000000. );
    cv::putText ( figure_map.view(), text, cv::Point ( figure_map.view().cols-100,20 ), cv::FONT_HERSHEY_PLAIN, 1, Figure::white,3, cv::LINE_AA );
    cv::putText ( figure_map.view(), text, cv::Point ( figure_map.view().cols-100,20 ), cv::FONT_HERSHEY_PLAIN, 1, Figure::black,1, cv::LINE_AA );

    figure_map.symbol ( pose_estimated_, 0.5, Figure::magenta, 1 );
}

void ParticleFilter::setConfig ( const void *config ) {
    config_ = * ( ( mr_self_localization::ParticleFilterConfig* ) config );
}
void ParticleFilter::loadMap ( int width_pixel, int height_pixel, double min_x, double max_x, double min_y, double max_y, double roation, const std::string &file ) {
    width_pixel_ = width_pixel,   height_pixel_ = height_pixel;
    min_y_ = min_y, max_y_ = max_y, min_x_ = min_x, max_x_ = max_x, roation_ = roation;
    double dx = max_x_ - min_x_;
    double dy = max_y_ - min_y_;
    double sy = height_pixel / dx;
    double sx = width_pixel  / dy;
    double oy = height_pixel / 2.0;
    double ox = width_pixel  / 2.0;
    double ca = cos ( roation ), sa = sin ( roation );
    if ( sy == sx ) scale_ = sy;
    else {
        std::cerr << "loadMap: nonsymmetric scale!";
        return;
    }
    double owx = min_x_ + dx/2.;
    double owy = min_y_ + dy/2.;
    cv::Matx<double, 3, 3 > Tw ( 1, 0, -owx, 0, 1, -owy, 0, 0, 1 ); // translation
    cv::Matx<double, 3, 3 > Sc ( sx, 0, 0, 0, sy, 0, 0, 0, 1 ); // scaling
    cv::Matx<double, 3, 3 > Sp ( -1, 0, 0, 0, 1, 0, 0, 0, 1 );  // mirroring
    cv::Matx<double, 3, 3 > R ( ca, -sa, 0, sa, ca, 0, 0, 0, 1 ); // rotation
    cv::Matx<double, 3, 3 > Tm ( 1, 0, ox, 0, 1, oy, 0, 0, 1 ); // translation
    tf_ = Tm * R * Sp * Sc * Tw;

    map_.create ( height_pixel_, width_pixel_ );
    distance_field_pixel_.create ( height_pixel_, width_pixel_ );
    likelihood_field_.create ( height_pixel_, width_pixel_ );
    cv::Mat image = cv::imread ( file, cv::IMREAD_GRAYSCALE );
    cv::resize ( image, map_, cv::Size ( map_.cols, map_.rows ), cv::INTER_AREA );

    uniform_distribution_x_ =  std::uniform_real_distribution<double> ( min_x_, max_x_ );
    uniform_distribution_y_ = std::uniform_real_distribution<double> ( min_y_, max_y_ );
    uniform_distribution_theta_ = std::uniform_real_distribution<double> ( -M_PI, M_PI );

    updateLikelihoodField ();
}
void ParticleFilter::updateLikelihoodField () {

    if ( sigma_likelihood_field_ == config_.sigma_hit ) return;
    sigma_likelihood_field_ = config_.sigma_hit;
    boost::math::normal normal_likelihood_field;
    if ( config_.sigma_hit > 0 ) {
        normal_likelihood_field = boost::math::normal ( 0, config_.sigma_hit );
    }
    /**
    * @ToDo Computing the Likelihood Field
    * using the cv::distanceTransform and the boost::math::pdf
    * Here, the likelihood field is computed using the following steps.
    * First the distancetransform is called on the map which outputs the distance to the nearest pixels holding the value 0
    * of all other pixels.
    * To transform from pixels to metric values, the field has to be divided by scale_.
    * The pdf can then be used on the metric distance field to optain the likelihood values.
    **/
#if SELF_LOCALIZATION_EXERCISE >= 21
#else
    /**
     * @node your code
     **/

    cv::distanceTransform(map_, distance_field_pixel_, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_8U);
    distance_field_ = distance_field_pixel_ / scale_;

    for ( int r = 0; r < likelihood_field_.rows; r++ ) {
        for ( int c = 0; c < likelihood_field_.cols; c++ ) {
            float v = ( float ) c / ( float ) likelihood_field_.cols;
            // likelihood_field_ ( r,c ) = v;
            likelihood_field_ ( r,c ) = boost::math::pdf(normal_likelihood_field, distance_field_ ( r,c ) );
        }
    }
#endif
}

void ParticleFilter::weighting ( const MeasurementLaserConstPtr &z ) {
    if ( config_.nr_of_beams >  z->size() ) config_.nr_of_beams = z->size();
    std::vector<size_t> used_beams ( config_.nr_of_beams ); /// vector of beam indexes used

    /**
    * @ToDo Select beams for weighting
    * Select beams either randomly or equally distributed with a if-else statement below.
    **/
#if SELF_LOCALIZATION_EXERCISE >= 23
#else
    /**
     * @node your code
     **/
    /// Dummy: fills all values with the same value
    auto q = static_cast<int>( z->size() / used_beams.size());
    for ( size_t i = 0; i < used_beams.size(); i++ ) {
        used_beams[i] = i * q;
        if(used_beams[i] > z->size()) {
            used_beams[i] = 0;
        }
    }
#endif

    /**
    * @ToDo Computing the Weight
    * the used_beams should define the index of used laser beams
    * Since we are useing c++11 and later you have to to use the function object / lambda expression weight_sample
    * --> http://en.cppreference.com/w/cpp/algorithm/for_each
    * --> http://en.cppreference.com/w/cpp/language/lambda
    * The function computes the likelihood field lookup for each laser measurement viewed
    * in the coordinate system of each Particle.
    * The likelihood values are then computed as in the book and lecture slides
    * The resulting weight is stored inside each particle.
    */
#if SELF_LOCALIZATION_EXERCISE >= 22
#else
    /**
     * @node your code
     **/

    /// Dummy: computes a funny weight
    auto weight_sample = [&] ( SamplePtr &s ) {
        // s->weight() = sqrt ( s->x() *s->x() +s->y() *s->y() );
        s->weight() = 1;
        for(auto k : used_beams) {
            if (z->operator [](k).length != z->range_max() ) {
                auto z_map = tf_ * s->tf() * z->pose2d().tf() * z->operator [](k).end_point;
                if(z_map.get_x() >= 0 && z_map.get_x() < likelihood_field_.rows && z_map.get_y() >= 0 && z_map.get_y() < likelihood_field_.cols) {
                    s->weight() *= config_.z_hit * likelihood_field_(z_map.get_y(), z_map.get_x()) + config_.z_rand / config_.z_max;
                } else {
                    s->weight() *= config_.z_rand / config_.z_max; 
                }
            }
        }   
    };

#endif
    std::for_each ( samples.begin(), samples.end(), weight_sample );
    std::sort ( samples.begin(),  samples.end(), Sample::greater );

    double samples_weight_sum = 0;
    for ( const SamplePtr &s: samples ) {
        samples_weight_sum += s->weight();
    }
    samples_weight_max_ = 0;
    for ( size_t i = 0; i < samples.size(); i++ ) {
        SamplePtr &s = samples[i];
        s->weight() /= samples_weight_sum;
        s->idx() = i;
        if ( samples_weight_max_ < s->weight() ) samples_weight_max_ = s->weight();
        // std::cout << s->idx() << ": " << s->weight() << std::endl;
    }
}

void ParticleFilter::resample () {
    double dt = duration_last_update_.total_microseconds() /1000000.;
    std::uniform_real_distribution<double> d ( 0,1 );
    std::uniform_int_distribution<size_t>  uniform_idx_des ( 0,samples.size()-1 );
    /**
    * @ToDo Resample
    * implement a resample wheel
    * Implementation of 2 different resampling steps is required for the exercise
    * The code below only shows the low variance resampler from thrun.
    */
#if SELF_LOCALIZATION_EXERCISE >= 31
#else
    /**
     * @node your code
     **/
    int M = static_cast<int>(config_.resample_rate * samples.size());
    
    switch(config_.resampling_mode) {
        case 0:
        {
            std::vector< Sample > temp;
            temp.resize(M);
            for(int i = 0; i < M; ++i ) {
                temp[i] = *samples[i];
            }
            for( int i = 0; i < M; ++i ) {
                SamplePtr &s = samples[samples.size() -1- i];
                normal (s, temp[i], config_.sigma_static_position*dt, config_.sigma_static_orientation*dt );
            }
            break;
        }    
        case 1:
        {
            std::vector< SamplePtr > temp;  
            auto r = d ( generator_ ) / M;
            auto c = samples[0]->weight();
            for( auto m = 1, i = 0; m < M; ++m ) {
                auto u = r + ( m - 1 ) / M;
                while( u > c && i < samples.size() - 1 ) {
                    c += samples[i]->weight();
                    ++i;
                }
                temp.push_back(samples[i]);
            }
            if(samples.size() != temp.size()) {
                samples.resize(temp.size());
                samples = temp;
            }
            break;
        }
    }


#endif
    /// update number of samples
    if ( config_.nr_of_samples < samples.size() ) samples.resize ( config_.nr_of_samples );
    while ( config_.nr_of_samples > samples.size() ) {
        SamplePtr &parent = samples[uniform_idx_des ( generator_ )];
        double p = d ( generator_ );
        samples.push_back ( std::make_shared<Sample> ( *parent ) );
        SamplePtr &s  = samples.back();
        normal ( s, *s, config_.sigma_static_position*dt, config_.sigma_static_orientation*dt );
    }
}

std::vector< SamplePtr > ParticleFilter::getSamples() const { return samples; }