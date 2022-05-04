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

        double theta = s->get_theta();

        std::normal_distribution<double> sample1 = std::normal_distribution<double> (0, config_.alpha1 * pow(u.v(),2) + config_.alpha2 * pow(u.w(),2));
        double new_v = u.v() + sample1(generator_);
        std::normal_distribution<double> sample2 = std::normal_distribution<double> (0, config_.alpha3 * pow(u.v(),2) + config_.alpha4 * pow(u.w(),2));
        double new_w = u.w() + sample2(generator_);
        std::normal_distribution<double> sample3 = std::normal_distribution<double> (0, config_.alpha5 * pow(u.v(),2) + config_.alpha6 * pow(u.w(),2));
        double new_y = sample3(generator_);
        

        if(abs(new_w) > 0.001) {
            dx = -new_v/new_w * sin(theta) + new_v/new_w * sin(theta + new_w*dt);
            dy = new_v/new_w * cos(theta) - new_v/new_w * cos(theta + new_w*dt);
            dtheta = new_w * dt + new_y * dt;
        }
        else {
            dx = cos(theta) * new_v * dt; 
            dy = sin(theta) * new_v * dt;
            dtheta = new_y * dt;
        }

        s->set(s->get_x() + dx, s->get_y() + dy, s->get_theta()+dtheta);
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

    cv::Mat background = figure_map.background();
    for(size_t x = 0; x < likelihood_field_.size().width; x ++){
        for(size_t y = 0; y < likelihood_field_.size().height; y ++){
            cv::Vec3b& color = background.at<cv::Vec3b>(cv::Point(x,y));
            if(color[0] == 255 && color[1] == 255 && color[2] == 255 )
            {
                double likelihood = likelihood_field_[y][x];
                color[2] = 255.;
                color[1] = 255.;
                color[0] = 255 - likelihood * 255.;
            }
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
        figure_map.symbol(*s, 0.1, {255 - s->weight()*scale, s->weight()*scale});
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

    distanceTransform(map_, distance_field_pixel_, 2, 0);
    
    for ( int r = 0; r < likelihood_field_.rows; r++ ) {
        for ( int c = 0; c < likelihood_field_.cols; c++ ) {
            likelihood_field_ ( r,c ) = boost::math::pdf(normal_likelihood_field, distance_field_pixel_(r,c)/scale_);
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
    for ( size_t i = 0; i < used_beams.size(); i++ ) {
        if(config_.random_beams)
            used_beams[i] = rand() % z->size();
        else
            used_beams[i] = z->size() / config_.nr_of_beams * i;
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
    double z_hit = config_.z_hit;
    double z_rand = config_.z_rand;
    double z_max = config_.z_max;
    auto weight_sample = [this, &z, used_beams, z_hit, z_rand, z_max] ( SamplePtr &s ) {
        double q = 1;

        cv::Matx33d M = this->tf_ * s->tf() * z->pose2d().tf();

        for(int k = 0; k<used_beams.size(); k++){
            auto end_point = (*z)[used_beams[k]].end_point;
            if((*z)[used_beams[k]].length >= z_max) continue;


            auto point = M * end_point;

            int x = int(std::round(point.x()));
            int y = int(std::round(point.y()));
            double likelihood = 0;
            if(point.inside(0, 0, likelihood_field_.cols-1, likelihood_field_.rows-1)){
                likelihood = this->likelihood_field_[y][x];
            }
            //x = std::min(this->likelihood_field_.cols-1, std::max(0, x));
            //y = std::min(this->likelihood_field_.rows-1, std::max(0, y));
            q = q * (z_hit * likelihood + z_rand/z_max);

        }

        s->weight() = q;
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
        //std::cout << s->idx() << ": " << s->weight() << std::endl;
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

    int count_new_samples = samples.size() * config_.resample_rate;
    
    if(config_.resample_strategy == 0){
        auto start = samples.end() - count_new_samples;
        auto end = samples.end();
        for (auto i = start; i != end; i++){
            samples.erase(i);
        }
        for(auto i = samples.begin(); i != samples.begin() + count_new_samples; i++){
            SamplePtr &parent = *i;
            samples.push_back(std::make_shared<Sample> ( *parent ));
            SamplePtr &s  = samples.back();
            normal ( s, *s, config_.sigma_static_position*dt, config_.sigma_static_orientation*dt );
        }
    }
    else if(config_.resample_strategy == 1){
        std::vector<moro::SamplePtr> samplesCopy;
        for (int i=0; i<samples.size(); i++)
            samplesCopy.push_back(samples[i]);

        for(int i = 0; i < count_new_samples; i++){
            samples.pop_back();
        }
        
        double step_size = 1 / count_new_samples;
        double weight_threshold = ((double)rand() / RAND_MAX) * step_size;
        double current_weight_sum = 0;
        for ( const SamplePtr &s: samplesCopy ) {
            current_weight_sum += s->weight();
            if(current_weight_sum > weight_threshold){
                samples.push_back(std::make_shared<Sample> ( *s ));
                SamplePtr &s  = samples.back();
                normal ( s, *s, config_.sigma_static_position*dt, config_.sigma_static_orientation*dt );

                weight_threshold += step_size;
            }
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
