#include <mr_self_localization/particle_filter.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/math/distributions/normal.hpp> // for normal_distribution
#include <random>
#include <iostream>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>


using namespace moro;

std::random_device ParticleFilter::rd_;
std::mt19937 ParticleFilter::generator_ ( rd_() );
std::uniform_real_distribution<double> ParticleFilter::uniform_distribution_x_;
std::uniform_real_distribution<double> ParticleFilter::uniform_distribution_y_;
std::uniform_real_distribution<double> ParticleFilter::uniform_distribution_theta_;
std::normal_distribution<double> ParticleFilter::normal_distribution_;

ParticleFilter::ParticleFilter( ros::NodeHandle & n ) : PoseFilter ( PARTICLE_FILTER ) {
    normal_distribution_ = std::normal_distribution<double> ();
    sigma_likelihood_field_ = 1.0;
    
    pub_particles_ = n.advertise<geometry_msgs::PoseArray>("particles", 1);
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
    double epsilon = 0.00001;
    
    double v = u.v(), w = u.w();
    for ( SamplePtr s : samples ) {
        /**
        * @ToDo Motion model
        * implement the forward sample_motion_velocity algorithm and be aware that w can be zero!!
        * use the config_.alpha1 - config_.alpha6 as noise parameters
        * Executes the forward prediction step for each particle
        **/
#if SELF_LOCALIZATION_EXERCISE >= 13
#else        
        // sample perturbed control parameters              
        double v_hat = v + normal_distribution_ ( generator_ ) * (config_.alpha1 * v * v + config_.alpha2 * w * w);
        double w_hat = w + normal_distribution_ ( generator_ ) * (config_.alpha3 * v * v + config_.alpha4 * w * w);
        double gamma_hat = normal_distribution_ ( generator_ ) * (config_.alpha5 * v * v + config_.alpha6 * w * w);        
        // compute updates
        if (fabs(w_hat) <= epsilon){
            // TODO: manage this better, the teacher suggested the robot should go straight
            // if w_hat is ~0 then the robot is going straight
            dx = 0; //sin(s->theta() * v_hat * dt);
            dy = 0; //cos(s->theta() * v_hat * dt);
            dtheta = 0; //gamma_hat * dt;
        }
        else{
            // if w_hat is non-zero then we use v/w ratio
            double vw_ratio = v_hat / w_hat;
            dx = - vw_ratio * sin(s->theta()) + vw_ratio * sin(s->theta() + w_hat * dt);
            dy = vw_ratio * cos(s->theta()) - vw_ratio * cos(s->theta() + w_hat * dt);
            dtheta = w_hat * dt + gamma_hat * dt;
        }        
        // update pose
        s->set_x(s->x() + dx);
        s->set_y(s->y() + dy);
        s->set_theta(s->theta() + dtheta);
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
    cv::Mat& background = figure_map.background();
    for(size_t r=0; r < likelihood_field_.rows; r++){
        for(size_t c=0; c < likelihood_field_.cols; c++){            
            background.at<cv::Vec3b>(r, c)[0] = (1 - likelihood_field_[r][c]) * 255;            
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
        double x = s->x(), y= s->y(), w = s->weight();
        cv::Scalar colorBGR(scale * (samples_weight_max_ - w), scale * w, 0);
        Point2D pm(x, y);
        figure_map.symbol(pm, 0.1, colorBGR);
#endif
    }
    sprintf ( text, "%4.3fsec", duration_last_update_.total_microseconds() /1000000. );
    cv::putText ( figure_map.view(), text, cv::Point ( figure_map.view().cols-100,20 ), cv::FONT_HERSHEY_PLAIN, 1, Figure::white, 3, cv::LINE_AA );
    cv::putText ( figure_map.view(), text, cv::Point ( figure_map.view().cols-100,20 ), cv::FONT_HERSHEY_PLAIN, 1, Figure::black, 1, cv::LINE_AA );

    figure_map.symbol ( pose_estimated_, 0.5, Figure::magenta, 1 );
    
    // publish particles
    geometry_msgs::PoseArray particles_msg;
    particles_msg.header.frame_id = "map";
    particles_msg.header.stamp = ros::Time::now();
    for (size_t i=0; i<samples.size(); i++){        
        geometry_msgs::Pose pose;
        pose.position.x = samples[i]->x();
        pose.position.y = samples[i]->y();
        pose.position.z = 0.0;     
        tf::Quaternion q;
        q.setRPY(0, 0, samples[i]->theta());
        geometry_msgs::Quaternion gq; 
        tf::quaternionTFToMsg(q, gq);   // not found better way to obtain orientation from theta
        pose.orientation = gq;
        particles_msg.poses.push_back( pose );
    }
    pub_particles_.publish(particles_msg);
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
    cv::distanceTransform(map_, distance_field_pixel_, cv::DIST_L1, cv::DIST_MASK_3, CV_32F);
    
    float scaleLikelihood = boost::math::pdf(normal_likelihood_field, 0.0);   // likelihood=1 when distance=0 m
    for ( int r = 0; r < likelihood_field_.rows; r++ ) {
        for ( int c = 0; c < likelihood_field_.cols; c++ ) {
            float meterDist = distance_field_pixel_ ( r, c ) / scale_;
            float likelihood = boost::math::pdf(normal_likelihood_field, meterDist) / scaleLikelihood;
            likelihood_field_ ( r,c ) = likelihood;
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
    if (config_.random_beams){        
        for ( size_t i = 0; i < used_beams.size(); i++ ){
            double sample = (double) rand() / RAND_MAX;
            size_t beamID = round(sample * z->size());
            used_beams[i] = beamID;
        }
    } else {
        int intervalRays = floor( z->size() / (used_beams.size() - 1)); // -1 because we want to cover first and last beams
        for ( size_t i = 0; i < used_beams.size(); i++ ){
            used_beams[i] = i * intervalRays;            
            if (i * intervalRays >= z->size()){     // cap to max beams to ensure valid ids
                used_beams[i] = z->size() - 1;
            }            
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
    
    auto weight_sample = [&] ( SamplePtr &s ) {
        double tot_w = 1.0;        
        for ( size_t i : used_beams){
            auto b = z->operator[](i);
            if (b.length < config_.z_max) {
                // transform from robot- to map- frame
                Point2D endpoint = s->tf() * z->pose2d().tf() * b.end_point;
                double p_hit = 0.0;     // prob hit is 0.0 unless endpoint is within the map limits
                if (endpoint.x() >= min_x_ && endpoint.x() <= max_x_ &&
                    endpoint.y() >= min_y_ && endpoint.y() <= max_y_) {
                    // transform map- to pixel- frame
                    Point2D pixels = tf_ * endpoint;
                    // compute weight as in the book
                    p_hit = likelihood_field_( (int) round(pixels.y()), (int) round(pixels.x()) );                                        
                }   
                double w = (config_.z_hit * p_hit + (config_.z_rand / config_.z_max));                
                tot_w *= w;   
                                 
            }
        }
        s->weight() = tot_w;
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
    std::uniform_int_distribution<size_t>  uniform_idx_des ( 0, samples.size()-1 );
    /**
    * @ToDo Resample
    * implement a resample wheel
    * Implementation of 2 different resampling steps is required for the exercise
    * The code below only shows the low variance resampler from thrun.
    */
#if SELF_LOCALIZATION_EXERCISE >= 31
#else
    size_t nParticlesToResample = round( samples.size() * config_.resample_rate );    
    if (config_.resampling_mode == SIMPLE) {  
        ROS_INFO_STREAM("SIMPLE\n");
        // simple: discard the M least important particles and resample the most important ones with a certain perturbation
        // note: the particles are already sorted from highest to lowest importance
        for(size_t n = 0; n < nParticlesToResample; n++){            
            SamplePtr &badParticle  = samples[ samples.size() - nParticlesToResample + n ];
            SamplePtr &goodParticle  = samples[ n ];
            normal ( badParticle, *goodParticle, config_.sigma_static_position*dt, config_.sigma_static_orientation*dt );
        } 
    } else {
        // low_variance: systematic sampling
        // compute cdf of particle weights
        ROS_INFO_STREAM("LOW VARIANCE\n");
        std::vector<double> cdf;
        double sum = 0.0;
        for(size_t n = 0; n < samples.size(); n++){
            sum += samples[n]->weight();
            cdf.push_back(sum);
        }
        if (sum > 0){ // note: avoid resampling in first iteration because all particles have weight 0            
            double deltaN = 1.0 / nParticlesToResample;
            double r = d( generator_ ) * deltaN;
            size_t i = 0;
            //         
            for(size_t n = 0; n < nParticlesToResample; n++){            
                double u = r + n * deltaN;
                ROS_ASSERT(u >= 0 && u<=1);
                while (u > cdf[ i ]){                    
                    i++;
                    ROS_ASSERT(i >= 0 && i < samples.size());
                }
                SamplePtr &badParticle  = samples[ samples.size() - nParticlesToResample + n ];
                SamplePtr &goodParticle  = samples[ i ];            
                normal ( badParticle, *goodParticle, config_.sigma_static_position*dt, config_.sigma_static_orientation*dt );
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
