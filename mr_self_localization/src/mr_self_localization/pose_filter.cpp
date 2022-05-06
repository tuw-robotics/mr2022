#include <mr_self_localization/pose_filter.h>
#include <opencv2/imgcodecs.hpp>

using namespace moro;

PoseFilter::PoseFilter ( Type type ) : reset_ ( true ), type_ ( type ), timestamp_last_update_() {
};
void PoseFilter::reset ( ) {
    reset_ = true;
}
void PoseFilter::setPoseInit ( const Pose2D &p ) {
    pose_init_ = p;
}
PoseFilter::Type  PoseFilter::getType() const {
    return type_;
}
const std::string PoseFilter::getTypeName() const {
    switch ( type_ ) {
    case PARTICLE_FILTER:
        return "PARTICLE_FILTER";
    case KALMAN_FILTER:
        return "KALMAN_FILTER";
    }
    return "NA";
}

const boost::posix_time::ptime& PoseFilter::time_last_update() const {
    return timestamp_last_update_;
}


const nav_msgs::OccupancyGrid& PoseFilter::get_map_msg_to_publish() const {
    return map_msg_to_publish;
}

void PoseFilter::loadMapToPublish ( int width_pixel, int height_pixel, double min_x, double max_x, double min_y, double max_y, double roation, const std::string &file ) {
    cv::Mat_<uint8_t> map;
    map.create ( height_pixel, width_pixel );
    cv::Mat image = cv::imread ( file, cv::IMREAD_GRAYSCALE );
    cv::resize ( image, map, cv::Size ( map.cols, map.rows ), cv::INTER_AREA );

    float resolution = (float) (max_x - min_x) / (float) height_pixel;

    map_msg_to_publish.info.height = height_pixel;
    map_msg_to_publish.info.width = width_pixel;
    map_msg_to_publish.info.resolution = resolution;
    // TODO  map_msg_to_publish.info.origin

    map_msg_to_publish.data.clear();
    for (int i = 0; i < map.rows; i++) {
        for (int k = 0; k < map.cols; k++) {
            auto val = map.at<uint8_t>(i,k);
            map_msg_to_publish.data.push_back(100 - round((double) val / 255 * 100));
        }
    }
}


///@return true on successful update, false on first use and if t is in the past
bool PoseFilter::updateTimestamp ( const boost::posix_time::ptime& t )  {
    if ( timestamp_last_update_.is_not_a_date_time() ) timestamp_last_update_ = t;
    if ( timestamp_last_update_ < t ) {
        duration_last_update_ = t - timestamp_last_update_;
        timestamp_last_update_ = t;
        return true;
    } else {
        return false;
    }
}
