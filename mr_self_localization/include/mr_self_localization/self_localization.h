#ifndef MR_H
#define MR_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <mr_geometry/geometry.h>
#include <mr_self_localization/particle_filter.h>
#include <mr_self_localization/kalman_filter.h>
#include <mr_self_localization/SelfLocalizationConfig.h>

namespace moro {
/**
 * Class for self-localization independent to the filter type
 */
class SelfLocalization {
public:
    /// Constructor
    SelfLocalization ( const std::string &ns );
    void init();     /// initialization
    void plot();     /// plots sensor input

    /**
     * function for userinteraction on the opencv window
     */
    static void onMouseMap ( int event, int x, int y, int, void* robot );
protected:
    std::string filename_map_image_;
    std::string filename_map_lines_;
    Point2D mouse_on_map_;
    Pose2D odom_;               /// State x, y, alpha
    Pose2D pose_ground_truth_;  /// State x, y, alpha
    Pose2D pose_estimated_;     /// State x, y, alpha
    Command cmd_;               /// output variables  v, w
    unsigned long loop_count_;  /// counts the filter cycles

    MeasurementLaserPtr measurement_laser_;    /// laser measurements

    Figure figure_map_;         /// figure to plot debug information
    PoseFilterPtr pose_filter_; /// filter used to estimate the vehicles pose
    ParticleFilterPtr particle_filter_; ///filter used to sen

    /**
     * starts the localization
     */
    void localization ();
    void plotMap();  /// plots sensor input
    mr_self_localization::SelfLocalizationConfig config_; /// global parameters
};
};

#endif // MR_H

