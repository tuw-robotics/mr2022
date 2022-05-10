#ifndef POSE_FILTER_H
#define POSE_FILTER_H

#include <ros/ros.h>
#include <memory>
#include <mr_geometry/geometry.h>
#include <boost/date_time/posix_time/ptime.hpp>

namespace moro {

/**
 * Base class for pose filters e.g. EKF (KALMAN_FILTER) or MCL (PARTICLE_FILTER)
 */
class PoseFilter;
typedef std::shared_ptr< PoseFilter > PoseFilterPtr;
typedef std::shared_ptr< PoseFilter const> PoseFilterConstPtr;
class PoseFilter {
public:
    enum Type {
        PARTICLE_FILTER = 0,
        KALMAN_FILTER = 1
    };
    /**
     * Constructor
     * @param type used identify the filter type
     **/
    PoseFilter(Type type);
    /**
     * set a reset  flag to reset the system on the next loop
     * @see PoseFilter::reset_ 
     **/
    void reset ( );
    /**
     * defines a init pose to which the system should be resetted if the PoseFilter::reset_ flag is set
     * @param p pose to reset
     **/
    void setPoseInit ( const Pose2D &p );;
    /**
     * Returns the filter type as enum
     * @returns enum
     * @see PoseFilter::Type
     **/
    Type getType() const;
    /**
     * Returns the filter type as name
     * @returns name
     * @see PoseFilter::Type
     **/
    const std::string getTypeName() const;
    /**
     * Time of the last processed measurement
     * @returns time
     **/
    const boost::posix_time::ptime& time_last_update() const;
    /**
     * virtual function to set the config parameters
     * @param config point to a autogenerated ros reconfiguration config
     **/
    virtual void setConfig ( const void *config ) = 0;
    /**
     * reinitializes the system with a pose
     * sets the reset flag and a init pose
     * @param p pose
     **/
    virtual void reinitialize (const Pose2D &p) = 0;
    /**
     * used to plot debug data into the map figure
     * @param figure_map 
     **/
    virtual void plotData ( Figure &figure_map ) = 0;
    /**
     * loads a given map 
     * @param width_pixel pixel size of the canvas
     * @param height_pixel pixel size of the canvas
     * @param min_x minimal x of the visualized space
     * @param max_x maximal x of the visualized space
     * @param min_y minimal y of the visualized space
     * @param max_y maximal y of the visualized space
     * @param rotation rotation of the visualized space
     * @param file map file
     **/
    virtual void loadMap ( int width_pixel, int height_pixel, double min_x, double max_x, double min_y, double max_y, double roation, const std::string &file ) = 0;    
    /**
     * starts the self-localization process and predicts the vehicles pose at the timestamp encoded into the measurement
     * @param u current control command
     * @param z measurement with a timestamp
     * @return estimated pose at the time of the measurement
     **/
    virtual Pose2D localization ( const Command &u, const MeasurementConstPtr &z ) = 0;
protected:
    /**
     * Inits the system
     **/
    virtual void init ( ) = 0;
    /**
     * updates the timestamp_last_update_ and the duration_last_update_
     * on the first call it will set the timestamp_last_update_ to t and the duration_last_update_ to zero
     * the duration_last_update_ will be used for the prediction step
     * @return true on successful update, false on first use and if t is in the past
     **/
    bool updateTimestamp(const boost::posix_time::ptime& t);
    bool reset_; /// on true the system should be resetted on the next loop
    Pose2D pose_init_;  /// reset pose
    Pose2D pose_estimated_;  /// computed estimated pose
    boost::posix_time::ptime timestamp_last_update_;  /// time of the last processed measurement
    boost::posix_time::time_duration duration_last_update_;  /// time since the previous processed measurement
private:  
    Type type_;
};
};

#endif // POSE_FILTER_H