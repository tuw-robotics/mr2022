#include <mr_geometry/measurement.h>

using namespace moro;

Measurement::Measurement ( Measurement::Type type )
    : type_ ( type ) {};

Measurement::Measurement ( const Measurement &o )
    : type_ ( o.type_ ), stamp_ ( o.stamp_ ), pose_ ( o.pose_) {
}
Measurement::Type Measurement::getType() const {
    return type_;
}
const std::string Measurement::getTypeName() const {
    switch ( type_ ) {
    case Type::LASER:
        return "LASER";
    case Type::LINE:
        return "LINE";
    }
    return "NA";
}
const moro::Pose2D& Measurement::pose2d() const {
    return pose_;
}
moro::Pose2D& Measurement::pose2d() {
    return pose_;
}

const boost::posix_time::ptime& Measurement::stamp() const {
    return stamp_;
}
boost::posix_time::ptime& Measurement::stamp() {
    return stamp_;
}
