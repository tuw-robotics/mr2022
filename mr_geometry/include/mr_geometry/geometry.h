#ifndef TUW_GEOMETRY
#define TUW_GEOMETRY

#include <mr_geometry/point2d.h>
#include <mr_geometry/polar2d.h>
#include <mr_geometry/pose2d.h>
#include <mr_geometry/line2d.h>
#include <mr_geometry/linesegment2d.h>
#include <mr_geometry/figure.h>
#include <mr_geometry/command.h>
#include <mr_geometry/measurement_laser.h>
#include <string>
#include <map>

namespace moro {
  enum DistributionType {
      NORMAL_DISTRIBUTION = 0,
      UNIFORM_DISTRIBUTION = 1,
      GRID_DISTRIBUTION = 2
  };
  static std::map<DistributionType, std::string> DistributionTypeName; 
}

#endif // TUW_GEOMETRY
