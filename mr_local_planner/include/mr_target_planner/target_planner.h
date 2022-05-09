#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <mr_geometry/geometry.h>

namespace moro {
/**
 * Robot class
 */
class TargetPlanner {
    public:
        TargetPlanner(); 
        void move();
    protected:
        Command cmd_;              /// output variables  v, w

};
} // namespace moro

#endif // PLANNER_LOCAL_H
