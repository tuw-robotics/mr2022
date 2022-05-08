#include "micropather.h"
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <mr_geometry/geometry.h>
#include <ros/ros.h>

namespace moro {

    struct MapOptions {
        int scale_factor;
        uint8_t blur_iterations;
        uint8_t blur_size;
        bool allow_diagonal_movement;
        uint8_t occupancy_threshold;
    };
    class Map : public micropather::Graph {
        public:
            Map(nav_msgs::OccupancyGrid map_data, MapOptions options);
            virtual float LeastCostEstimate( void* stateStart, void* stateEnd );
            virtual void AdjacentCost( void* state, MP_VECTOR< micropather::StateCost > *adjacent );
            virtual void  PrintStateInfo( void* state );

            void* worldToNode(Pose2D point) {
                int x = ((point.get_x() - origin_x_)*width_) / (width_ * resolution_);
                int y = ((point.get_y() - origin_y_)*height_) / (height_ * resolution_);
                return xyToNode(x, y);
            };

            Point2D nodeToWorld(void* node) 
            {
                auto xy = nodeToXY(node);
                return Point2D(xy.first*resolution_ + origin_x_, xy.second*resolution_ + origin_y_);
            };
        private:
            cv::Mat map_;
            uint8_t occupancy_theshold_;    
            float resolution_;
            float origin_x_;
            float origin_y_;
            double rotation_;
            int width_;
            int height_;
            bool allow_diagonal_movement_;

            std::pair<int, int> nodeToXY( void* node) 
            {
                intptr_t index = (intptr_t)node;
                int y = index / width_;
                return {index - y * width_, y};
            };

            void* xyToNode( int x, int y )
            {
                // this cast is quite ugly and might not be compliant with systems 
                // where sizeof(int) > sizeof(void*), but unfortunately micropather
                // expects abuses void pointers instead of using some safer alternative
                return reinterpret_cast<void*>(static_cast<intptr_t>( y*width_ + x ));
            };

            uint8_t get(int x, int y) {
                if(x >= width_ || y >= height_) {
                    ROS_INFO("Exceeded!");
                    return 0;
                } else {
                    return map_.at<uint8_t>(y, x);
                }
            };
    };
}
