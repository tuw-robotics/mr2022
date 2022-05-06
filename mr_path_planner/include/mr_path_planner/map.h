#include "micropather.h"
#include <vector>
#include <nav_msgs/MapMetaData.h>

namespace moro {
    class Map : public micropather::Graph {
        public:
            Map(std::vector<int8_t> map_data, nav_msgs::MapMetaData map_metadata, int8_t occupancy_threshold);
            virtual float LeastCostEstimate( void* stateStart, void* stateEnd );
            virtual void AdjacentCost( void* state, MP_VECTOR< micropather::StateCost > *adjacent );
            virtual void  PrintStateInfo( void* state );

            void* worldToNode(float x_world, float y_world) {
                int x = ((x_world - origin_x_)*width_) / (width_ * resolution_);
                int y = ((y_world - origin_y_)*height_) / (height_ * resolution_);
                return xyToNode(x, y);
            }

            std::pair<double,double> nodeToWorld(void* node) 
            {
                auto xy = nodeToXY(node);
                return {xy.first*resolution_ + origin_x_, xy.second*resolution_ + origin_y_};
            }
        private:
            int8_t occupancy_theshold_;
            std::vector<int8_t> map_data_;
            uint32_t width_;
            uint32_t height_;
            float resolution_;
            float origin_x_;
            float origin_y_;

            std::pair<int,int> nodeToXY( void* node) 
            {
                intptr_t index = (intptr_t)node;
                int y = index / width_;
                return {index - y * width_, y};
            }

            void* xyToNode( int x, int y )
            {
                return (void*) ( y*width_ + x );
            }

            uint8_t get(int x, int y) {
                return map_data_.at(y*width_ + x);
            }
    };
}
