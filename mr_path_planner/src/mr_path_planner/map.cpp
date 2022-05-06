#include "mr_path_planner/map.h";
#include <math.h>;

namespace moro {

    Map::Map(std::vector<int8_t> map_data, nav_msgs::MapMetaData map_metadata, int8_t occupancy_theshold): micropather::Graph() {
        map_data_ = map_data;
        width_ = map_metadata.width;
        height_ = map_metadata.height;
        resolution_ = map_metadata.resolution;
        origin_x_ = map_metadata.origin.position.x;
        origin_y_ = map_metadata.origin.position.y;
        occupancy_theshold_ = occupancy_theshold;
    }

    float Map::LeastCostEstimate( void* stateStart, void* stateEnd ) {
        auto start = nodeToXY(stateStart);
        auto end = nodeToXY(stateEnd);

        return sqrt(pow(start.first - end.first, 2) + pow(start.second - end.second, 2));
    }

    void Map::AdjacentCost( void* state, MP_VECTOR< micropather::StateCost > *adjacent ) {
        auto node = nodeToXY(state);

        std::vector<std::pair<int, int>> neighbors {{node.first+1, node.second}, {node.first-1, node.second}, {node.first, node.second+1}, {node.first, node.second-1}};
        for (auto neighbor : neighbors) {
            if(neighbor.first >= 0 && neighbor.first < width_ && 
               neighbor.second >= 0 && neighbor.second < height_ && 
               get(neighbor.first, neighbor.second) < occupancy_theshold_) {
                   micropather::StateCost nodeCost = { xyToNode( neighbor.first, neighbor.second ), 1.0 };
                   adjacent->push_back(nodeCost);
            }
        }
    }

    void Map::PrintStateInfo(void* state) {

    }
}
