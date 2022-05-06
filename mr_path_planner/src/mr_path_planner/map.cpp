#include "mr_path_planner/map.h"
#include <math.h>

namespace moro {

    Map::Map(nav_msgs::OccupancyGrid map_data, MapOptions options): micropather::Graph() {
        std::vector<uint8_t> pixel_brightness;

        std::transform(map_data.data.begin(), map_data.data.end(), std::back_inserter(pixel_brightness), [](int8_t pixel) {
            if(pixel == -1) {
                return (uint8_t) 0; // color unknown pixels black
            }

            // for the pixels with a known value, turn it into a brightness value (0=black=occupied, 255=white=free)
            return (uint8_t) (255 - ((uint8_t) (pixel / 100.0 * 255.0)));
        });

        cv::Mat source_image (cv::Size(map_data.info.width, map_data.info.height), CV_8UC1, pixel_brightness.data());

        cv::Mat scaled_image;
        if(options.scale_factor != 1) {
            int scale_width = source_image.size().width / options.scale_factor;
            int scale_height = source_image.size().height / options.scale_factor;

            cv::resize(source_image, scaled_image, cv::Size(scale_width, scale_height), cv::INTER_LINEAR);
        } else {
            scaled_image = source_image;
        }

        cv::Mat blurred_image = scaled_image;
        if(options.blur_iterations > 0) {
            for(int b = 0; b < options.blur_iterations; b++) {
                cv::Mat blurred_image_intermediary;
                cv::GaussianBlur(blurred_image, blurred_image_intermediary, cv::Size(11,11), 0);
                blurred_image = blurred_image_intermediary;
            }
        }

        cv::imwrite("blub.jpg", blurred_image);
        map_ = blurred_image.clone();
        width_ = map_data.info.width / options.scale_factor;
        height_ = map_data.info.height / options.scale_factor;
        origin_x_ = map_data.info.origin.position.x;
        origin_y_ = map_data.info.origin.position.y;
        resolution_ = map_data.info.resolution * options.scale_factor;
        occupancy_theshold_ = options.occupancy_threshold;
        allow_diagonal_movement_ = options.allow_diagonal_movement;
    }

    float Map::LeastCostEstimate( void* stateStart, void* stateEnd ) {
        auto start = nodeToXY(stateStart);
        auto end = nodeToXY(stateEnd);

        return sqrt(pow(start.first - end.first, 2) + pow(start.second - end.second, 2));
    }

    void Map::AdjacentCost( void* state, MP_VECTOR< micropather::StateCost > *adjacent ) {
        auto node = nodeToXY(state);


        std::vector<std::pair<int, int>> neighbors;
        if(allow_diagonal_movement_) {
            neighbors = {{node.first+1, node.second}, {node.first-1, node.second}, {node.first, node.second+1}, {node.first, node.second-1},
                         {node.first+1, node.second+1}, {node.first-1, node.second-1}, {node.first+1, node.second-1}, {node.first-1, node.second+1}};
        } else {
            neighbors = {{node.first+1, node.second}, {node.first-1, node.second}, {node.first, node.second+1}, {node.first, node.second-1}};
        }
        for (int i = 0; i < neighbors.size(); i++) {
            if(neighbors[i].first >= 0 && neighbors[i].first < width_ && 
               neighbors[i].second >= 0 && neighbors[i].second < height_ && 
               get(neighbors[i].first, neighbors[i].second) > occupancy_theshold_) {
                   micropather::StateCost nodeCost = { xyToNode( neighbors[i].first, neighbors[i].second ), 1.0 };
                   adjacent->push_back(nodeCost);
            }
        }
    }

    void Map::PrintStateInfo(void* state) {

    }
}
