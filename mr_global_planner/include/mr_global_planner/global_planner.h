#ifndef SRC_GLOBAL_PLANNER_H
#define SRC_GLOBAL_PLANNER_H

#include <vector>
#include <queue>
#include <set>
#include <mr_geometry/geometry.h>

namespace moro {
    struct Cell {
        size_t x;
        size_t y;

        bool operator==(const Cell &o) const {
            return x == o.x && y == o.y;
        }

        bool operator!=(const Cell &o) const {
            return x != o.x || y != o.y;
        }

        bool operator<(const Cell &o)  const {
            return x < o.x || (x == o.x && y < o.y);
        }
    };

    struct WeightedCell : Cell {
        double weight;
    };

    struct WeightedCellComparator {
    public:
        bool operator()(WeightedCell &a, WeightedCell &b) // overloading both operators
        {
            return a.weight > b.weight;
        }
    };

    class GlobalPlanner {
    protected:
        void plan();

        std::map<Cell, int8_t> map; // Map with values -1, 0 and 100. -1: Unknown, 0: Empty, 100: Blocked
        std::map<Cell, Cell> pred; // Map with the predecessor for this cell - used in backtracking.
        uint32_t map_width; // #cells in them map per row
        uint32_t map_height; // #cells in them map per column

        Cell start; // Start
        Cell goal; // Finish

        std::vector<Cell> path; // Ordered list of cells as a result of backtracking from the goal to the start

        std::map<Cell, double> cost; // Estimation of how costly it is to get to the target, through this node.
    private:
        std::priority_queue<WeightedCell, std::vector<WeightedCell>, WeightedCellComparator> open_set; // Cells that need to be explored
        std::set<Cell> closed_set;

        void reset();
        void expandNode(Cell c);
        static double edgeCost(Cell start, Cell finish);
        double heuristic(Cell a);
        void backtrack();
    };
}

#endif //SRC_GLOBAL_PLANNER_H
