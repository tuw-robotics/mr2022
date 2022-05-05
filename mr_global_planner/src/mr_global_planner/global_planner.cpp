#include "mr_global_planner/global_planner.h"

#include <iostream>

using namespace moro;

// Reset the internal state, such that the A* algorithm can get started.
void GlobalPlanner::reset() {
    this->cost.clear();
    this->pred.clear();
    this->path.clear();
    this->closed_set.clear();
    this->open_set = {};

    // Initialize open-set with the start;
    WeightedCell startCell{};
    startCell.x = this->start.x;
    startCell.y = this->start.y;
    startCell.weight = this->heuristic(startCell);

    this->cost.insert({startCell, 0});
    this->open_set.push(startCell);
}

double GlobalPlanner::edgeCost(Cell a, Cell b) {
    size_t x = a.x > b.x ? a.x - b.x : b.x - a.x;
    size_t y = a.y > b.y ? a.y - b.y : b.y - a.y;

    double cost = sqrt(x * x + y * y);

    return cost;
}

double GlobalPlanner::heuristic(Cell a) {
    return this->edgeCost(a, this->goal);
}

void GlobalPlanner::expandNode(Cell c) {
    for (size_t x = fmax(c.x - 1, 0); x <= fmin(c.x + 1, this->map_width - 1); x++) {
        for (size_t y = fmax(c.y - 1, 0); y <= fmin(c.y + 1, this->map_height - 1); y++) {
            Cell successor = {x, y};

            if (this->closed_set.find(successor) != this->closed_set.end()) { // Only examine non-closed nodes.
                continue;
            }

            if (this->map.at(successor) > 0) { // Close walls.
                this->closed_set.insert(successor);
                continue;
            }

            double tentative_g = this->cost.at(c) + this->edgeCost(c, successor);

            if (this->cost.find(successor) == this->cost.end() ||
                (this->cost.find(successor) != this->cost.end() && tentative_g < this->cost.at(successor)))
            {
                if (this->pred.find(successor) != this->pred.end()) {
                    this->pred.at(successor) = c;
                } else {
                    this->pred.insert({successor, c});
                }

                if (this->cost.find(successor) != this->cost.end()) {
                    this->cost.at(successor) = tentative_g;
                } else {
                    this->cost.insert({successor, tentative_g});
                }

                WeightedCell weightedSuccessor = {};
                weightedSuccessor.x = successor.x;
                weightedSuccessor.y = successor.y;
                weightedSuccessor.weight = tentative_g + this->heuristic(successor);
                this->open_set.push(weightedSuccessor);
            }
        }
    }
}

void GlobalPlanner::plan() {
    this->reset();

    while (!this->open_set.empty()) {
        WeightedCell current = this->open_set.top();
        this->open_set.pop();

        if (current.x == goal.x && current.y == goal.y) {
            break; // TODO: FOUND!
        }

        this->closed_set.insert(current);
        this->expandNode(current);
    }

    this->backtrack();
}

void GlobalPlanner::backtrack() {
    Cell current = this->goal;

    this->path.push_back(current);

    while (current != this->start) {
        current = this->pred.at(current);
        this->path.push_back(current);
    }

//    std::reverse(this->pred.begin(), this->pred.end());
}
