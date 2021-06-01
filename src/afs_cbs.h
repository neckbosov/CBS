//
// Created by olgashimanskaia on 22.05.2021.
//

#ifndef CBS_AFS_CBS_H
#define CBS_AFS_CBS_H


#include <string>
#include "cbs.h"

class AFS_CBS {
private:
    double w1;
    std::vector<std::vector<int>> grid;
public:
    /**
     * Create AFS_CBS instance
     * @param w initial weight which will be used on the first iteration
     * @param raw_grid grid where '.' signifies empty cell and '#' - obstacle
     */
    explicit AFS_CBS(double w, std::vector<std::string> raw_grid);

    /**
     * Find paths using AFS.
     * @param tasks vector of agents' start and goal positions
     * @param seconds_limit time limit in seconds
     * @return non-conflict paths for agents (or empty list if not found/timeout) and vector of (time, weight) pairs for stats
     */
    std::tuple<vector<Path<Cell>>, std::vector<std::pair<double, double>>>
    find_paths(const vector<std::pair<Cell, Cell>> &tasks, long seconds_limit);
};


#endif //CBS_AFS_CBS_H
