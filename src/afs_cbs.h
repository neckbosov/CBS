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
    explicit AFS_CBS(double w, std::vector<std::string> raw_grid);

    vector<Path<Cell>> find_paths(const vector<std::pair<Cell, Cell>> &tasks, long seconds_limit);
};


#endif //CBS_AFS_CBS_H
