//
// Created by olgashimanskaia on 24.05.2021.
//

#ifndef CBS_UTIL_H
#define CBS_UTIL_H

#include "vector"
#include "string"
#include "cbs.h"

void print_paths_to_file(const std::vector<Path<Cell>>& paths, const std::string& filename, size_t hl_expanded,
                         size_t ll_expanded);

struct Statistics {

    explicit Statistics(const std::vector<std::vector<int>> &grid);

    size_t high_level_expanded;
    size_t low_level_expanded;
    std::vector<std::vector<int>> conflicts_grid;
    size_t num_of_vertex_conflicts;
    size_t num_of_edge_conflicts;
};
#endif //CBS_UTIL_H
