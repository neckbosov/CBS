//
// Created by olgashimanskaia on 24.05.2021.
//

#ifndef CBS_UTIL_H
#define CBS_UTIL_H

#include "vector"
#include "string"

void print_paths_to_file(const std::vector<Path<Cell>>& paths, const std::string& filename, size_t hl_expanded,
                         size_t ll_expanded);

// usage in .py benchmarks
void print_weight_time_map_to_file(const std::vector<std::pair<double, double>>& weights_time_map, const std::string& filename);

#endif //CBS_UTIL_H
