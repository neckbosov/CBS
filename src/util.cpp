//
// Created by olgashimanskaia on 24.05.2021.
//

#include <fstream>
#include "vector"
#include "string"
#include "cbs.h"

void print_paths_to_file(const std::vector<Path<Cell>>& paths, const std::string& filename,
                         size_t hl_expanded, size_t ll_expanded) {
    std::ofstream out(filename.c_str());
    out << "@ " << hl_expanded << " " << ll_expanded << std::endl;
    int agent = 1;
    for (const auto&path: paths) {
        out << agent << std::endl;
        for (const auto& cell: path) {
            out << cell.coordinates.x << ' ' << cell.coordinates.y << ' ' << cell.time << std::endl;
        }
        agent++;
    }
    out.close();
}

Statistics::Statistics(const std::vector<vector<int>> &grid) {
    high_level_expanded = 0;
    low_level_expanded = 0;
    conflicts_grid = vector<vector<int>>(grid.size(), vector<int>(grid[0].size(), 0));
    num_of_edge_conflicts = 0;
    num_of_vertex_conflicts = 0;
}
