//
// Created by olgashimanskaia on 24.05.2021.
//

#include <fstream>
#include "vector"
#include "string"
#include "cbs.h"
#include "util.h"

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