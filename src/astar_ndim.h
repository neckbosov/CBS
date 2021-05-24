//
// Created by neckbosov on 24.05.2021.
//

#ifndef CBS_ASTAR_NDIM_H
#define CBS_ASTAR_NDIM_H

#include "graph.h"
#include "astar.h"
#include "cbs.h"
#include <vector>
#include <unordered_map>
#include <string>

using std::vector;
using std::unordered_map;

struct NCoors {
    vector<Cell> coors;

    bool operator==(const NCoors &other) const {
        return coors == other.coors;
    }

    bool operator!=(const NCoors &other) const {
        return !(*this == other);
    }
};

namespace std {
    template<>
    struct hash<NCoors> {
        size_t operator()(const NCoors &coors) const {
            std::size_t seed = coors.coors.size();
            auto cell_hasher = hash<Cell>();
            for (auto &i : coors.coors) {
                seed ^= cell_hasher(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };
}

class AstarNDimGraph : public Graph<NCoors> {
public:
    vector<vector<int>> &source_cells;

    double get_cost(NCoors a, NCoors b) override;

    double get_h_value(NCoors goal, NCoors current_coors) override;

    std::vector<NCoors> get_neighbours(NCoors coors) override;

    explicit AstarNDimGraph(vector<vector<int>> &cells) : source_cells(cells) {};
};

class AstarNDim {

private:
    vector<vector<int>> grid;
public:
    explicit AstarNDim(vector<std::string> raw_grid);
    vector<Path<Cell>> find_paths(const vector<std::pair<Cell, Cell>> &tasks);
};

#endif //CBS_ASTAR_NDIM_H
