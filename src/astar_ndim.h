//
// Created by neckbosov on 24.05.2021.
//

#ifndef CBS_ASTAR_NDIM_H
#define CBS_ASTAR_NDIM_H

#include "graph.h"
#include "astar.h"
#include "cbs.h"
#include <vector>
#include <string>
#include <boost/unordered_map.hpp>
#include <boost/container_hash/hash.hpp>

using std::vector;

struct NCoors {
    vector<Cell> coors;

    bool operator==(const NCoors &other) const {
        return coors == other.coors;
    }

    bool operator!=(const NCoors &other) const {
        return !(*this == other);
    }
};

std::size_t  hash_value(NCoors const &value) ;

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

    std::pair<vector<Path<Cell>>, size_t> find_paths(const vector<std::pair<Cell, Cell>> &tasks);
};

#endif //CBS_ASTAR_NDIM_H
