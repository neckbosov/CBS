//
// Created by neckbosov on 24.05.2021.
//

#include "astar_ndim.h"
#include <cmath>

double AstarNDimGraph::get_cost(NCoors a, NCoors b) {
    return 1.0;
}

double AstarNDimGraph::get_h_value(NCoors goal, NCoors current_coors) {
    double res = 0.0;
    for (size_t i = 0; i < goal.coors.size(); i++) {
        res += hypot(goal.coors[i].x - current_coors.coors[i].x, goal.coors[i].y - current_coors.coors[i].y);
    }
    return res;
}

void fill_moves(const vector<vector<Cell>> &directions, size_t pos, NCoors cur, vector<NCoors> &buf) {
    if (pos == directions.size()) {
        buf.push_back(cur);
        return;
    }
    for (const auto &direction : directions[pos]) {
        auto new_coors = cur;
        new_coors.coors[pos] = new_coors.coors[pos] + direction;
        fill_moves(directions, pos + 1, new_coors, buf);
    }
}

std::vector<NCoors> AstarNDimGraph::get_neighbours(NCoors coors) {
    vector<Cell> directions{{0,  1},
                            {1,  0},
                            {0,  -1},
                            {-1, 0},
                            {0,  0}};
    vector<vector<Cell>> res;
    for (Cell mv: directions) {
        for (size_t i = 0; i < coors.coors.size(); i++) {
            Cell cur_cell = coors.coors[i] + mv;
            if (cur_cell.x >= 0 && cur_cell.x < (int) source_cells.size() && cur_cell.y >= 0 &&
                cur_cell.y < (int) source_cells.back().size() && source_cells[cur_cell.x][cur_cell.y] == 0) {
                res[i].push_back(cur_cell);
            }
        }
    }
    vector<NCoors> ans;
    fill_moves(res, 0, coors, ans);
    return ans;
}

AstarNDim::AstarNDim(vector<std::string> raw_grid) {
    size_t n = raw_grid.size();
    size_t m = raw_grid.back().size();
    grid = vector<vector<int>>(n, vector<int>(m));
    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < m; j++) {
            if (raw_grid[i][j] == '.') {
                grid[i][j] = 0;
            } else {
                grid[i][j] = 1;
            }
        }
    }
}

vector<Path<Cell>> AstarNDim::find_paths(const vector<std::pair<Cell, Cell>> &tasks) {
    auto astar_ndim = AstarNDimGraph(grid);
    vector<Cell> start_coors(tasks.size()), goal_coors(tasks.size());
    for (size_t i = 0; i < tasks.size(); i++) {
        start_coors[i] = tasks[i].first;
        goal_coors[i] = tasks[i].second;
    }
    NCoors start{start_coors}, goal{goal_coors};
    auto ndim_path = astar(&astar_ndim, start, goal);
    vector<Path<Cell>> res(tasks.size());
    for (auto ndim_node : ndim_path) {
        for (size_t i = 0; i < ndim_node.coordinates.coors.size(); i++) {
            res[i].push_back(TimedCell{ndim_node.coordinates.coors[i], ndim_node.time});
        }
    }
    return res;
}
