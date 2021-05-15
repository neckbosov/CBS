//
// Created by neckbosov on 06.05.2021.
//

#include "cbs.h"
#include <vector>
#include <queue>
#include <unordered_map>

using std::vector;


int AStarGridGraph::get_cost(Cell a, Cell b) {
    return 1;
}

double AStarGridGraph::get_h_value(Cell goal, Cell current_coors) {
    return hypot(
            abs(goal.x - current_coors.x),
            abs(goal.y - current_coors.y)
    );
}

std::vector<Cell> AStarGridGraph::get_neighbours(Cell coors) {
    vector<Cell> directions{{0,  1},
                            {1,  0},
                            {0,  -1},
                            {-1, 0}};
    vector<Cell> res;
    for (Cell mv: directions) {
        Cell cur_cell = coors + mv;
        if (cur_cell.x >= 0 && cur_cell.x < (int) source_cells.size() && cur_cell.y >= 0 &&
            cur_cell.y < (int) source_cells.back().size() && source_cells[cur_cell.x][cur_cell.y] == 0) {
            res.push_back(cur_cell);
        }

    }
    return res;
}


int CBSLowLevelGraph::get_cost(TimedCell a, TimedCell b) {
    return 1;
}

double CBSLowLevelGraph::get_h_value(TimedCell goal, TimedCell current_coors) {
    return hypot(
            abs(goal.coordinates.x - current_coors.coordinates.x),
            abs(goal.coordinates.y - current_coors.coordinates.y),
            abs(goal.time - current_coors.time)
    );
}

vector<TimedCell> CBSLowLevelGraph::get_neighbours(TimedCell coors) {
    vector<Cell> directions{{0,  1},
                            {1,  0},
                            {0,  -1},
                            {-1, 0},
                            {0,  0}};
    vector<TimedCell> res;
    for (Cell mv: directions) {
        Cell cur_cell = coors.coordinates + mv;
        if (cur_cell.x >= 0 && cur_cell.x < (int) source_cells.size() && cur_cell.y >= 0 &&
            cur_cell.y < (int) source_cells.back().size() && source_cells[cur_cell.x][cur_cell.y] == 0) {
            TimedCell cur_times_cell{cur_cell, coors.time + 1};
            if (banned_cells.find(cur_times_cell) == banned_cells.end()) {
                res.push_back(cur_times_cell);
            }
        }

    }
    return res;
}

bool CBSLowLevelGraph::is_same_point(TimedCell a, TimedCell b) {
    return a.coordinates == b.coordinates;
}


CBS::CBS(vector<std::string> raw_grid) {
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

CBSHighLevelNode::CBSHighLevelNode(size_t actors) {
    solution = vector<Path<Cell>>(actors);
    vertex_conflicts = vector<std::unordered_set<TimedCell>>(actors);
    cost = std::nullopt;
}

CBSHighLevelNode::CBSHighLevelNode(const CBSHighLevelNode &other) {
    solution = other.solution;
    vertex_conflicts = other.vertex_conflicts;
    cost = other.cost;
}

void CBSHighLevelNode::update_cost() {
    *cost = 0;
    for (auto &p: solution) {
        if (p.empty()) {
            cost = std::nullopt;
        } else if (cost.has_value()) {
            *cost += p.back().time;
        }
    }
}

Conflict CBSHighLevelNode::find_conflict() const {
    std::unordered_map<TimedCell, size_t> visits;
    for (size_t i = 0; i < solution.size(); i++) {
        for (auto coors: solution[i]) {
            auto it = visits.find(coors);
            if (it != visits.end()) {
                return std::tuple(i, it->second, it->first);
            } else {
                visits[coors] = i;
            }
        }
    }
    return std::nullopt;
}


vector<Path<Cell>> CBS::find_paths(const vector<std::pair<Cell, Cell>> &tasks) {
    size_t actors = tasks.size();
    auto low_graph = AStarGridGraph(grid);
    auto root_node = CBSHighLevelNode(actors);
    for (size_t i = 0; i < actors; i++) {
        auto[start, goal] = tasks[i];
        root_node.solution[i] = astar(&low_graph, start, goal);
    }
    root_node.update_cost();
    std::priority_queue<CBSHighLevelNode, vector<CBSHighLevelNode>, std::greater<>> q;
    q.push(root_node);
    while (!q.empty()) {
        CBSHighLevelNode node = q.top();
        q.pop();
        Conflict conflict = node.find_conflict();
        if (!conflict.has_value()) {
            return node.solution;
        }
        auto[actor1, actor2, timedCell] = conflict.value();
        for (auto actor: {actor1, actor2}) {
            auto new_node = node;
            new_node.vertex_conflicts[actor].insert(timedCell);
            auto left_low_graph = CBSLowLevelGraph(grid, node.vertex_conflicts[actor1]);
            auto new_path = astar(&left_low_graph, TimedCell{tasks[actor].first, 0},
                                  TimedCell{tasks[actor].second, 0});
            new_node.solution[actor] = Path<Cell>();
            for (auto[cell, new_time]: new_path) {
                new_node.solution[actor].push_back(cell);
            }
            new_node.update_cost();
            if (new_node.cost.has_value()) {
                q.push(new_node);
            }
        }
    }
    return vector<Path<Cell>>();
}