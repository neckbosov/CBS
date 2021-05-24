//
// Created by neckbosov on 06.05.2021.
//

#include "cbs.h"
#include <vector>
#include <queue>
#include <unordered_map>

using std::vector;


double AStarGridGraph::get_cost(Cell a, Cell b) {
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


double CBSLowLevelGraph::get_cost(TimedCell a, TimedCell b) {
    return 1;
}

double CBSLowLevelGraph::get_h_value(TimedCell goal, TimedCell current_coors) {
    return hypot(
            abs(goal.coordinates.x - current_coors.coordinates.x),
            abs(goal.coordinates.y - current_coors.coordinates.y)
//            abs(goal.time - current_coors.time)
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
            auto timed_edge = TimedEdge{coors, cur_times_cell};
            if (banned_cells.find(cur_times_cell) == banned_cells.end() &&
                banned_edges.find(timed_edge) == banned_edges.end()) {
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
    edge_conflicts = vector<std::unordered_set<TimedEdge>>(actors);
    cost = std::nullopt;
}

CBSHighLevelNode::CBSHighLevelNode(const CBSHighLevelNode &other) {
    solution = other.solution;
    vertex_conflicts = other.vertex_conflicts;
    edge_conflicts = other.edge_conflicts;
    cost = other.cost;
}

void CBSHighLevelNode::update_cost() {
    cost = std::optional<int>(0);
//    std::cout << cost.has_value() << std::endl;
    for (auto &p: solution) {
//        std::cout << "Path size: " << p.size() << std::endl;
        if (p.empty()) {
            cost = std::nullopt;
        } else if (cost.has_value()) {
            *cost += p.back().time;
        }
    }
}

VertexConflict CBSHighLevelNode::find_vertex_conflict() const {
    std::unordered_map<TimedCell, size_t> visits;
    for (size_t i = 0; i < solution.size(); i++) {
        for (TimedCell coors: solution[i]) {
            auto it = visits.find(coors);
            if (it != visits.end()) {
                return VertexConflict(std::tuple(i, it->second, it->first));
            } else {
                visits[coors] = i;
            }
        }
    }
    return std::nullopt;
}

EdgeConflict CBSHighLevelNode::find_edge_conflict() const {
    std::unordered_map<TimedEdge, size_t> passes;
    for (size_t i = 0; i < solution.size(); i++) {
        for (size_t j = 1; j < solution[i].size(); j++) {
            auto prev = solution[i][j - 1];
            auto cur = solution[i][j];
            auto edge = TimedEdge{prev, cur};
            auto it = passes.find(edge);
            if (it != passes.end()) {
                return EdgeConflict({{i,          edge},
                                     {it->second, it->first}});
            } else {
                passes[edge] = i;
            }
        }
    }
    return EdgeConflict();
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
        VertexConflict conflict = node.find_vertex_conflict();
        if (!conflict.has_value()) {
            auto edge_conflict = node.find_edge_conflict();
            if (edge_conflict.empty()) {
                return node.solution;
            }
            for (auto[actor, edge]: edge_conflict) {
                auto new_node = node;
                new_node.edge_conflicts[actor].insert(edge);
                auto left_low_graph = CBSLowLevelGraph(grid, new_node.vertex_conflicts[actor], new_node.edge_conflicts[actor]);
                auto new_path = astar(&left_low_graph, TimedCell{tasks[actor].first, 0},
                                      TimedCell{tasks[actor].second, 0});
//            std::cout << "new path for actor " << actor << std::endl;
//            for (auto v : new_path) {
//                std::cout << "Time: " << v.time << " x: " << v.coordinates.coordinates.x << " y: "
//                          << v.coordinates.coordinates.y << std::endl;
//            }
                new_node.solution[actor] = Path<Cell>();
                for (auto[cell, new_time]: new_path) {
                    new_node.solution[actor].push_back(TimedCell{cell.coordinates, new_time});
                }
                new_node.update_cost();
                if (new_node.cost.has_value()) {
//                std::cout << "update queue" << std::endl;
                    q.push(new_node);
                }
            }
            continue;
        }
//        std::cout << node.cost.value();
        auto[actor1, actor2, timedCell] = conflict.value();
//        std::cout << "path 1:" << std::endl;
//        for (auto v : node.solution[actor1]) {
//            std::cout << "Time: " << v.time << " x: " << v.coordinates.x << " y: " << v.coordinates.y << std::endl;
//        }
//        std::cout << "path 2:" << std::endl;
//        for (auto v : node.solution[actor2]) {
//            std::cout << "Time: " << v.time << " x: " << v.coordinates.x << " y: " << v.coordinates.y << std::endl;
//        }
//        std::cout << "VertexConflict: time " << timedCell.time << " x " << timedCell.coordinates.x << " y "
//                  << timedCell.coordinates.y << std::endl;
        for (auto actor: {actor1, actor2}) {
            auto new_node = node;
            new_node.vertex_conflicts[actor].insert(timedCell);
            auto left_low_graph = CBSLowLevelGraph(grid, new_node.vertex_conflicts[actor], new_node.edge_conflicts[actor]);
            auto new_path = astar(&left_low_graph, TimedCell{tasks[actor].first, 0},
                                  TimedCell{tasks[actor].second, 0});
//            std::cout << "new path for actor " << actor << std::endl;
//            for (auto v : new_path) {
//                std::cout << "Time: " << v.time << " x: " << v.coordinates.coordinates.x << " y: "
//                          << v.coordinates.coordinates.y << std::endl;
//            }
            new_node.solution[actor] = Path<Cell>();
            for (auto[cell, new_time]: new_path) {
                new_node.solution[actor].push_back(TimedCell{cell.coordinates, new_time});
            }
            new_node.update_cost();
            if (new_node.cost.has_value()) {
//                std::cout << "update queue" << std::endl;
                q.push(new_node);
            }
        }
    }
    return vector<Path<Cell>>();
}