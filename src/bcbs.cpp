//
// Created by olgashimanskaia on 20.05.2021.
//

#include "bcbs.h"
#include <set>

BCBS::BCBS(vector<std::string> raw_grid, double weight) {
    size_t n = raw_grid.size();
    size_t m = raw_grid.back().size();
    w = weight;
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

BCBSHighLevelNode::BCBSHighLevelNode(size_t actors, int idd) {
    num_of_actors = (int) actors;
    solution = vector<Path<Cell>>(actors);
    vertex_conflicts = vector<std::unordered_set<TimedCell>>(actors);
    cost = std::nullopt;
    h = 0;
    id = idd;
}

BCBSHighLevelNode::BCBSHighLevelNode(const BCBSHighLevelNode &other) {
    num_of_actors = other.num_of_actors;
    solution = other.solution;
    vertex_conflicts = other.vertex_conflicts;
    cost = other.cost;
    h = other.h;
    id = other.id;
}

void BCBSHighLevelNode::update_cost() {
    cost = std::optional<int>(0);
    for (auto &p: solution) {
        if (p.empty()) {
            cost = std::nullopt;
        } else if (cost.has_value()) {
            *cost += p.back().time;
        }
    }
}

void BCBSHighLevelNode::update_h() {
    h = 0;
    for (int i = 0; i < num_of_actors; i++) {
        for (int j = i + 1; j < num_of_actors; j++) {
            bool has_conflict = false;
            for (TimedCell corr1: solution[i]) {
                for (TimedCell corr2: solution[j]) {
                    if (corr1 == corr2) {
                        has_conflict = true;
                        break;
                    }
                }
                if (has_conflict) {
                    h += 1;
                    break;
                }
            }
        }
    }
}

Conflict BCBSHighLevelNode::find_conflict() const {
    std::unordered_map<TimedCell, size_t> visits;
    for (size_t i = 0; i < solution.size(); i++) {
        for (TimedCell coors: solution[i]) {
            auto it = visits.find(coors);
            if (it != visits.end()) {
                return Conflict(std::tuple(i, it->second, it->first));
            } else {
                visits[coors] = i;
            }
        }
    }
    return std::nullopt;
}

//template<typename T, typename Comp>
//using BCBSHeap = __gnu_pbds::priority_queue<T, Comp>;

vector<Path<Cell>> BCBS::find_paths(const vector<std::pair<Cell, Cell>> &tasks) {
    size_t actors = tasks.size();
    auto low_graph = AStarGridGraph(grid);
    int id = 0;
    auto root_node = BCBSHighLevelNode(actors, id);
    id += 1;
    for (size_t i = 0; i < actors; i++) {
        auto[start, goal] = tasks[i];
        root_node.solution[i] = astar(&low_graph, start, goal);
    }
    root_node.update_cost();
    root_node.update_h();

    auto g = [](const BCBSHighLevelNode &a, const BCBSHighLevelNode &b) {
        return a.cost < b.cost;
    };

    auto h_c = [](const BCBSHighLevelNode &a, const BCBSHighLevelNode &b) {
        return a.h > b.h;
    };

    std::set<BCBSHighLevelNode, decltype(g)> open(g);
//    std::unordered_map<int, bool> in_open = {};
    auto focal = std::priority_queue<BCBSHighLevelNode, vector<BCBSHighLevelNode>, decltype(h_c)>(h_c);
    open.insert(root_node);
    focal.push(root_node);

    while (!focal.empty()) {
//        std::make_heap(open.begin(), open.end(), g);
        BCBSHighLevelNode min_open = *open.begin();
//        while (in_open[min_open.id]) {
//            std::pop_heap(open.begin(), open.end(), g);
//            open.pop_back();
//            min_open = open.front();
//        }
        if (!min_open.cost.has_value()) {
            // no solution
            return vector<Path<Cell>>();
        }
        BCBSHighLevelNode node = focal.top();
        focal.pop();
        open.erase(node);
        Conflict conflict = node.find_conflict();
        if (!conflict.has_value()) {
            return node.solution;
        }
        double cost_min = min_open.cost.value();
        auto[actor1, actor2, timedCell] = conflict.value();
        for (auto actor: {actor1, actor2}) {
            auto new_node = node;
            new_node.vertex_conflicts[actor].insert(timedCell);
            auto left_low_graph = CBSLowLevelGraph(grid, node.vertex_conflicts[actor]);
            auto new_path = astar(&left_low_graph, TimedCell{tasks[actor].first, 0},
                                  TimedCell{tasks[actor].second, 0});
            new_node.solution[actor] = Path<Cell>();
            for (auto[cell, new_time]: new_path) {
                new_node.solution[actor].push_back(TimedCell{cell.coordinates, new_time});
            }
            new_node.update_cost();
            new_node.update_h();
            if (new_node.cost.has_value()) {
                open.insert(new_node);
                if (new_node.cost < w * cost_min) {
                    focal.push(new_node);
                }
            }
        }

//        std::make_heap(open.begin(), open.end(), g);
//        while (in_open[open.front().id]) {
//            std::pop_heap(open.begin(), open.end(), g);
//            open.pop_back();
//        }
        double new_cost_min = open.begin()->cost.value_or(0);
        if (!open.empty() && cost_min < open.begin()->cost.value_or(0)) {
            for (const BCBSHighLevelNode &n: open) {
                if (n.cost > w * cost_min && n.cost <= w * new_cost_min) {
                    focal.push(n);
                }
            }
        }
    }
    return vector<Path<Cell>>();
}