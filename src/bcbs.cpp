//
// Created by olgashimanskaia on 20.05.2021.
//

#include "bcbs.h"
#include <set>
#include <utility>

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

BCBS::BCBS(vector<vector<int>> grid, double weight) : grid(std::move(grid)) {
    w = weight;
}

BCBSHighLevelNode::BCBSHighLevelNode(size_t actors, int idd) {
    num_of_actors = (int) actors;
    solution = vector<Path<Cell>>(actors);
    vertex_conflicts = vector<boost::unordered_set<TimedCell>>(actors);
    edge_conflicts = vector<boost::unordered_set<TimedEdge>>(actors);
    cost = std::nullopt;
    h = 0;
    id = idd;
}

BCBSHighLevelNode::BCBSHighLevelNode(const BCBSHighLevelNode &other) {
    num_of_actors = other.num_of_actors;
    solution = other.solution;
    vertex_conflicts = other.vertex_conflicts;
    edge_conflicts = other.edge_conflicts;
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
    int number_of_conflicts = 0;
    for (int i = 0; i < num_of_actors; i++) {
        boost::unordered_set<TimedCell> visits_by_agent;
        for (TimedCell coors: solution[i]) {
            visits_by_agent.insert(coors);
        }
        for (int j = 0; j < num_of_actors; j++) {
            if (i != j) {
                for (TimedCell coors: solution[j]) {
                    if (visits_by_agent.find(coors) != visits_by_agent.end()) {
                        number_of_conflicts += 1;
                        break; // done with agent j, conflict (i, j) detected
                    }
                }
            }
        }
    }
    h = number_of_conflicts;
}

VertexConflict BCBSHighLevelNode::find_vertex_conflict() const {
    boost::unordered_map<TimedCell, size_t> visits;
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

EdgeConflict BCBSHighLevelNode::find_edge_conflict() const {
    boost::unordered_map<TimedEdge, size_t> passes;
    for (size_t i = 0; i < solution.size(); i++) {
        for (size_t j = 1; j < solution[i].size(); j++) {
            auto prev = solution[i][j - 1];
            auto cur = solution[i][j];
            auto rprev = cur;
            auto rcur = prev;
            rprev.time = prev.time;
            rcur.time = cur.time;
            auto edge = TimedEdge{rprev, rcur};
            auto it = passes.find(edge);
            if (it != passes.end()) {
                return EdgeConflict({{i,          TimedEdge{prev, cur}},
                                     {it->second, it->first}});
            } else {
                passes[TimedEdge{prev, cur}] = i;
            }
        }
    }
    return EdgeConflict();
}

