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

BCBS::BCBS(vector<vector<int>> grid, double weight): grid(std::move(grid)) {
    w = weight;
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