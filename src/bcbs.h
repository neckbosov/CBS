//
// Created by olgashimanskaia on 20.05.2021.
//

#ifndef CBS_BCBS_H
#define CBS_BCBS_H

#include "cbs.h"

using Conflict = std::optional<std::tuple<size_t, size_t, TimedCell>>;

struct BCBSHighLevelNode {
    int id;
    int num_of_actors;
    vector<Path<Cell>> solution;
    vector<std::unordered_set<TimedCell>> vertex_conflicts;
    std::optional<int> cost;
    double h;

    bool operator>(const CBSHighLevelNode &other) const {
        if (!cost.has_value()) {
            return false;
        } else if (!other.cost.has_value()) {
            return true;
        } else {
            return cost.value() < other.cost.value();
        }
    }

    bool operator==(const BCBSHighLevelNode &other) const {
        return id == other.id;
    }

    bool operator!=(const BCBSHighLevelNode &other) const {
        return !(*this == other);
    }

    explicit BCBSHighLevelNode(size_t actors, int id);

    BCBSHighLevelNode(const BCBSHighLevelNode &other);

    void update_cost();

    void update_h();

    Conflict find_conflict() const;

};

class BCBS {
private:
    vector<vector<int>> grid;
    double w;
public:
    explicit BCBS(vector<std::string> raw_grid, double w);

    vector<Path<Cell>> find_paths(const vector<std::pair<Cell, Cell>> &tasks);
};


#endif //CBS_BCBS_H
