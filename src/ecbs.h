//
// Created by pavlo on 10.05.2021.
//

#ifndef COURSE_PROJECT_ECBS_H
#define COURSE_PROJECT_ECBS_H

#include <set>
#include "cbs.h"


struct ECBSHighLevelNode {
    vector<Path<Cell>> solution;
    vector<double> agent_f1_min;
    vector<std::unordered_set<TimedCell>> vertex_conflicts;
    vector<std::unordered_set<TimedEdge>> edge_conflicts;

    std::optional<int> cost;
    double LB;
    int focal_heuristic;


    explicit ECBSHighLevelNode(size_t actors);

    void updateCost();

    EdgeConflict findEdgeConflict() const;

    VertexConflict findConflict() const;
};


class ECBS {
private:
    double w;
    vector<vector<int>> grid;
public:
    explicit ECBS(double w, vector<std::string> raw_grid);

    std::tuple<vector<Path<Cell>>, Statistics> find_paths(const vector<std::pair<Cell, Cell>> &tasks);

};


#endif //COURSE_PROJECT_ECBS_H
