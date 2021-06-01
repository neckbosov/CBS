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
    vector<boost::unordered_set<TimedCell>> vertex_conflicts;
    vector<boost::unordered_set<TimedEdge>> edge_conflicts;
    vector<boost::unordered_set<size_t >> agent_conflicts;

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
    /**
     * Create ECBS instance
     * @param w initial weight which will be used on the both levels
     * @param raw_grid grid where '.' signifies empty cell and '#' - obstacle
     */
    explicit ECBS(double w, vector<std::string> raw_grid);

    /**
     * Find paths with suboptimal ECBS
     * @param tasks
     * @return non-conflict paths for agents (or empty list if not found/timeout)
     */
    std::tuple<vector<Path<Cell>>, size_t, size_t> find_paths(const vector<std::pair<Cell, Cell>> &tasks);

};


#endif //COURSE_PROJECT_ECBS_H
