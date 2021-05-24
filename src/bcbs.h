//
// Created by olgashimanskaia on 20.05.2021.
//

#ifndef CBS_BCBS_H
#define CBS_BCBS_H

#include "cbs.h"
#include <set>

struct BCBSHighLevelNode {
    int id;
    int num_of_actors;
    vector<Path<Cell>> solution;
    vector<std::unordered_set<TimedCell>> vertex_conflicts;
    vector<std::unordered_set<TimedEdge>> edge_conflicts;
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
        return id != other.id;
    }

    explicit BCBSHighLevelNode(size_t actors, int id);

    BCBSHighLevelNode(const BCBSHighLevelNode &other);

    void update_cost();

    void update_h();

    VertexConflict find_vertex_conflict() const;

    EdgeConflict find_edge_conflict() const;
};

class BCBS {
private:
    vector<vector<int>> grid;
    double w;
public:
    explicit BCBS(vector<std::string> raw_grid, double w);

    BCBS(vector<vector<int>> grid, double weight);

    template<typename Cmp1, typename Cmp2>
    vector<Path<Cell>> find_paths(const vector<std::pair<Cell, Cell>> &tasks,
                                  std::set<BCBSHighLevelNode, Cmp1> &open,
                                  std::set<BCBSHighLevelNode, Cmp2> &focal,
                                  int &id, clock_t tStart, long seconds_limit) {
        while (!focal.empty() && (clock() - tStart) / CLOCKS_PER_SEC <= seconds_limit) {
            BCBSHighLevelNode min_open = *open.begin();
            if (!min_open.cost.has_value()) {
                // no solution
                return vector<Path<Cell>>();
            }
            BCBSHighLevelNode node = *focal.begin();
            focal.erase(focal.begin());
            open.erase(node);
            VertexConflict conflict = node.find_vertex_conflict();
            double cost_min = min_open.cost.value();
            if (!conflict.has_value()) {
                auto edge_conflict = node.find_edge_conflict();
                if (edge_conflict.empty()) {
                    return node.solution;
                }
                for (auto[actor, edge]: edge_conflict) {
                    auto new_node = node;
                    new_node.id = id;
                    id++;
                    new_node.edge_conflicts[actor].insert(edge);
                    auto left_low_graph = CBSLowLevelGraph(grid, new_node.vertex_conflicts[actor], new_node.edge_conflicts[actor]);
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
                        if (new_node.cost <= w * cost_min) {
                            focal.insert(new_node);
                        }
                    }
                }
            }
            auto[actor1, actor2, timedCell] = conflict.value();
            for (auto actor: {actor1, actor2}) {
                auto new_node = node;
                new_node.id = id;
                id++;
                new_node.vertex_conflicts[actor].insert(timedCell);
                auto left_low_graph = CBSLowLevelGraph(grid, new_node.vertex_conflicts[actor], new_node.edge_conflicts[actor]);
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
                    if (new_node.cost <= w * cost_min) {
                        focal.insert(new_node);
                    }
                }
            }
            double new_cost_min = open.begin()->cost.value_or(0);
            if (!open.empty() && cost_min < new_cost_min) {
                for (const BCBSHighLevelNode &n: open) {
                    if (n.cost > w * cost_min && n.cost <= w * new_cost_min) {
                        focal.insert(n);
                    }
                }
            }
        }
        return vector<Path<Cell>>();
    }
};


#endif //CBS_BCBS_H
