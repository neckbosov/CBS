//
// Created by olgashimanskaia on 20.05.2021.
//

#ifndef CBS_BCBS_H
#define CBS_BCBS_H

#include "cbs.h"
#include <set>
#include <ctime>

struct BCBSHighLevelNode {
    int id; // special field for easier comparison
    int num_of_actors; // number of actors
    vector<Path<Cell>> solution; // list of paths for each actor
    vector<boost::unordered_set<TimedCell>> vertex_conflicts; // existing vertex conflicts for node solution
    vector<boost::unordered_set<TimedEdge>> edge_conflicts; // existing edge conflicts for node solution
    std::optional<int> cost; // cost of node solution
    double h; // heuristic: pairs number of conflicting agents

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
    /**
     * Create BCBS(w, 1) instance
     * @param w weight which will be used on the high level
     * @param raw_grid grid where '.' signifies empty cell and '#' - obstacle
     */
    explicit BCBS(vector<std::string> raw_grid, double w);

    /**
     * Create BCBS(w, 1) instance
     * @param weight weight which will be used on the high level
     * @param grid where 0 signifies empty cell and 1 - obstacle
     */
    BCBS(vector<vector<int>> grid, double weight);

    /**
     * Find paths with suboptimal BCBS(w, 1)
     * @tparam Cmp1 comparator for open
     * @tparam Cmp2 comparator for focal
     * @param tasks list of start and goal positions of agents
     * @param open high level open set
     * @param focal high level focal set
     * @param id the first free id which we can assign
     * @param tStart time of start in clocks
     * @param seconds_limit execution limit in seconds
     * @return non-conflict paths for agents (or empty list if not found/timeout)
     */
    template<typename Cmp1, typename Cmp2>
    vector<Path<Cell>> find_paths(const vector<std::pair<Cell, Cell>> &tasks,
                                  std::set<BCBSHighLevelNode, Cmp1> &open,
                                  std::set<BCBSHighLevelNode, Cmp2> &focal,
                                  int &id, clock_t tStart, long seconds_limit) {
        while (!focal.empty() && 1.0L * (clock() - tStart) / CLOCKS_PER_SEC <= 1.0L * seconds_limit) {
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
                    auto left_low_graph = CBSLowLevelGraph(grid, new_node.vertex_conflicts[actor],
                                                           new_node.edge_conflicts[actor]);
                    auto[new_path, expanded] = astar(&left_low_graph, TimedCell{tasks[actor].first, 0},
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
            } else {
                auto[actor1, actor2, timedCell] = conflict.value();
                for (auto actor: {actor1, actor2}) {
                    auto new_node = node;
                    new_node.id = id;
                    id++;
                    new_node.vertex_conflicts[actor].insert(timedCell);
                    auto left_low_graph = CBSLowLevelGraph(grid, new_node.vertex_conflicts[actor],
                                                           new_node.edge_conflicts[actor]);
                    auto[new_path, expanded] = astar(&left_low_graph, TimedCell{tasks[actor].first, 0},
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
            double new_cost_min = open.begin()->cost.value_or(0);
            if (!open.empty() && cost_min < new_cost_min) {
                auto lnode = BCBSHighLevelNode(0, id + 1);
                auto rnode = BCBSHighLevelNode(0, id + 2);
                id += 3;
                lnode.cost = std::optional<int>(w * cost_min);
                rnode.cost = std::optional<int>(w * new_cost_min);
                focal.insert(open.upper_bound(lnode), open.upper_bound(rnode));
            }
        }
        return vector<Path<Cell>>();
    }
};


#endif //CBS_BCBS_H
