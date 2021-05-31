//
// Created by olgashimanskaia on 20.05.2021.
//

#ifndef CBS_BCBS_H
#define CBS_BCBS_H

#include "cbs.h"
#include <set>
#include <ctime>

struct BCBSHighLevelNode {
    int id;
    int num_of_actors;
    vector<Path<Cell>> solution;
    vector<boost::unordered_set<TimedCell>> vertex_conflicts;
    vector<boost::unordered_set<TimedEdge>> edge_conflicts;
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
        while (!focal.empty() && 1.0L * (clock() - tStart) / CLOCKS_PER_SEC <= 1.0L * seconds_limit) {
//            std::cout << focal.size() << ' ' << open.size() << std::endl;
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
//            std::cout << cost_min << std::endl;
            if (!conflict.has_value()) {
//                std::cout << "no vertex conflict" << std::endl;
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
                continue;
            }
            auto[actor1, actor2, timedCell] = conflict.value();
//            std::cout << "vertex conflict " << actor1 << ' ' << actor2 << std::endl;
            for (auto actor: {actor1, actor2}) {
//                std::cout << "current actor " << actor << std::endl;
                auto new_node = node;
                new_node.id = id;
                id++;
                new_node.vertex_conflicts[actor].insert(timedCell);
                auto left_low_graph = CBSLowLevelGraph(grid, new_node.vertex_conflicts[actor],
                                                       new_node.edge_conflicts[actor]);
                auto[new_path, expanded] = astar(&left_low_graph, TimedCell{tasks[actor].first, 0},
                                                 TimedCell{tasks[actor].second, 0});
//                std::cout << "astar ok " << std::endl;
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
                auto lnode = BCBSHighLevelNode(0, id + 1);
                auto rnode = BCBSHighLevelNode(0, id + 2);
                lnode.cost = std::optional<int>(w * cost_min);
                rnode.cost = std::optional<int>(w * new_cost_min);
                focal.insert(open.upper_bound(lnode), open.upper_bound(rnode));
            }
        }
        return vector<Path<Cell>>();
    }
};


#endif //CBS_BCBS_H
