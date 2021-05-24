//
// Created by olgashimanskaia on 20.05.2021.
//

#ifndef CBS_BCBS_H
#define CBS_BCBS_H

#include "cbs.h"
#include "set"
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
        return id != other.id;
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
    BCBS(vector<vector<int>> grid, double weight);

    template<typename Cmp1, typename Cmp2>
    vector<Path<Cell>> find_paths(const vector<std::pair<Cell, Cell>> &tasks,
                                  std::set<BCBSHighLevelNode, Cmp1> &open,
                                   std::set<BCBSHighLevelNode, Cmp2> &focal,
                                  int &id) {
        while (!focal.empty()) {
            BCBSHighLevelNode min_open = *open.begin();
            if (!min_open.cost.has_value()) {
                // no solution
                return vector<Path<Cell>>();
            }
            BCBSHighLevelNode node = *focal.begin();
            std::cout << 'k' << focal.size() << ' ' << open.size() << std::endl;
            focal.erase(focal.begin());
            std::cout << 'l' << focal.size() << ' ' << open.size() << std::endl;
            open.erase(node);
            std::cout << focal.size() << ' ' << open.size() << std::endl;
            Conflict conflict = node.find_conflict();
            if (!conflict.has_value()) {
                return node.solution;
            }
            double cost_min = min_open.cost.value();
            auto[actor1, actor2, timedCell] = conflict.value();
            for (auto actor: {actor1, actor2}) {
                auto new_node = node;
                new_node.id = id;
                id += 1;
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
                    std::cout << "new node to open" << open.size() << std::endl;
                    if (new_node.cost <= w * cost_min) {
                        focal.insert(new_node);
                        std::cout << "new node to focal" << std::endl;
                    }
                }
            }
            double new_cost_min = open.begin()->cost.value_or(0);
            if (!open.empty() && cost_min < new_cost_min) {
                for (const BCBSHighLevelNode &n: open) {
                    if (n.cost > w * cost_min && n.cost <= w * new_cost_min) {
                        focal.insert(n);
                        std::cout << "node from open to focal" << std::endl;
                    }
                }
            }
        }
        return vector<Path<Cell>>();
    }
};


#endif //CBS_BCBS_H
