//
// Created by pavlo on 10.05.2021.
//

#include <cassert>
#include "ecbs.h"

ECBS::ECBS(double w, vector<std::string> raw_grid): w(w) {
    size_t n = raw_grid.size();
    size_t m = raw_grid.back().size();
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

std::string showTimedCell(TimedCell c) {
    return std::string("{") + std::to_string(c.coordinates.x) + std::string(", ") +
        std::to_string(c.coordinates.y) + std::string("}@") + std::to_string(c.time);
}

std::string showPath(const Path<Cell>& path) {
    std::string printablePath = "";
    for(auto cell: path){
        printablePath += std::string("{") + std::to_string(cell.coordinates.x) +
                std::string(", ") + std::to_string(cell.coordinates.y) +
                std::string("}@") + std::to_string(cell.time) + std::string(" ");
    }
    return printablePath;
}

class FocalNode {
public:
    TimedCell coordinates;
    double g_value;
    double h_value;
    double f_value;
    TimedCell parent;
    double focal_heuristic;

    bool operator==(const FocalNode &other) const {
        return coordinates == other.coordinates;
    }

    FocalNode(TimedCell coordinates1, double g_value1, double h_value1, TimedCell parent1, double focal_heuristic1) {
        g_value = g_value1;
        coordinates = coordinates1;
        h_value = h_value1;
        f_value = h_value + g_value;
        parent = parent1;
        focal_heuristic = focal_heuristic1;
    }
};

size_t lowLevelFocalHeuristic(size_t agent_id,
                              const std::vector<std::pair<Cell, int>>& time_conflicts,
                              TimedCell node) {
    std::set<size_t> conflict_pairs;
    for (auto [coors, other_agent_id]: time_conflicts) {
        if (coors == node.coordinates) {
            conflict_pairs.insert(other_agent_id);
        }
    }
    return conflict_pairs.size();
}



template<typename Coordinates, typename Compare = std::greater<Node<Coordinates>>>
std::pair<Path<Coordinates>, size_t>
astarF1Min(Graph<Coordinates> *graph, Coordinates start, Coordinates goal,
           double& f1_min, Compare comp = std::greater<Node<Coordinates>>()) {
    size_t expanded_nodes = 0;
    auto open = Open<Coordinates>(comp);
    std::unordered_map<Coordinates, Coordinates> real_parent;
    std::unordered_map<Coordinates, int> dist;
    auto closed = Closed<Coordinates>();
    auto start_node = Node<Coordinates>(start, 0, 0.0, start);
    open.add_node(start_node);
    real_parent[start] = start;
    dist[start] = 0.0;
    while (!open.empty()) {
        auto v = open.get_best_node();
        f1_min = v.f_value;
        if (closed.was_expanded(v.coordinates)) {
            continue;
        }
        closed.add_node(v.coordinates);
        real_parent[v.coordinates] = v.parent;
        dist[v.coordinates] = v.g_value;
        if (graph->is_same_point(v.coordinates, goal)) {
            Path<Coordinates> path;
            auto cur_coors = v.coordinates;
            while (real_parent[cur_coors] != cur_coors) {
                path.push_back(TimedCoordinates<Coordinates>{cur_coors, dist[cur_coors]});
                cur_coors = real_parent[cur_coors];
            }
            path.push_back(TimedCoordinates<Coordinates>{cur_coors, dist[cur_coors]});
            std::reverse(path.begin(), path.end());
            return {path, open.size() + closed.size()};
        }
        for (auto x : graph->get_neighbours(v.coordinates)) {
            if (!closed.was_expanded(x)) {
                auto nodeX = Node<Coordinates>(x, v.g_value + graph->get_cost(v.coordinates, x),
                                               graph->get_h_value(goal, x),
                                               v.coordinates);
                open.add_node(nodeX);
            }
        }
    }
    return {Path<Coordinates>(), 0};
}

bool lowLevelFocalComparator(const FocalNode& a, const FocalNode& b) {
    return std::make_tuple(a.focal_heuristic, a.f_value, a.g_value, a.h_value, a.coordinates.coordinates.x, a.coordinates.coordinates.y, a.coordinates.time) >
            std::make_tuple(b.focal_heuristic, b.f_value, b.g_value, b.h_value, b.coordinates.coordinates.x, b.coordinates.coordinates.y, b.coordinates.time);
}

template <typename T>
std::ostream& operator << ( std::ostream& outs, const FocalNode & p )
{
    return outs << "(FocalNode " << showTimedCell(p.coordinates) << ", f_value=" << p.f_value << ", focal_heuristic" << p.focal_heuristic << ")";
}


std::pair<Path<Cell>, size_t> lowLevelEcbs(Graph<TimedCell> *graph, TimedCell start, TimedCell goal, double w, size_t agent_id, const vector<Path<Cell>>& solution,
                        double& f1_min) {

#ifdef VERBOSE
    bool verbose = true;
#else
    bool verbose = false;
#endif

    // PreCalculate map: time -> (pos, agent)
    // it's need to calculate focal heuristics
    // (we need to now, how many different agens conflict with node (time, pos) in current solution
    // simple algorithm, finding number of conflicts by given time for every node expansion would work O(solution_size^2)
    // We can't afford that!

    // time -> [(cell, agent_id)]
    std::unordered_map<int, std::vector<std::pair<Cell, int>>> time_conflict_map;

    for (size_t i = 0; i < solution.size(); i++) {
        if (i == agent_id)
            continue;
        for (TimedCell coors: solution[i]) {
            if(time_conflict_map.find(coors.time) != time_conflict_map.end()) {
                time_conflict_map[coors.time].push_back({coors.coordinates, i});
            } else {
                time_conflict_map[coors.time] = {{coors.coordinates, i}};
            }
        }
    }

    if (verbose) {
        std::cout << "Low level search" << '\n';
    }

    auto f1 = [](const FocalNode& a, const FocalNode& b)
    {
        return std::make_tuple(a.f_value, a.coordinates.coordinates.x, a.coordinates.coordinates.y, a.coordinates.time) <
        std::make_tuple(b.f_value, b.coordinates.coordinates.x, b.coordinates.coordinates.y, b.coordinates.time);
    };

    auto f2 = lowLevelFocalComparator;

    std::set<FocalNode, decltype(f1)> open(f1);
    // Heap order is inverse!
    auto focal = std::priority_queue<FocalNode, std::vector<FocalNode>, decltype(f2)>(f2);

    auto closed = Closed<TimedCell>();
    auto start_focal_heuristic = lowLevelFocalHeuristic(
            agent_id,
            time_conflict_map.count(0) > 0 ? time_conflict_map[0] : vector<std::pair<Cell, int>>(),
            start
    );
    auto start_node = FocalNode(start, 0, 0.0, start, start_focal_heuristic);

    std::unordered_map<TimedCell, TimedCell> real_parent;
    std::unordered_map<TimedCell, int> dist;
    real_parent[start] = start;
    dist[start] = 0.0;

    size_t expanded_nodes = 0;

    open.insert(start_node);
    focal.push(start_node);

    while(!open.empty()) {
        {
            // Add more nodes to focal
            double old_f1_min = f1_min;
            f1_min = open.begin()->f_value;

            assert(f1_min >= old_f1_min);

            if (f1_min > old_f1_min) {
                auto iter = open.begin();
                auto iter_end = open.end();
                // todo: rewrite rebuilding
                focal = std::priority_queue<FocalNode, std::vector<FocalNode>, decltype(f2)>(f2);
                for(; iter != iter_end; ++ iter) {
                    auto val = iter->f_value;
                    if (val <= f1_min * w) {
                        focal.push(*iter);
                    }
                    if (val > f1_min * w) {
                        break;
                    }
                }
            } else if (focal.empty()) {
                auto iter = open.begin();
                auto iter_end = open.end();
                for(; iter != iter_end; ++ iter) {
                    auto val = iter->f_value;
                    if (val <= f1_min * w) {
                        focal.push(*iter);
                    }
                    if (val > f1_min * w) {
                        break;
                    }
                }
            }
        }

        assert(!focal.empty());

        auto v = focal.top();

        if (verbose) {
            std::cout << "focal size " << focal.size() << " open size " << open.size() << '\n';
            std::cout << showTimedCell(v.coordinates) <<  " opened\n";
        }
        focal.pop();
        open.erase(v);

        if(closed.was_expanded(v.coordinates)) {
            continue;
        }
        closed.add_node(v.coordinates);
        real_parent[v.coordinates] = v.parent;
        dist[v.coordinates] = int(v.g_value); // time is integer
        if (graph->is_same_point(v.coordinates, goal)) {
            Path<Cell> path;

            auto cur_coors = v.coordinates;
            while (real_parent[cur_coors] != cur_coors) {
                path.push_back(TimedCoordinates<Cell>{cur_coors.coordinates, dist[cur_coors]});
                cur_coors = real_parent[cur_coors];
            }
            path.push_back(TimedCoordinates<Cell>{cur_coors.coordinates, dist[cur_coors]});
            std::reverse(path.begin(), path.end());
            return {path, open.size() + closed.size()};
        }
        for (auto x : graph->get_neighbours(v.coordinates)) {
            if (!closed.was_expanded(x)) {
                auto time = int(v.g_value + graph->get_cost(v.coordinates, x)); // time is int


                auto focal_heuristic = lowLevelFocalHeuristic(agent_id,
                                                              time_conflict_map.count(time) > 0
                                                              ? time_conflict_map[time]
                                                              : vector<std::pair<Cell, int>>(),
                                                              x);


                if (verbose)
                    std:: cout << showTimedCell(x) << " added to open\n";
                auto nodeX = FocalNode(x, time,
                                               graph->get_h_value(goal, x),
                                               v.coordinates,
                                               v.focal_heuristic +
                                               focal_heuristic);
                open.insert(nodeX);
            }
        }
    }

    return {Path<Cell>(), 0};
}

bool highLevelFocalComparator(const ECBSHighLevelNode& a, const ECBSHighLevelNode& b) {

    return std::make_tuple(a.focal_heuristic, a.cost.value(), a.LB) > std::make_tuple(b.focal_heuristic, b.cost.value(), b.LB);
}

std::tuple<vector<Path<Cell>>, Statistics> ECBS::find_paths(const vector<std::pair<Cell, Cell>> &tasks) {
    // initialization

#ifdef VERBOSE
    bool verbose = true;
#else
    bool verbose = false;
#endif


    Statistics stats = Statistics(grid);
    size_t actors = tasks.size();
    auto low_graph = AStarGridGraph(grid);
    auto root_node = ECBSHighLevelNode(actors);


    for (size_t i = 0; i < actors; i++) {
        auto[start, goal] = tasks[i];
        double f1_min = 0.0;
        auto [astar_solution, expanded] = astarF1Min(&low_graph, start, goal,
                                                     f1_min);
        root_node.solution[i] = astar_solution;
        stats.low_level_expanded += expanded;
        root_node.LB += f1_min;
        root_node.agent_f1_min[i] = f1_min;
    }

    if(verbose) {
        std::cout << "High Level started\n";
        for (size_t i = 0; i < actors; i++) {
            std::cout << "Init Solution for " << i << ": " << showPath(root_node.solution[i]) << '\n';
        }
    }

    // For focal heuristic we must know number of pairs of conflict agents
    // After expanding single node, only one agent change its solution
    // We can store for each agent number of conflicts and recalculate this number every time


    std::unordered_map<size_t, std::set<size_t>> agent_conflicts;
    int number_of_conflicts = 0;
    for(size_t i = 0; i < actors; i++) {
        agent_conflicts[i] = {};
        std::unordered_set<TimedCell> visits_by_agent;
        for (TimedCell coors: root_node.solution[i]) {
            visits_by_agent.insert(coors);
        }
        for (size_t j = 0; j < actors; j++) {
            if (i != j) {
                for (TimedCell coors: root_node.solution[j]) {
                    if (visits_by_agent.find(coors) != visits_by_agent.end()) {
                        agent_conflicts[i].insert(j);
                        number_of_conflicts += 1;
                        break; // done with agent j, conflict (i, j) detected
                    }
                }
            }
        }
    }

    // Number of conflict pairs
    root_node.focal_heuristic = number_of_conflicts;


    // Comparator to extract FOCAL

    auto f1 = [](const ECBSHighLevelNode& a, const ECBSHighLevelNode& b)
    {
        return a.LB < b.LB;
    };

    // Comparator for focal heuristic

    auto f2 = highLevelFocalComparator;

    root_node.updateCost();
    // todo: open and focal, open is set
    std::set<ECBSHighLevelNode, decltype(f1)> open(f1);
    auto focal = std::priority_queue<ECBSHighLevelNode, vector<ECBSHighLevelNode>, decltype(f2)>(f2);

    double high_f1_min = root_node.LB;

    open.insert(root_node);
    focal.push(root_node);

    while (!open.empty()) {
        // Update focal
        {
            stats.high_level_expanded += 1;
            double old_f1_min = high_f1_min;

            if(!open.begin()->cost.has_value())
                // no solution
                // todo: ?????
                return {vector<Path<Cell>>(), stats};

            high_f1_min = open.begin()->LB;

            //assert(high_f1_min >= old_f1_min);

            if(verbose)
                std::cout << "High f1_min changed: From " << old_f1_min << " to " << high_f1_min << '\n';
            auto iter = open.begin();
            auto iter_end = open.end();
            focal = std::priority_queue<ECBSHighLevelNode, vector<ECBSHighLevelNode>, decltype(f2)>(f2);
            for (; iter != iter_end; ++iter) {
                assert(iter->cost.has_value());
                auto val = iter->cost.value();
                if (val <= high_f1_min * w) {
                    focal.push(*iter);
                }
//                if (val > high_f1_min * w) {
//                    break;
//                }
            }
        }

        assert(!focal.empty());

        if (verbose) {
            std::cout << "High Level open size: " << open.size() << " Focal size: " << focal.size() << '\n';
        }

        auto v = focal.top();
        focal.pop();
        open.erase(v);
        VertexConflict conflict = v.findConflict();
        EdgeConflict  edgeConflict = v.findEdgeConflict();
        if (!conflict.has_value()) {
            if (edgeConflict.empty()) {
                return {v.solution, stats};
            }
            for (auto[actor, edge]: edgeConflict) {
                auto new_node = v;
                new_node.edge_conflicts[actor].insert(edge);

                auto left_low_graph = CBSLowLevelGraph(grid, new_node.vertex_conflicts[actor],
                                                       new_node.edge_conflicts[actor]);
                double f1_min = 0.0;

                auto [new_path, expanded] = lowLevelEcbs(
                        &left_low_graph, TimedCell{tasks[actor].first, 0},
                        TimedCell{tasks[actor].second, 0},
                        w,
                        actor,
                        new_node.solution, f1_min);
                stats.low_level_expanded += expanded;
                new_node.solution[actor] = Path<Cell>();
                new_node.LB -= new_node.agent_f1_min[actor];
                new_node.LB += f1_min;
                new_node.agent_f1_min[actor] = f1_min;

                {
                    int newNodeFocalHeuristic = root_node.focal_heuristic - int(agent_conflicts[actor].size() * 2);

                    for (auto conflict_agent: agent_conflicts[actor]) {
                        agent_conflicts[conflict_agent].erase(actor);
                    }

                    std::unordered_set<TimedCell> visits_by_agent;
                    for (auto coors: new_path) {
                        visits_by_agent.insert(coors);
                    }

                    for (size_t i = 0; i < root_node.solution.size(); i++) {
                        if (i == actor) continue;
                        for (auto cell: root_node.solution[i]) {
                            if (visits_by_agent.find(cell) != visits_by_agent.end()) {
                                newNodeFocalHeuristic += 2;
                                agent_conflicts[actor].insert(i);
                                agent_conflicts[i].insert(actor);
                            }
                        }
                    }
                    new_node.focal_heuristic = newNodeFocalHeuristic;
                }

                for (auto cell: new_path) {
                    new_node.solution[actor].push_back(cell);
                }

                if(verbose) {
                    std::cout << "For actor: " << actor << " new path found: " << showPath(new_node.solution[actor]) << '\n';
                }

                new_node.updateCost();
                if (new_node.cost.has_value()) {
                    open.insert(new_node);
                }
            }
            continue;
        }

        auto[actor1, actor2, timedCell] = conflict.value();
        stats.conflicts_grid[timedCell.coordinates.x][timedCell.coordinates.y]++;
        if (verbose)
            std::cout << "Found conflict: a1:" << actor1 << " a2:" << actor2 << " cell: " << showTimedCell(timedCell) << '\n';

        for (auto actor: {actor1, actor2}) {
            auto new_node = v;
            new_node.vertex_conflicts[actor].insert(timedCell);
            auto left_low_graph = CBSLowLevelGraph(grid, new_node.vertex_conflicts[actor],
                                                   new_node.edge_conflicts[actor]);
            double f1_min = 0.0;

            auto [new_path, expanded] = lowLevelEcbs(
                    &left_low_graph, TimedCell{tasks[actor].first, 0},
                    TimedCell{tasks[actor].second, 0},
                    w,
                    actor,
                    new_node.solution, f1_min);
            stats.low_level_expanded += expanded;
            new_node.solution[actor] = Path<Cell>();
            new_node.LB -= new_node.agent_f1_min[actor];
            new_node.LB += f1_min;
            new_node.agent_f1_min[actor] = f1_min;

            {
                int newNodeFocalHeuristic = root_node.focal_heuristic - int(agent_conflicts[actor].size() * 2);

                for (auto conflict_agent: agent_conflicts[actor]) {
                    agent_conflicts[conflict_agent].erase(actor);
                }

                std::unordered_set<TimedCell> visits_by_agent;
                for (auto coors: new_path) {
                    visits_by_agent.insert(coors);
                }

                for (size_t i = 0; i < root_node.solution.size(); i++) {
                    if (i == actor) continue;
                    for (auto cell: root_node.solution[i]) {
                        if (visits_by_agent.find(cell) != visits_by_agent.end()) {
                            newNodeFocalHeuristic += 2;
                            agent_conflicts[actor].insert(i);
                            agent_conflicts[i].insert(actor);
                        }
                    }
                }
                new_node.focal_heuristic = newNodeFocalHeuristic;
            }

            for (auto cell: new_path) {
                new_node.solution[actor].push_back(cell);
            }

            if(verbose) {
                std::cout << "For actor: " << actor << " new path found: " << showPath(new_node.solution[actor]) << '\n';
            }

            new_node.updateCost();
            if (new_node.cost.has_value()) {
                open.insert(new_node);
            }
        }
    }
    return {vector<Path<Cell>>(), stats};
}


void ECBSHighLevelNode::updateCost() {
    *cost = 0;
    for (auto &p: solution) {
        if (p.empty()) {
            cost = std::nullopt;
        } else if (cost.has_value()) {
            *cost += p.back().time;
        }
    }
}


ECBSHighLevelNode::ECBSHighLevelNode(size_t actors) {
    focal_heuristic = 0;
    LB = 0;
    solution = vector<Path<Cell>>(actors);
    agent_f1_min = vector<double>(actors);
    vertex_conflicts = vector<std::unordered_set<TimedCell>>(actors);
    edge_conflicts = vector<std::unordered_set<TimedEdge>>(actors);
    cost = 0;
}

VertexConflict ECBSHighLevelNode::findConflict() const {
    std::unordered_map<TimedCell, size_t> visits;
    for (size_t i = 0; i < solution.size(); i++) {
        for (TimedCell coors: solution[i]) {
            auto it = visits.find(coors);
            if (it != visits.end()) {
                return std::tuple(i, it->second, it->first);
            } else {
                visits[coors] = i;
            }
        }
    }
    return std::nullopt;
}



EdgeConflict ECBSHighLevelNode::findEdgeConflict() const {
    std::unordered_map<TimedEdge, size_t> passes;
    for (size_t i = 0; i < solution.size(); i++) {
        for (size_t j = 1; j < solution[i].size(); j++) {
            auto prev = solution[i][j - 1];
            auto cur = solution[i][j];
            auto edge = TimedEdge{prev, cur};
            auto it = passes.find(edge);
            if (it != passes.end()) {
                return EdgeConflict({{i,          edge},
                                     {it->second, it->first}});
            } else {
                passes[edge] = i;
            }
        }
    }
    return EdgeConflict();
}
