//
// Created by neckbosov on 04.05.2021.
//

#ifndef COURSE_PROJECT_ASTAR_H
#define COURSE_PROJECT_ASTAR_H

#include <queue>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <iostream>
#include "graph.h"

template<typename Coordinates, typename Compare=std::greater<Node<Coordinates>>>
class Open {
private:
    std::priority_queue<Node<Coordinates>, std::vector<Node<Coordinates>>, Compare> elements;
public:
    bool empty() {
        return elements.empty();
    }

    Node<Coordinates> get_best_node() {
        auto res = elements.top();
        elements.pop();
        return res;
    }

    void add_node(Node<Coordinates> node) {
        elements.push(node);
    }

    Open() {
        elements = std::priority_queue<Node<Coordinates>, std::vector<Node<Coordinates>>, Compare>(
                std::greater<Node<Coordinates>>());
    }

    explicit Open(Compare comp) {
        elements = std::priority_queue<Node<Coordinates>, std::vector<Node<Coordinates>>, Compare>(comp);
    }
};

template<typename Coordinates>
class Closed {
private:
    std::unordered_set<Coordinates> elements;
public:
    void add_node(Coordinates coors) {
        elements.insert(coors);
    }

    bool was_expanded(Coordinates coors) {
        return elements.find(coors) != elements.end();
    }
};

template<typename Coordinates>
struct TimedCoordinates {
    Coordinates coordinates;
    int time;

    bool operator==(const TimedCoordinates<Coordinates> &other) const {
        return coordinates == other.coordinates && time == other.time;
    }

    bool operator!=(const TimedCoordinates<Coordinates> &other) const {
        return !(*this == other);
    }
};

namespace std {
    template<typename Coordinates>
    struct hash<TimedCoordinates<Coordinates>> {
        size_t operator()(const TimedCoordinates<Coordinates> &timedCoordinates) const {
            auto int_hasher = hash<int>();
            auto coors_hasher = hash<Coordinates>();
            return (int_hasher(timedCoordinates.time) << 1) ^ coors_hasher(timedCoordinates.coordinates);
        }
    };
}

template<typename Coordinates>
using Path = std::vector<TimedCoordinates<Coordinates>>;

template<typename Coordinates, typename Compare = std::greater<Node<Coordinates>>>
Path<Coordinates>
astar(Graph<Coordinates> *graph, Coordinates start, Coordinates goal,
      Compare comp = std::greater<Node<Coordinates>>()) {
    auto open = Open<Coordinates>(comp);
    std::unordered_map<Coordinates, Coordinates> real_parent;
    std::unordered_map<Coordinates, double> dist;
    auto closed = Closed<Coordinates>();
    auto start_node = Node<Coordinates>(start, 0, 0.0, start);
    open.add_node(start_node);
    real_parent[start] = start;
    dist[start] = 0.0;
    while (!open.empty()) {
        auto v = open.get_best_node();
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
            return path;
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
    return Path<Coordinates>();
}

template<typename Coordinates, typename Compare = std::greater<Node<Coordinates>>>
double count_path_len(Graph<Coordinates> *graph, Path<Coordinates> path) {
    if (path.empty()) {
        return -1.0;
    }
    Coordinates prev = path[0].coordinates;
    double len = 0.0;
    for (auto i = 1; i < path.size(); i++) {
        len += graph->get_cost(prev, path[i].coordinates);
        prev = path[i].coordinates;
    }
    return len;
}

#endif //COURSE_PROJECT_ASTAR_H
