//
// Created by neckbosov on 04.05.2021.
//

#ifndef COURSE_PROJECT_ASTAR_H
#define COURSE_PROJECT_ASTAR_H

#include <queue>
#include <vector>
#include <algorithm>
#include <iostream>
#include "graph.h"
#include <boost/heap/d_ary_heap.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <boost/container_hash/hash.hpp>

template<typename Coordinates, typename Compare=std::greater<Node<Coordinates>>>
using astar_heap = boost::heap::d_ary_heap<Node<Coordinates>, boost::heap::arity<2>, boost::heap::compare<Compare>>;

template<typename Coordinates, typename Compare=std::greater<Node<Coordinates>>>
class Open {
private:
    astar_heap<Coordinates, Compare> elements;
public:
    size_t size() {
        return elements.size();
    }

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
        elements = astar_heap<Coordinates, Compare>(
                std::greater<Node<Coordinates>>());
    }

    explicit Open(Compare comp) {
        elements = astar_heap<Coordinates, Compare>(comp);
    }
};

template<typename Coordinates>
class Closed {
private:
    boost::unordered_set<Coordinates> elements;
public:
    size_t size() {
        return elements.size();
    }

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

template<typename Coordinates>
std::size_t hash_value(TimedCoordinates<Coordinates> const &value) {
    size_t seed = 0;
    boost::hash_combine(seed, value.coordinates);
    boost::hash_combine(seed, value.time);
    return seed;
}

template<typename Coordinates>
using Path = std::vector<TimedCoordinates<Coordinates>>;

template<typename Coordinates, typename Compare = std::greater<Node<Coordinates>>>
std::pair<Path<Coordinates>, size_t>
astar(Graph<Coordinates> *graph, Coordinates start, Coordinates goal,
      Compare comp = std::greater<Node<Coordinates>>()) {
//    std::cout << "astar started" << std::endl;
    size_t expanded_nodes = 0;
    auto open = Open<Coordinates>(comp);
    boost::unordered_map<Coordinates, Coordinates> real_parent;
    boost::unordered_map<Coordinates, double> dist;
    auto closed = Closed<Coordinates>();
    auto start_node = Node<Coordinates>(start, 0, 0.0, start);
    open.add_node(start_node);
    real_parent[start] = start;
    dist[start] = 0.0;
    while (!open.empty()) {
        auto v = open.get_best_node();
//        std::cout << "iter" << std::endl;
//        std::cout << v.<< ' ' << v.coordinates.y << std::endl;
        if (closed.was_expanded(v.coordinates)) {
            continue;
        }
        closed.add_node(v.coordinates);
        real_parent[v.coordinates] = v.parent;
        dist[v.coordinates] = v.g_value;
//        std::cout << "added parent" << std::endl;
        if (graph->is_same_point(v.coordinates, goal)) {
            Path<Coordinates> path;
            auto cur_coors = v.coordinates;
            while (real_parent[cur_coors] != cur_coors) {
                path.push_back(TimedCoordinates<Coordinates>{cur_coors, dist[cur_coors]});
                cur_coors = real_parent[cur_coors];
            }
            path.push_back(TimedCoordinates<Coordinates>{cur_coors, dist[cur_coors]});
            std::reverse(path.begin(), path.end());
            expanded_nodes = open.size() + closed.size();
            return std::make_pair(path, expanded_nodes);
        }
        for (auto x : graph->get_neighbours(v.coordinates)) {
//            std::cout << "neighbour" << std::endl;
            if (!closed.was_expanded(x)) {
                auto nodeX = Node<Coordinates>(x, v.g_value + graph->get_cost(v.coordinates, x),
                                               graph->get_h_value(goal, x),
                                               v.coordinates);
                open.add_node(nodeX);
            }
        }
    }
    return std::make_pair(Path<Coordinates>(), 0);
}

template<typename Coordinates, typename Compare = std::greater<Node<Coordinates>>>
double count_path_len(Graph<Coordinates> *graph, Path<Coordinates> path) {
    if (path.empty()) {
        return -1.0;
    }
    Coordinates prev = path[0].coordinates;
    double len = 0.0;
    for (size_t i = 1; i < path.size(); i++) {
        len += graph->get_cost(prev, path[i].coordinates);
        prev = path[i].coordinates;
    }
    return len;
}

template<typename Coordinates, typename Compare = std::greater<Node<Coordinates>>>
bool is_path_correct(Graph<Coordinates> *graph, Path<Coordinates> path) {
    if (path.empty()) {
        return false;
    }
    Coordinates prev = path[0].coordinates;
    for (auto i = 1; i < path.size(); i++) {
        if (prev == path[i].coordinates) {
            continue;
        }
        std::vector<Coordinates> neighbours = graph->get_neighbours(prev);
//        std::cout << "prev coordinates" << prev.x << ' ' << prev.y << std::endl;
//        std::cout << "neighbours" << std::endl;
//        for (auto v : neighbours) {
//            std::cout << v.x << ' ' << v.y << std::endl;
//        }
//        std::cout << "current coors: " << path[i].coordinates.x << ' ' << path[i].coordinates.y << std::endl;
        if (std::find(neighbours.begin(), neighbours.end(), path[i].coordinates) == neighbours.end())
            return false;
        prev = path[i].coordinates;
    }
    return true;
}

#endif //COURSE_PROJECT_ASTAR_H
