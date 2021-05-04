//
// Created by neckbosov on 04.05.2021.
//

#include "astar.h"
#include <unordered_map>
#include <algorithm>

template<typename Coordinates, typename Compare>
bool Open<Coordinates, Compare>::empty() {
    return elements.empty();
}

template<typename Coordinates, typename Compare>
Node<Coordinates> Open<Coordinates, Compare>::get_best_node() {
    auto res = elements.top();
    elements.pop();
    return res;
}

template<typename Coordinates, typename Compare>
void Open<Coordinates, Compare>::add_node(Node<Coordinates> node) {
    elements.push(node);
}

template<typename Coordinates, typename Compare>
Open<Coordinates, Compare>::Open() {
    elements = std::priority_queue<Node<Coordinates>, std::vector<Node<Coordinates>>, Compare>(
            std::greater<Node<Coordinates>>());
}

template<typename Coordinates, typename Compare>
Open<Coordinates, Compare>::Open(Compare comp) {
    elements = std::priority_queue<Node<Coordinates>, std::vector<Node<Coordinates>>, Compare>(comp);
}

template<typename Node>
void Closed<Node>::add_node(Node coors) {
    elements.insert(coors);
}

template<typename Node>
bool Closed<Node>::was_expanded(Node coors) {
    return elements.find(coors) != elements.end();
}

template<typename Coordinates, typename Compare>
std::vector<std::pair<Coordinates, double>>
astar(Graph<Coordinates> graph, Coordinates start, Coordinates goal, Compare comp) {
    auto open = Open<Coordinates, Compare>(comp);
    std::unordered_map<Coordinates, Coordinates> real_parent;
    std::unordered_map<Coordinates, double> dist;
    auto closed = Closed<Coordinates>();
    auto start_node = Node(start, 0.0, 0.0, start);
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
        if (v.coordinates == goal) {
            std::vector<Coordinates> path;
            auto cur_coors = v.coordinates;
            while (real_parent[cur_coors] != cur_coors) {
                path.emplace_back(cur_coors, dist[cur_coors]);
                cur_coors = real_parent[cur_coors];
            }
            path.push_back(cur_coors, dist[cur_coors]);
            std::reverse(path.begin(), path.end());
            return path;
        }
        for (auto x : graph.get_neighbours(v.coordinates)) {
            if (!closed.was_expanded(x)) {
                auto nodeX = Node(x, v.g_value + graph.get_cost(v, x), graph.get_h_value(goal, x), v.coordinates);
            }
        }
    }
    return std::vector<std::pair<Coordinates, double>>();
}