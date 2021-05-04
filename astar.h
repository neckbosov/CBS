//
// Created by neckbosov on 04.05.2021.
//

#ifndef COURSE_PROJECT_ASTAR_H
#define COURSE_PROJECT_ASTAR_H

#include <queue>
#include <vector>
#include <unordered_set>
#include <tuple>
#include <optional>

#include "graph.h"

template<typename Coordinates, typename Compare=std::greater<Node<Coordinates>>>
class Open {
private:
    std::priority_queue<Node<Coordinates>, std::vector<Node<Coordinates>>, Compare> elements;
public:
    bool empty();

    Node<Coordinates> get_best_node();

    void add_node(Node<Coordinates> node);

    Open();

    explicit Open(Compare comp);
};

template<typename Coordinates>
class Closed {
private:
    std::unordered_set<Coordinates> elements;
public:
    void add_node(Coordinates coors);

    bool was_expanded(Coordinates coors);
};

template<typename Coordinates, typename Compare = std::greater<Node<Coordinates>>>
std::optional<Node<Coordinates>>
astar(Graph<Coordinates> graph, Coordinates start, Coordinates goal, Compare comp = std::greater<Node<Coordinates>>());

#endif //COURSE_PROJECT_ASTAR_H
