//
// Created by neckbosov on 04.05.2021.
//

#ifndef COURSE_PROJECT_GRAPH_H
#define COURSE_PROJECT_GRAPH_H

#include <vector>

template<typename Coordinates>
class Node {
public:
    Coordinates coordinates;
    double g_value;
    double h_value;
    double f_value;
    Coordinates parent;

    bool operator<(const Node &other);

    Node();

    Node(Coordinates coordinates1, double g_value1, double h_value1,Coordinates parent);

    Node(const Node &other);
};

template<typename Coordinates>
class Graph {
public:
    virtual std::vector<Coordinates> get_neighbours(Coordinates coors) = 0;

    virtual double get_h_value(Coordinates goal, Coordinates current_coors) = 0;

    virtual double get_cost(Coordinates a, Coordinates b) = 0;
};

#endif //COURSE_PROJECT_GRAPH_H
