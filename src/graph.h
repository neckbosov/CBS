//
// Created by neckbosov on 04.05.2021.
//

#ifndef COURSE_PROJECT_GRAPH_H
#define COURSE_PROJECT_GRAPH_H

#include <vector>
#include <string>

template<typename Coordinates>
class Node {
public:
    Coordinates coordinates;
    double g_value;
    double h_value;
    double f_value;
    Coordinates parent;

    bool operator<(const Node<Coordinates> &other) const {
        return this->f_value + 1e-7 < other.f_value;
    }

    bool operator>(const Node<Coordinates> &other) const {
        return this->f_value > other.f_value + 1e-7;
    }

    Node() {
        g_value = h_value = f_value = 0.0;
    }

    Node(Coordinates coordinates1, double g_value1, double h_value1, Coordinates parent1) {
        g_value = g_value1;
        coordinates = coordinates1;
        h_value = h_value1;
        f_value = h_value + g_value;
        parent = parent1;
    }

    Node(const Node<Coordinates> &other) {
        g_value = other.g_value;
        f_value = other.f_value;
        h_value = other.h_value;
        coordinates = other.coordinates;
        parent = other.parent;
    }
};

template<typename Coordinates>
class Graph {
public:
    virtual std::vector<Coordinates> get_neighbours(Coordinates coors) = 0;

    virtual double get_h_value(Coordinates goal, Coordinates current_coors) = 0;

    virtual double get_cost(Coordinates a, Coordinates b) = 0;

    virtual bool is_same_point(Coordinates a, Coordinates b) {
        return a == b;
    }
};

#endif //COURSE_PROJECT_GRAPH_H
