//
// Created by neckbosov on 04.05.2021.
//

#include "graph.h"

template<typename Coordinates>
bool Node<Coordinates>::operator<(const Node<Coordinates> &other) {
    return this->f_value < other.f_value;
}

template<typename Coordinates>
Node<Coordinates>::Node(Coordinates coordinates1, double g_value1, double h_value1, Coordinates parent) {
    g_value = g_value1;
    coordinates = coordinates1;
    h_value = h_value1;
    f_value = h_value + g_value;
    this->parent = parent;
}

template<typename Coordinates>
Node<Coordinates>::Node() {
    g_value = h_value = f_value = 0.0;
}

template<typename Coordinates>
Node<Coordinates>::Node(const Node &other) {
    g_value = other.g_value;
    f_value = other.f_value;
    h_value = other.h_value;

}
