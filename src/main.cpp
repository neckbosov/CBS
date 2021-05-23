#include <iostream>
#include "graph.h"
#include "astar.h"
#include <unordered_map>
#include <vector>

class SimpleGraph : public Graph<int> {
public:
    std::vector<std::unordered_map<int, int>> weights;
    std::vector<std::vector<int>> g;

    double get_cost(int a, int b) override {
        return weights[a][b];
    }

    double get_h_value(int goal, int current_coors) override {
        return 0.0;
    }

    std::vector<int> get_neighbours(int coors) override {
        return g[coors];
    }
};

int main() {
    auto gr = SimpleGraph();
    gr.weights.resize(10);
    gr.g.resize(10);
    gr.g[1].push_back(2);
    gr.g[1].push_back(3);
    gr.g[2].push_back(4);
    gr.g[3].push_back(4);
    gr.weights[1][2] = 1;
    gr.weights[1][3] = 2;
    gr.weights[2][4] = 3;
    gr.weights[3][4] = 1;
    auto path = astar(&gr, 1, 4);
    for (auto[x, w]: path) {
        std::cout << x << ' ' << w << '\n';
    }
    return 0;
}
