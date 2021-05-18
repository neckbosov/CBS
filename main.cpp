#include <iostream>
#include "graph.h"
#include <unordered_map>
//#include "cbs.h"
#include <vector>
#include "ecbs.h"

class SimpleGraph : public Graph<int> {
public:
    std::vector<std::unordered_map<int, int>> weights;
    std::vector<std::vector<int>> g;

    int get_cost(int a, int b) override {
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
    vector<std::string> raw_grid;
    raw_grid.emplace_back("...");
    raw_grid.emplace_back("...");
    raw_grid.emplace_back("...");
    auto ecbs = ECBS(1.0, raw_grid);

    auto start = Cell{0, 0};
    auto goal = Cell{2, 2};
    vector<std::pair<Cell, Cell>> tasks = {{start, goal}, {goal, start}};
    ecbs.find_paths(tasks);
}
