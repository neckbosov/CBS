//
// Created by neckbosov on 05.05.2021.
//

#ifndef COURSE_PROJECT_CBS_H
#define COURSE_PROJECT_CBS_H

#include "graph.h"
#include "astar.h"
#include <functional>
#include <vector>
#include <unordered_set>
#include <cmath>
#include <string>
#include <utility>
#include <optional>
#include <tuple>

using std::vector;
using std::hypot;
using std::abs;


struct Cell {
    int x;
    int y;

    Cell operator+(const Cell &other) const {
        return Cell{x + other.x, y + other.y};
    }

    bool operator==(const Cell &other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Cell &other) const {
        return !(*this == other);
    }
};


using TimedCell = TimedCoordinates<Cell>;

namespace std {
    template<>
    struct hash<Cell> {
        size_t operator()(const Cell &cell) const {
            auto int_hasher = hash<int>();
            return (int_hasher(cell.x) << 1) ^ int_hasher(cell.y);
        }
    };
}

class AStarGridGraph : public Graph<Cell> {
public:
    vector<vector<int>> &source_cells;

    double get_cost(Cell a, Cell b) override;

    double get_h_value(Cell goal, Cell current_coors) override;

    std::vector<Cell> get_neighbours(Cell coors) override;

    explicit AStarGridGraph(vector<vector<int>> &cells) : source_cells(cells) {}

    AStarGridGraph(vector<vector<int>> &&) = delete;
};

class CBSLowLevelGraph : public Graph<TimedCell> {
public:
    vector<vector<int>> &source_cells;
    std::unordered_set<TimedCell> &banned_cells;

    double get_cost(TimedCell a, TimedCell b) override;

    double get_h_value(TimedCell goal, TimedCell current_coors) override;

    std::vector<TimedCell> get_neighbours(TimedCell coors) override;

    bool is_same_point(TimedCell a, TimedCell b) override;

    explicit CBSLowLevelGraph(vector<vector<int>> &cells, std::unordered_set<TimedCell> &banned) : source_cells(cells),
                                                                                                   banned_cells(
                                                                                                           banned) {}

    CBSLowLevelGraph(vector<vector<int>> &&, std::unordered_set<TimedCell> &&) = delete;
};

using Conflict = std::optional<std::tuple<size_t, size_t, TimedCell>>;

struct CBSHighLevelNode {
    vector<Path<Cell>> solution;
    vector<std::unordered_set<TimedCell>> vertex_conflicts;
    std::optional<int> cost;

    bool operator>(const CBSHighLevelNode &other) const {
        if (!cost.has_value()) {
            return false;
        } else if (!other.cost.has_value()) {
            return true;
        } else {
            return cost.value() < other.cost.value();
        }
    }

    explicit CBSHighLevelNode(size_t actors);

    CBSHighLevelNode(const CBSHighLevelNode &other);

    void update_cost();

    Conflict find_conflict() const;

};

class CBS {
private:
    vector<vector<int>> grid;
public:
    explicit CBS(vector<std::string> raw_grid);

    vector<Path<Cell>> find_paths(const vector<std::pair<Cell, Cell>> &tasks);
};

#endif //COURSE_PROJECT_CBS_H
