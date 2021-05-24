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

struct TimedEdge {
    TimedCell first;
    TimedCell second;

    bool operator==(const TimedEdge &other) const {
        return (first == other.first && second == other.second) ||
               (first.time == other.first.time && second.time == other.second.time &&
                first.coordinates == other.second.coordinates && second.coordinates == other.first.coordinates);
    }

    bool operator!=(const TimedEdge &other) const {
        return !(*this == other);
    }
};

namespace std {
    template<>
    struct hash<Cell> {
        size_t operator()(const Cell &cell) const {
            auto int_hasher = hash<int>();
            return (int_hasher(cell.x) << 1) ^ int_hasher(cell.y);
        }
    };

    template<>
    struct hash<TimedEdge> {

        size_t operator()(const TimedEdge &edge) const {
            auto timed_cell_hasher = hash<TimedCell>();
            return (timed_cell_hasher(edge.first) << 1) ^ timed_cell_hasher(edge.second);
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
    std::unordered_set<TimedEdge> &banned_edges;

    double get_cost(TimedCell a, TimedCell b) override;

    double get_h_value(TimedCell goal, TimedCell current_coors) override;

    std::vector<TimedCell> get_neighbours(TimedCell coors) override;

    bool is_same_point(TimedCell a, TimedCell b) override;

    explicit CBSLowLevelGraph(
            vector<vector<int>> &cells,
            std::unordered_set<TimedCell> &banned_cells,
            std::unordered_set<TimedEdge> &banned_edges
    ) : source_cells(cells), banned_cells(banned_cells), banned_edges(banned_edges) {}

    CBSLowLevelGraph(vector<vector<int>> &&, std::unordered_set<TimedCell> &&) = delete;
};

using VertexConflict = std::optional<std::tuple<size_t, size_t, TimedCell>>;
using EdgeConflict = std::vector<std::pair<size_t, TimedEdge>>;

struct CBSHighLevelNode {
    vector<Path<Cell>> solution;
    vector<std::unordered_set<TimedCell>> vertex_conflicts;
    vector<std::unordered_set<TimedEdge>> edge_conflicts;
    std::optional<int> cost;

    bool operator>(const CBSHighLevelNode &other) const {
        return cost.value() > other.cost.value();
    }

    explicit CBSHighLevelNode(size_t actors);

    CBSHighLevelNode(const CBSHighLevelNode &other);

    void update_cost();

    VertexConflict find_vertex_conflict() const;

    EdgeConflict find_edge_conflict() const;

};

class CBS {
private:
    vector<vector<int>> grid;
public:
    explicit CBS(vector<std::string> raw_grid);

    std::tuple<vector<Path<Cell>>, size_t, size_t> find_paths(const vector<std::pair<Cell, Cell>> &tasks);
};

#endif //COURSE_PROJECT_CBS_H
