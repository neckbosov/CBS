//
// Created by neckbosov on 05.05.2021.
//

#ifndef COURSE_PROJECT_CBS_H
#define COURSE_PROJECT_CBS_H

#include "graph.h"
#include "astar.h"
#include <functional>
#include <vector>
#include <cmath>
#include <string>
#include <utility>
#include <optional>
#include <tuple>
#include <boost/unordered_set.hpp>
#include <boost/container_hash/hash.hpp>

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

std::size_t hash_value(Cell const &value);


using TimedCell = TimedCoordinates<Cell>;

struct TimedEdge {
    TimedCell first;
    TimedCell second;

    bool operator==(const TimedEdge &other) const {
        return (first == other.first && second == other.second);
    }

    bool operator!=(const TimedEdge &other) const {
        return !(*this == other);
    }
};

std::size_t hash_value(TimedEdge const &value);

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
    boost::unordered_set<TimedCell> &banned_cells;
    boost::unordered_set<TimedEdge> &banned_edges;

    double get_cost(TimedCell a, TimedCell b) override;

    double get_h_value(TimedCell goal, TimedCell current_coors) override;

    std::vector<TimedCell> get_neighbours(TimedCell coors) override;

    bool is_same_point(TimedCell a, TimedCell b) override;

    explicit CBSLowLevelGraph(
            vector<vector<int>> &cells,
            boost::unordered_set<TimedCell> &banned_cells,
            boost::unordered_set<TimedEdge> &banned_edges
    ) : source_cells(cells), banned_cells(banned_cells), banned_edges(banned_edges) {}

    CBSLowLevelGraph(vector<vector<int>> &&, boost::unordered_set<TimedCell> &&) = delete;
};

using VertexConflict = std::optional<std::tuple<size_t, size_t, TimedCell>>;
using EdgeConflict = std::vector<std::pair<size_t, TimedEdge>>;

struct CBSHighLevelNode {
    vector<Path<Cell>> solution;
    vector<boost::unordered_set<TimedCell>> vertex_conflicts;
    vector<boost::unordered_set<TimedEdge>> edge_conflicts;
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
