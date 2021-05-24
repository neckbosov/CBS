#ifndef MAP_H
#define MAP_H

#include <filesystem>
#include <fstream>
#include "graph.h"
#include "iostream"
#include "cbs.h"
#include <algorithm>
#include <cmath>
#include <string>

using std::vector;

class Map : public Graph<Cell> {
private:
    static void removeNewLine(std::string &data) {
        if (!data.empty() && (data[data.length() - 1] == '\n' || (int) data[data.length() - 1] == 13)) {
            data.erase(data.length() - 1);
        }
    }

public:
    std::vector<std::vector<bool>> map;
    std::string filename;
    int width = 0;
    int height = 0;

    explicit Map(const std::filesystem::path &path) {
        std::ifstream inp(path);
        if (!inp.is_open()) {
            throw std::invalid_argument("unknown path");
        }
        filename = path.filename().string();
        std::string data;

        std::getline(inp, data);
        removeNewLine(data);
        if (data != "type octile")
            throw std::invalid_argument("unexpected header");

        int h, w;
        std::getline(inp, data);
        if (sscanf(data.c_str(), "height %d", &h) != 1)
            throw std::invalid_argument("expected height on the second line");
        std::getline(inp, data);
        if (sscanf(data.c_str(), "width %d", &w) != 1)
            throw std::invalid_argument("expected width on the third line");

        if (h <= 0 || w <= 0)
            throw std::invalid_argument("height and/or width have illegal values");

        this->height = h;
        this->width = w;
        this->map = std::vector(h, std::vector<bool>(w, false));

        std::getline(inp, data);
        removeNewLine(data);
        if (data != "map")
            throw std::invalid_argument("unexpected header on the fourth line");

        for (int i = 0; i < h; i++) {
            std::getline(inp, data);
            removeNewLine(data);
            if (data.length() != static_cast<std::size_t>(w))
                throw std::invalid_argument("map's width does not match the content's width");
            for (int j = 0; j < w; j++)
                this->map[i][j] = (data[j] == '.' || data[j] == 'G' || data[j] == 'S' || data[j] == 'W');
        }
    }

    double get_cost(Cell a, Cell b) override {
        return 1;
    }

    // euclidean metric
    double get_h_value(Cell goal, Cell current_coors) override {
        return std::sqrt((double) abs(goal.x - current_coors.x) * abs(goal.x - current_coors.x) +
                         (double) abs(goal.y - current_coors.y) * abs(goal.y - current_coors.y));
    }

    // get 4-conn neighbours for CBS
    std::vector<Cell> get_neighbours(Cell coors) override {
        vector<Cell> directions{{0,  1},
                                {1,  0},
                                {0,  -1},
                                {-1, 0}};
        vector<Cell> res;
        for (Cell mv: directions) {
            Cell cur_cell = coors + mv;
            if (cur_cell.x >= 0 && cur_cell.x < (int) map.size() && cur_cell.y >= 0 &&
                cur_cell.y < (int) map.back().size() && map[cur_cell.x][cur_cell.y]) {
                res.push_back(cur_cell);
            }

        }
        return res;
    }

    std::vector<std::string> generate_raw_grid() const {
        std::vector<std::string> raw_grid = std::vector<std::string>();
        for (const auto &line: map) {
            std::stringstream raw_grid_line;
            for (const auto &cell: line) {
                raw_grid_line << (cell ? '.' : '#');
            }
            raw_grid.push_back(raw_grid_line.str());
        }
        return raw_grid;
    }
};

class MapAStar : public Map {
public:

    explicit MapAStar(const std::filesystem::path &path) : Map(path) {

    }

    double get_cost(Cell a, Cell b) override {
        return std::sqrt((double) std::abs(a.x - b.x) * std::abs(a.x - b.x) +
                         (double) std::abs(a.y - b.y) * std::abs(a.y - b.y));
    }

    // get 8-conn neighbours for testing AStar
    std::vector<Cell> get_neighbours(Cell coors) override {
        vector<Cell> directions{{0,  1},
                                {1,  0},
                                {0,  -1},
                                {-1, 0},};
        vector<Cell> diagDirections{{1,  1},
                                    {-1, -1},
                                    {1,  -1},
                                    {-1, 1}};
        vector<Cell> res;
        for (Cell mv: directions) {
            Cell cur_cell = coors + mv;
            if (cur_cell.x >= 0 && cur_cell.x < (int) map.size() && cur_cell.y >= 0 &&
                cur_cell.y < (int) map.back().size() && map[cur_cell.x][cur_cell.y]) {
                res.push_back(cur_cell);
            }

        }
        for (Cell mv: diagDirections) {
            Cell cur_cell = coors + mv;
            if (cur_cell.x >= 0 && cur_cell.x < (int) map.size() && cur_cell.y >= 0 &&
                cur_cell.y < (int) map.back().size() && map[cur_cell.x][cur_cell.y] &&
                (std::find(res.begin(), res.end(), Cell({cur_cell.x - mv.x, cur_cell.y})) != res.end()) &&
                (std::find(res.begin(), res.end(), Cell({cur_cell.x, cur_cell.y - mv.y})) != res.end())) {
                res.push_back(cur_cell);
            }
        }
        return res;
    }
};

#endif