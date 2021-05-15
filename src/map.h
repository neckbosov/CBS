#ifndef MAP_H
#define MAP_H

#include <filesystem>
#include <fstream>
#include "graph.h"

using std::vector;

struct Pos {
public:
    bool operator==(Pos const &b) const {
        return this->x == b.x && this->y == b.y;
    }

    bool operator<(Pos const &b) const {
        return x < b.x || (x == b.x && y < b.y);
    }

    bool operator!=(Pos const &b) const {
        return !(*this == b);
    }

    Pos operator+(const Pos &other) const {
        return Pos{x + other.x, y + other.y};
    }

    int x;
    int y;
};

namespace std {

    template<>
    struct hash<Pos> {
        size_t operator()(const Pos &pos) const {
            auto int_hasher = hash<int>();
            return (int_hasher(pos.x) << 1) ^ int_hasher(pos.y);
        }
    };

}

class Map : public Graph<Pos> {
public:
    std::vector<std::vector<bool>> map;
    int width = 0;
    int height = 0;

    static Map fromMovingAI(const std::filesystem::path &path) {
        Map result;

        std::ifstream inp(path);
        if (!inp.is_open()) {
            throw std::invalid_argument("unknown path");
        }
        std::string data;

        std::getline(inp, data);
        if (data != "type octile")
            throw std::invalid_argument("unexpected header");

        int height, width;
        std::getline(inp, data);
        if (sscanf(data.c_str(), "height %d", &height) != 1)
            throw std::invalid_argument("expected height on the second line");
        std::getline(inp, data);
        if (sscanf(data.c_str(), "width %d", &width) != 1)
            throw std::invalid_argument("expected width on the third line");

        if (height <= 0 || width <= 0)
            throw std::invalid_argument("height and/or width have illegal values");

        result.height = height;
        result.width = width;
        result.map = std::vector(height, std::vector<bool>(width, false));

        std::getline(inp, data);
        if (data != "map")
            throw std::invalid_argument("unexpected header on the fourth line");

        for (int i = 0; i < height; i++) {
            std::getline(inp, data);
            if (data.length() != static_cast<std::size_t>(width))
                throw std::invalid_argument("map's width does not match the content's width");
            for (int j = 0; j < width; j++)
                result.map[i][j] = (data[j] == '.' || data[j] == 'G' || data[j] == 'S' || data[j] == 'W');
        }

        return result;
    }

    int get_cost(Pos a, Pos b) override {
        return 1;
    }

    double get_h_value(Pos goal, Pos current_coors) override {
        return std::abs(goal.x - current_coors.x) + std::abs(current_coors.y - goal.y);
    }

    // get 4-conn neighbours for CBS
    std::vector<Pos> get_neighbours(Pos coors) override {
        vector<Pos> directions{{0,  1},
                               {1,  0},
                               {0,  -1},
                               {-1, 0}};
        vector<Pos> res;
        for (Pos mv: directions) {
            Pos cur_cell = coors + mv;
            if (cur_cell.x >= 0 && cur_cell.x < (int) map.size() && cur_cell.y >= 0 &&
                cur_cell.y < (int) map.back().size() && map[cur_cell.x][cur_cell.y]) {
                res.push_back(cur_cell);
            }

        }
        return res;
    }
};

#endif