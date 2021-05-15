#ifndef MAP_H
#define MAP_H

#include <filesystem>
#include <fstream>
#include "graph.h"

struct Pos {
public:
    Pos(int i, int j) {
        this->x = i;
        this->y = j;
    }
    bool operator==(Pos const& b) {
        return std::tie(this->x, this->y) < std::tie(b.x, b.y);
    }
    bool operator<(Pos const& b) const {
        return x < b.x || (x == b.x && y < b.y);
    }
    bool operator!=(Pos const& b) {
        return std::tie(x, y) != std::tie(b.x, b.y);
    }
    int x;
    int y;
};

namespace std {

    template<> struct hash<Pos> {
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

    static Map fromMovingAI(const std::filesystem::path& path) {
        Map result;

        std::ifstream inp(path);
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
                result.map[i][j] = data[j] == '.' || data[j] == 'G' || data[j] == 'S' || data[j] == 'W';
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
        std::vector<Pos> neighbours = std::vector<Pos>();
        if (coors.x - 1 >= 0) {
            neighbours.emplace_back(coors.x - 1, coors.y);
        }
        if (coors.y - 1 >= 0) {
            neighbours.emplace_back(coors.x, coors.y - 1);
        }
        if (coors.x + 1 < height) {
            neighbours.emplace_back(coors.x + 1, coors.y);
        }
        if (coors.y + 1 < width) {
            neighbours.emplace_back(coors.x + 1, coors.y);
        }
        return neighbours;
    }
};

#endif