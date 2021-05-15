#include "task.h"

#include <cmath>
#include <stdexcept>
#include <utility>
#include <string>

Task::Task(std::string mapName,
           std::size_t mapHeight, std::size_t mapWidth,
           std::pair<int, int> from, std::pair<int, int> to, double bestDistance)
        : start{std::move(from)}, finish{std::move(to)}, bestDistance{bestDistance}, mapHeight{mapHeight}, mapWidth{mapWidth},
          mapName{std::move(mapName)} {}

double Task::getBestPossibleDistance(const Map &map) const {
    double ratioW = static_cast<double>(map.width) / mapWidth;
    double ratioH = static_cast<double>(map.height) / mapHeight;
    if (fabs(ratioW - ratioH) > 1e-9)
        throw std::invalid_argument("scenario's map size cannot be scaled to actual map's size");
    return bestDistance * ratioW;
}

std::vector<Task> Task::fromMovingAI(const std::filesystem::path& path) {
    std::ifstream inp(path);
    std::string data;

    std::getline(inp, data);
    std::string version("version");
    if (data.length() < version.length()) {
        throw std::invalid_argument("unexpected header");
    }
    auto res = std::mismatch(version.begin(), version.end(), data.begin());

    if (res.first != version.end())
        throw std::invalid_argument("unexpected header");

    std::vector<Task> tasks;
    while (std::getline(inp, data) && data.length() > 0) {
        std::stringstream reader{data};
        int bucket;
        std::size_t w, h;
        std::string name;
        std::pair<int, int> from, to;
        double dist;
        if (!(reader >> bucket)) throw std::invalid_argument("expected bucket");
        if (!(reader >> name)) throw std::invalid_argument("expected map name");
        if (!(reader >> w)) throw std::invalid_argument("expected map width");
        if (!(reader >> h)) throw std::invalid_argument("expected map height");
        if (!(reader >> from.first)) throw std::invalid_argument("expected start.x");
        if (!(reader >> from.second)) throw std::invalid_argument("expected start.y");
        if (!(reader >> to.first)) throw std::invalid_argument("expected goal.x");
        if (!(reader >> to.second)) throw std::invalid_argument("expected goal.y");
        if (!(reader >> dist)) throw std::invalid_argument("expected best possible distance");
        tasks.push_back(Task(name, h, w, from, to, dist));
    }

    return tasks;
}
