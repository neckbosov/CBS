#ifndef TASK_H
#define TASK_H

#include "map.h"

#include <filesystem>
#include <vector>

class Task {
public:
    const std::pair<int, int> start;
    const std::pair<int, int> finish;

    double getBestPossibleDistance(const Map& map) const;
    static std::vector<Task> fromMovingAI(const std::filesystem::path&);
protected:
    Task() = default;
    Task(std::string mapName,
             std::size_t mapHeight, std::size_t mapWidth,
             std::pair<int, int>  from, std::pair<int, int>  to, double bestDistance);

    const double bestDistance;
    const std::size_t mapHeight;
    const std::size_t mapWidth;
    const std::string mapName;
};

#endif