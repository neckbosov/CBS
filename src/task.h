#ifndef TASK_H
#define TASK_H

#include "map.h"

#include <filesystem>
#include <vector>

class Task {
public:
    const Pos start;
    const Pos finish;

    static std::vector<Task> fromMovingAI(const std::filesystem::path&);

    const double bestDistance;
protected:
    Task() = default;
    Task(std::string mapName,
             std::size_t mapHeight, std::size_t mapWidth,
             Pos from, Pos to, double bestDistance);

    const std::size_t mapHeight;
    const std::size_t mapWidth;
    const std::string mapName;
};

#endif