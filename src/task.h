#ifndef TASK_H
#define TASK_H

#include "map.h"

#include <filesystem>
#include <vector>

class Task {
public:
    const Cell start;
    const Cell finish;

    static std::vector<Task> fromMovingAI(const std::filesystem::path&);

    const double bestDistance;
protected:
    Task() = default;
    Task(Cell from, Cell to, double bestDistance);
};

#endif