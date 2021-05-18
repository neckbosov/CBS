//
// Created by olgashimanskaia on 15.05.2021.
//

#include "astar.h"
#include "map.h"

#include <gtest/gtest.h>
#include <iostream>
#include <task.h>
#include <cbs.h>

const double EPS = 1e-6;

class CBSTest : public ::testing::Test {
protected:
    CBSTest() {} //constructor runs before each test
    virtual ~CBSTest() {} //destructor cleans up after tests
    virtual void SetUp() {} //sets up before each test (after constructor)
    virtual void TearDown() {} //clean up after each test, (before destructor)
};

TEST(CBSTest, CBSSimple) {
    Map map = Map("../data/maps/one-way-simple.map");
    std::vector<Task> tasks = Task::fromMovingAI("../data/scens/mapf/one-way-simple.scen");
    auto cbs = CBS(map.generate_raw_grid());
    std::vector<std::pair<Cell, Cell>> cell_tasks = std::vector<std::pair<Cell, Cell>>();
    for (const auto&task:tasks) {
        cell_tasks.emplace_back(Cell({task.start.x, task.start.y}), Cell({task.finish.x, task.finish.y}));
    }
    auto paths = cbs.find_paths(cell_tasks);
    ASSERT_EQ(paths.size(), cell_tasks.size());
    for (int i = 0; i < (int)paths.size(); i++) {
        ASSERT_TRUE(paths[i][0].coordinates == cell_tasks[i].first);
        ASSERT_TRUE(paths[i].back().coordinates == cell_tasks[i].second);
        ASSERT_TRUE(is_path_correct(&map, paths[i]));
    }
}


TEST(CBSTest, CBSMaze) {
    MapAStar map = MapAStar("../data/maps/mapf/maze-32-32-2.map");
    std::vector<Task> tasks = Task::fromMovingAI("../data/scens/mapf/maze-32-32-2-even-1.scen");
    std::vector<Task> firstNTasks = std::vector<Task>();
    for (int i = 0; i < fmin(42, (int)tasks.size()); i++) {
        firstNTasks.push_back(tasks[i]);
    }
    auto cbs = CBS(map.generate_raw_grid());
    std::vector<std::pair<Cell, Cell>> cell_tasks = std::vector<std::pair<Cell, Cell>>();
    for (const auto&task:firstNTasks) {
        cell_tasks.emplace_back(Cell({task.start.x, task.start.y}), Cell({task.finish.x, task.finish.y}));
    }
    auto paths = cbs.find_paths(cell_tasks);
    ASSERT_EQ(paths.size(), cell_tasks.size());
    for (int i = 0; i < (int)paths.size(); i++) {
        ASSERT_TRUE(paths[i][0].coordinates == cell_tasks[i].first);
        ASSERT_TRUE(paths[i].back().coordinates == cell_tasks[i].second);
        ASSERT_TRUE(is_path_correct(&map, paths[i]));
    }
}
/*
TEST(CBSTest, CBSBoston) {
    MapAStar map = MapAStar("../data/maps/astar/Boston_0_256.map");
    std::vector<Task> tasks = Task::fromMovingAI("../data/scens/astar/Boston_0_256.map.scen");
    for (int i = 0; i < (int)tasks.size(); i += 10) {
        Task task = tasks[i];
        auto path = astar(&map, task.start, task.finish);
        auto result = count_path_len(&map, path);
        ASSERT_TRUE(std::abs(result - task.bestDistance) < EPS);
        ASSERT_TRUE(path[0].coordinates == task.start);
        ASSERT_TRUE(path.back().coordinates == task.finish);
    }
}*/