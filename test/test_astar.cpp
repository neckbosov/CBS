//
// Created by olgashimanskaia on 15.05.2021.
//

#include "astar.h"
#include "map.h"

#include <gtest/gtest.h>
#include <iostream>
#include <task.h>

const double EPS = 1e-6;

class AStarTest : public ::testing::Test {
protected:
    AStarTest() {} //constructor runs before each test
    virtual ~AStarTest() {} //destructor cleans up after tests
    virtual void SetUp() {} //sets up before each test (after constructor)
    virtual void TearDown() {} //clean up after each test, (before destructor)
};
/*
TEST(AStarTest, AStarSimple) {
    Map map = Map("../data/maps/one-way-simple.map");
    std::vector<Task> tasks = Task::fromMovingAI("../data/scens/astar/one-way-simple.scen");
    for (const auto &task:tasks) {
        auto path = astar(&map, task.start, task.finish);
        auto result = count_path_len(&map, path);
        ASSERT_TRUE(std::abs(result - task.bestDistance) < EPS);
        ASSERT_TRUE(path[0].coordinates == task.start);
        ASSERT_TRUE(path.back().coordinates == task.finish);
    }
}

TEST(AStarTest, AStarMaze) {
    MapAStar map = MapAStar("../data/maps/astar/maze512-16-0.map");
    std::vector<Task> tasks = Task::fromMovingAI("../data/scens/astar/maze512-16-0.map.scen");
    for (int i = 0; i < (int) tasks.size(); i += 50 * (int) log(i + 10)) {
        Task task = tasks[i];
        auto path = astar(&map, task.start, task.finish);
        auto result = count_path_len(&map, path);
        ASSERT_TRUE(std::abs(result - task.bestDistance) < EPS);
        ASSERT_TRUE(path[0].coordinates == task.start);
        ASSERT_TRUE(path.back().coordinates == task.finish);
    }
}

TEST(AStarTest, AStarBoston) {
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