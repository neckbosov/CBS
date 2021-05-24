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
public:
    static void
    testCBS(const std::string &mapFilename, const std::string &scenFilename, int taskStart, int taskFinish) {
        Map map = Map(mapFilename);
        std::vector<Task> tasks = Task::fromMovingAI(scenFilename);
        std::vector<Task> firstNTasks = std::vector<Task>();
        for (int i = taskStart; i < fmin(taskFinish, (int) tasks.size()); i++) {
            firstNTasks.push_back(tasks[i]);
        }
        auto cbs = CBS(map.generate_raw_grid());
        std::vector<std::pair<Cell, Cell>> cell_tasks = std::vector<std::pair<Cell, Cell>>();
        for (const auto &task:firstNTasks) {
            cell_tasks.emplace_back(Cell({task.start.x, task.start.y}), Cell({task.finish.x, task.finish.y}));
        }
        auto paths = cbs.find_paths(cell_tasks);
        ASSERT_EQ(paths.size(), cell_tasks.size());
        for (int i = 0; i < (int) paths.size(); i++) {
            ASSERT_TRUE(paths[i][0].coordinates == cell_tasks[i].first);
            ASSERT_TRUE(paths[i].back().coordinates == cell_tasks[i].second);
            ASSERT_TRUE(is_path_correct(&map, paths[i]));
        }
    }

protected:
    CBSTest() = default; //constructor runs before each test
    ~CBSTest() override = default; //destructor cleans up after tests
    void SetUp() override {} //sets up before each test (after constructor)
    void TearDown() override {} //clean up after each test, (before destructor)
};

TEST(CBSTest, CBSSimple) {
    CBSTest::testCBS("../data/maps/one-way-simple.map",
                     "../data/scens/mapf/one-way-simple.scen",
                     0, 2);
}

TEST(CBSTest, CBSBostonSmall1) {
    CBSTest::testCBS("../data/maps/mapf/Boston.map",
                     "../data/scens/mapf/Boston.scen",
                     0, 5);
}

TEST(CBSTest, CBSBostonSmall2) {
    CBSTest::testCBS("../data/maps/mapf/Boston.map",
                     "../data/scens/mapf/Boston.scen",
                     10, 15);
}

TEST(CBSTest, CBSBostonSmall3) {
    CBSTest::testCBS("../data/maps/mapf/Boston.map",
                     "../data/scens/mapf/Boston.scen",
                     300, 306);
}

TEST(CBSTest, CBSBostonSmall4) {
    CBSTest::testCBS("../data/maps/mapf/Boston.map",
                     "../data/scens/mapf/Boston.scen",
                     350, 354);
}

TEST(CBSTest, CBSMazeSmall1) {
    CBSTest::testCBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     0, 5);
}

TEST(CBSTest, CBSMazeSmall2) {
    CBSTest::testCBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     50, 60);
}

TEST(CBSTest, CBSMazeSmall3) {
    CBSTest::testCBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     75, 83);
}

TEST(CBSTest, CBSMazeSmall4) {
    CBSTest::testCBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     200, 220);
}

TEST(CBSTest, CBSMazeLarge1) {
    CBSTest::testCBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     0, 40);
}

TEST(CBSTest, CBSMazeLarge2) {
    CBSTest::testCBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     100, 130);
}

TEST(CBSTest, CBSMazeLarge3) {
    CBSTest::testCBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     180, 220);
}

TEST(CBSTest, CBSMazeLarge4) {
    CBSTest::testCBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     56, 90);
}

TEST(CBSTest, CBSCoastSmall1) {
    CBSTest::testCBS("../data/maps/mapf/w_woundedcoast.map",
                     "../data/scens/mapf/w_woundedcoast-even-1.scen",
                     0, 5);
}

TEST(CBSTest, CBSCoastSmall2) {
    CBSTest::testCBS("../data/maps/mapf/w_woundedcoast.map",
                     "../data/scens/mapf/w_woundedcoast-even-1.scen",
                     50, 56);
}

TEST(CBSTest, CBSCoastSmall3) {
    CBSTest::testCBS("../data/maps/mapf/w_woundedcoast.map",
                     "../data/scens/mapf/w_woundedcoast-even-1.scen",
                     75, 83);
}

TEST(CBSTest, CBSCoastSmall4) {
    CBSTest::testCBS("../data/maps/mapf/w_woundedcoast.map",
                     "../data/scens/mapf/w_woundedcoast-even-1.scen",
                     200, 203);
}