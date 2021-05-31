//
// Created by olgashimanskaia on 19.05.2021.
//

#include "astar.h"
#include "map.h"

#include <gtest/gtest.h>
#include <iostream>
#include <task.h>
#include <ecbs.h>

const double EPS = 1e-6;

class ECBSTest : public ::testing::Test {
public:
    static void testECBS(const std::string& mapFilename, const std::string& scenFilename, int taskStart, int taskFinish) {
        Map map = Map(mapFilename);
        std::vector<Task> tasks = Task::fromMovingAI(scenFilename);
        std::vector<Task> firstNTasks = std::vector<Task>();
        for (int i = taskStart; i < fmin(taskFinish, (int)tasks.size()); i++) {
            firstNTasks.push_back(tasks[i]);
        }
        auto cbs = ECBS(1.1, map.generate_raw_grid());
        std::vector<std::pair<Cell, Cell>> cell_tasks = std::vector<std::pair<Cell, Cell>>();
        for (const auto&task:firstNTasks) {
            cell_tasks.emplace_back(Cell({task.start.x, task.start.y}), Cell({task.finish.x, task.finish.y}));
        }
        auto [paths, hl, ll] = cbs.find_paths(cell_tasks);
        ASSERT_EQ(paths.size(), cell_tasks.size());
        for (int i = 0; i < (int)paths.size(); i++) {
            ASSERT_TRUE(paths[i][0].coordinates == cell_tasks[i].first);
            ASSERT_TRUE(paths[i].back().coordinates == cell_tasks[i].second);
            ASSERT_TRUE(is_path_correct(&map, paths[i]));
        }
    }
protected:
    ECBSTest() {} //constructor runs before each test
    virtual ~ECBSTest() {} //destructor cleans up after tests
    virtual void SetUp() {} //sets up before each test (after constructor)
    virtual void TearDown() {} //clean up after each test, (before destructor)
};

TEST(ECBSTest, ECBSSimple) {
    ECBSTest::testECBS("../data/maps/one-way-simple.map",
                     "../data/scens/mapf/one-way-simple.scen",
                     0, 2);
}

TEST(ECBSTest, ECBSBostonSmall1) {
    ECBSTest::testECBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     0, 5);
}

TEST(ECBSTest, ECBSBostonSmall2) {
    ECBSTest::testECBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     10, 15);
}

TEST(ECBSTest, ECBSBostonSmall3) {
    ECBSTest::testECBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     300, 306);
}

TEST(ECBSTest, ECBSBostonSmall4) {
    ECBSTest::testECBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     350, 357);
}

TEST(ECBSTest, ECBSMazeSmall1) {
    ECBSTest::testECBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     0, 5);
}

TEST(ECBSTest, ECBSMazeSmall2) {
    ECBSTest::testECBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     50, 60);
}

TEST(ECBSTest, ECBSMazeSmall3) {
    ECBSTest::testECBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     75, 83);
}

TEST(ECBSTest, ECBSMazeSmall4) {
    ECBSTest::testECBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     200, 220);
}

TEST(ECBSTest, ECBSMazeLarge1) {
    ECBSTest::testECBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     0, 30);
}

TEST(ECBSTest, ECBSMazeLarge2) {
    ECBSTest::testECBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     100, 130);
}

TEST(ECBSTest, ECBSMazeLarge3) {
    ECBSTest::testECBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     180, 210);
}

TEST(ECBSTest, ECBSMazeLarge4) {
    ECBSTest::testECBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     56, 80);
}

TEST(ECBSTest, ECBSCoastSmall1) {
    ECBSTest::testECBS("../data/maps/mapf/w_woundedcoast.map",
                     "../data/scens/mapf/w_woundedcoast-even-1.scen",
                     0, 5);
}

TEST(ECBSTest, ECBSCoastSmall2) {
    ECBSTest::testECBS("../data/maps/mapf/w_woundedcoast.map",
                     "../data/scens/mapf/w_woundedcoast-even-1.scen",
                     50, 56);
}

TEST(ECBSTest, ECBSCoastSmall3) {
    ECBSTest::testECBS("../data/maps/mapf/w_woundedcoast.map",
                     "../data/scens/mapf/w_woundedcoast-even-1.scen",
                     75, 83);
}

TEST(ECBSTest, ECBSCoastSmall4) {
    ECBSTest::testECBS("../data/maps/mapf/w_woundedcoast.map",
                     "../data/scens/mapf/w_woundedcoast-even-1.scen",
                     200, 203);
}

TEST(ECBSTest, ECBSden520dLarge1) {
    ECBSTest::testECBS("../data/maps/mapf/den520d.map",
                     "../data/scens/mapf/den520d-even-1.scen",
                     196, 200);
}

TEST(ECBSTest, ECBSbrc202dLarge1) {
    ECBSTest::testECBS("../data/maps/mapf/brc202d.map",
                     "../data/scens/mapf/brc202d-even-1.scen",
                     200, 206);
}