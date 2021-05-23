//
// Created by olgashimanskaia on 20.05.2021.
//

#include "astar.h"
#include "map.h"

#include <gtest/gtest.h>
#include <iostream>
#include <task.h>
#include <bcbs.h>

const double EPS = 1e-6;

class BCBSTest : public ::testing::Test {
public:
    static void testBCBS(const std::string& mapFilename, const std::string& scenFilename, int taskStart, int taskFinish) {
        Map map = Map(mapFilename);
        std::vector<Task> tasks = Task::fromMovingAI(scenFilename);
        std::vector<Task> firstNTasks = std::vector<Task>();
        for (int i = taskStart; i < fmin(taskFinish, (int)tasks.size()); i++) {
            firstNTasks.push_back(tasks[i]);
        }
        auto bcbs = BCBS(map.generate_raw_grid(), 1.5);
        auto cbs = CBS(map.generate_raw_grid());
        std::vector<std::pair<Cell, Cell>> cell_tasks = std::vector<std::pair<Cell, Cell>>();
        for (const auto&task:firstNTasks) {
            cell_tasks.emplace_back(Cell({task.start.x, task.start.y}), Cell({task.finish.x, task.finish.y}));
        }
        auto paths = bcbs.find_paths(cell_tasks);
        auto cbs_paths = cbs.find_paths(cell_tasks);
        auto c1 = 0;
        auto c2 = 0;
        for (auto path: paths) {
            c1 += path.back().time;
        }
        for (auto path: cbs_paths) {
            c2 += path.back().time;
        }
        std::cout << c1 << ' ' << c2;
        ASSERT_EQ(paths.size(), cell_tasks.size());
        for (int i = 0; i < (int)paths.size(); i++) {
            ASSERT_TRUE(paths[i][0].coordinates == cell_tasks[i].first);
            ASSERT_TRUE(paths[i].back().coordinates == cell_tasks[i].second);
            ASSERT_TRUE(is_path_correct(&map, paths[i]));
        }
    }
protected:
    BCBSTest() {} //constructor runs before each test
    virtual ~BCBSTest() {} //destructor cleans up after tests
    virtual void SetUp() {} //sets up before each test (after constructor)
    virtual void TearDown() {} //clean up after each test, (before destructor)
};

TEST(BCBSTest, BCBSSimple) {
    BCBSTest::testBCBS("../data/maps/one-way-simple.map",
                     "../data/scens/mapf/one-way-simple.scen",
                     0, 2);
}

TEST(BCBSTest, BCBSBostonSmall1) {
    BCBSTest::testBCBS("../data/maps/mapf/Boston.map",
                       "../data/scens/mapf/Boston.scen",
                     0, 5);
}

TEST(BCBSTest, BCBSBostonSmall2) {
    BCBSTest::testBCBS("../data/maps/mapf/Boston.map",
                     "../data/scens/mapf/Boston.scen",
                     10, 15);
}

TEST(BCBSTest, BCBSBostonSmall3) {
    BCBSTest::testBCBS("../data/maps/mapf/Boston.map",
                       "../data/scens/mapf/Boston.scen",
                     300, 306);
}

TEST(BCBSTest, BCBSBostonSmall4) {
    BCBSTest::testBCBS("../data/maps/mapf/Boston.map",
                       "../data/scens/mapf/Boston.scen",
                       351, 352);
}

TEST(BCBSTest, BCBSCoastSmall1) {
    BCBSTest::testBCBS("../data/maps/mapf/w_woundedcoast.map",
                       "../data/scens/mapf/w_woundedcoast-even-1.scen",
                       0, 5);
}

TEST(BCBSTest, BCBSCoastSmall2) {
    BCBSTest::testBCBS("../data/maps/mapf/w_woundedcoast.map",
                       "../data/scens/mapf/w_woundedcoast-even-1.scen",
                       50, 56);
}

TEST(BCBSTest, BCBSCoastSmall3) {
    BCBSTest::testBCBS("../data/maps/mapf/w_woundedcoast.map",
                       "../data/scens/mapf/w_woundedcoast-even-1.scen",
                       75, 83);
}

TEST(BCBSTest, BCBSCoastSmall4) {
    BCBSTest::testBCBS("../data/maps/mapf/w_woundedcoast.map",
                       "../data/scens/mapf/w_woundedcoast-even-1.scen",
                       200, 203);
}

TEST(BCBSTest, BCBSMazeSmall1) {
    BCBSTest::testBCBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     0, 5);
}

TEST(BCBSTest, BCBSMazeSmall2) {
    BCBSTest::testBCBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     50, 60);
}

TEST(BCBSTest, BCBSMazeSmall3) {
    BCBSTest::testBCBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     75, 83);
}

TEST(BCBSTest, BCBSMazeSmall4) {
    BCBSTest::testBCBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     200, 220);
}

/*
TEST(BCBSTest, BCBSMazeLarge1) {
    BCBSTest::testBCBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     0, 40);
}

TEST(BCBSTest, BCBSMazeLarge2) {
    BCBSTest::testBCBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     100, 140);
}

TEST(BCBSTest, BCBSMazeLarge3) {
    BCBSTest::testBCBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     180, 220);
}

TEST(BCBSTest, BCBSMazeLarge4) {
    BCBSTest::testBCBS("../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     56, 90);
}*/