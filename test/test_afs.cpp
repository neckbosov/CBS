//
// Created by olgashimanskaia on 20.05.2021.
//

#include "astar.h"
#include "map.h"

#include <gtest/gtest.h>
#include <task.h>
#include "afs_cbs.h"
#include "util.h"

const double EPS = 1e-6;

class AFSTest : public ::testing::Test {
public:
    static void
    testAFS(const std::string &task_filename, const std::string &mapFilename, const std::string &scenFilename,
            int taskStart, int taskFinish) {
        Map map = Map(mapFilename);
        std::vector<Task> tasks = Task::fromMovingAI(scenFilename);
        std::vector<Task> firstNTasks = std::vector<Task>();
        for (int i = taskStart; i < fmin(taskFinish, (int)tasks.size()); i++) {
            firstNTasks.push_back(tasks[i]);
        }
        auto afs = AFS_CBS(1.5, map.generate_raw_grid());
        std::vector<std::pair<Cell, Cell>> cell_tasks = std::vector<std::pair<Cell, Cell>>();
        for (const auto&task:firstNTasks) {
            cell_tasks.emplace_back(Cell({task.start.x, task.start.y}), Cell({task.finish.x, task.finish.y}));
        }
        auto paths = afs.find_paths(cell_tasks, 2 * 60);
        print_paths_to_file(paths, task_filename);
        ASSERT_EQ(paths.size(), cell_tasks.size());
        for (int i = 0; i < (int)paths.size(); i++) {
            ASSERT_TRUE(paths[i][0].coordinates == cell_tasks[i].first);
            ASSERT_TRUE(paths[i].back().coordinates == cell_tasks[i].second);
            ASSERT_TRUE(is_path_correct(&map, paths[i]));
        }
    }

protected:
    AFSTest() {} //constructor runs before each test
    virtual ~AFSTest() {} //destructor cleans up after tests
    virtual void SetUp() {} //sets up before each test (after constructor)
    virtual void TearDown() {} //clean up after each test, (before destructor)
};


TEST(AFSTest, AFSSimple) {
    AFSTest::testAFS("AFSSimple.txt", "../data/maps/one-way-simple.map",
                     "../data/scens/mapf/one-way-simple.scen",
                     0, 2);
}

TEST(AFSTest, AFSBostonSmall1) {
    AFSTest::testAFS("AFSBostonSmall1.txt", "../data/maps/mapf/Boston.map",
                     "../data/scens/mapf/Boston.scen",
                     0, 5);
}

TEST(AFSTest, AFSBostonSmall2) {
    AFSTest::testAFS("AFSBostonSmall2.txt", "../data/maps/mapf/Boston.map",
                     "../data/scens/mapf/Boston.scen",
                     10, 15);
}

TEST(AFSTest, AFSBostonSmall3) {
    AFSTest::testAFS("AFSBostonSmall3.txt", "../data/maps/mapf/Boston.map",
                     "../data/scens/mapf/Boston.scen",
                     300, 306);
}

TEST(AFSTest, AFSBostonSmall4) {
    AFSTest::testAFS("AFSBostonSmall4.txt", "../data/maps/mapf/Boston.map",
                     "../data/scens/mapf/Boston.scen",
                     351, 352);
}

TEST(AFSTest, AFSCoastSmall1) {
    AFSTest::testAFS("AFSCoastSmall1.txt", "../data/maps/mapf/w_woundedcoast.map",
                     "../data/scens/mapf/w_woundedcoast-even-1.scen",
                     0, 5);
}

TEST(AFSTest, AFSCoastSmall2) {
    AFSTest::testAFS("AFSCoastSmall2.txt", "../data/maps/mapf/w_woundedcoast.map",
                     "../data/scens/mapf/w_woundedcoast-even-1.scen",
                     50, 56);
}

TEST(AFSTest, AFSCoastSmall3) {
    AFSTest::testAFS("AFSCoastSmall3.txt", "../data/maps/mapf/w_woundedcoast.map",
                     "../data/scens/mapf/w_woundedcoast-even-1.scen",
                     75, 83);
}

TEST(AFSTest, AFSCoastSmall4) {
    AFSTest::testAFS("AFSCoastSmall4.txt","../data/maps/mapf/w_woundedcoast.map",
                     "../data/scens/mapf/w_woundedcoast-even-1.scen",
                     200, 203);
}

TEST(AFSTest, AFSMazeSmall1) {
    AFSTest::testAFS("AFSMazeSmall1.txt", "../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     0, 5);
}

TEST(AFSTest, AFSMazeSmall2) {
    AFSTest::testAFS("AFSMazeSmall2.txt", "../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     50, 60);
}

TEST(AFSTest, AFSMazeSmall3) {
    AFSTest::testAFS("AFSMazeSmall3.txt", "../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     75, 83);
}

TEST(AFSTest, AFSMazeSmall4) {
    AFSTest::testAFS("AFSMazeSmall4.txt", "../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     200, 212);
}


TEST(AFSTest, AFSMazeLarge1) {
    AFSTest::testAFS("AFSMazeLarge1.txt", "../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     0, 40);
}

TEST(AFSTest, AFSMazeLarge2) {
    AFSTest::testAFS("AFSMazeLarge2.txt", "../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     100, 140);
}

TEST(AFSTest, AFSMazeLarge3) {
    AFSTest::testAFS("AFSMazeLarge3.txt", "../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     180, 220);
}

TEST(AFSTest, AFSMazeLarge4) {
    AFSTest::testAFS("AFSMazeLarge4.txt", "../data/maps/mapf/maze-32-32-2.map",
                     "../data/scens/mapf/maze-32-32-2-even-1.scen",
                     56, 90);
}