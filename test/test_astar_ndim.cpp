//
// Created by neckbosov on 25.05.2021.
//
#include <gtest/gtest.h>
#include <vector>
#include "map.h"
#include "astar_ndim.h"
#include "task.h"

class AstarNDimTest : public ::testing::Test {
public:
    static void
    testAstarNDim(const std::string &mapFilename, const std::string &scenFilename, int taskStart, int taskFinish) {
        Map map = Map(mapFilename);
        std::vector<Task> tasks = Task::fromMovingAI(scenFilename);
        std::vector<Task> firstNTasks = std::vector<Task>();
        for (int i = taskStart; i < fmin(taskFinish, (int) tasks.size()); i++) {
            firstNTasks.push_back(tasks[i]);
        }
        auto astarNDim = AstarNDim(map.generate_raw_grid());
        std::vector<std::pair<Cell, Cell>> cell_tasks = std::vector<std::pair<Cell, Cell>>();
        for (const auto &task:firstNTasks) {
            cell_tasks.emplace_back(Cell({task.start.x, task.start.y}), Cell({task.finish.x, task.finish.y}));
        }
        auto[paths, hl] = astarNDim.find_paths(cell_tasks);
        ASSERT_EQ(paths.size(), cell_tasks.size());
        for (int i = 0; i < (int) paths.size(); i++) {
            ASSERT_TRUE(paths[i][0].coordinates == cell_tasks[i].first);
            ASSERT_TRUE(paths[i].back().coordinates == cell_tasks[i].second);
            ASSERT_TRUE(is_path_correct(&map, paths[i]));
        }
    }

protected:
    AstarNDimTest() = default; //constructor runs before each test
    ~AstarNDimTest() override = default; //destructor cleans up after tests
    void SetUp() override {} //sets up before each test (after constructor)
    void TearDown() override {} //clean up after each test, (before destructor)
};

TEST(AstarNDimTest, AstarSimple) {
    AstarNDimTest::testAstarNDim("../data/maps/one-way-simple.map",
                                 "../data/scens/mapf/one-way-simple.scen",
                                 0, 2);
}

TEST(AstarNDimTest, AstarMaze) {
    AstarNDimTest::testAstarNDim("../data/maps/mapf/maze-32-32-2.map",
                                 "../data/scens/mapf/maze-32-32-2-even-1.scen",
                                 0, 2);
}

