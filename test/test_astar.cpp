//
// Created by olgashimanskaia on 15.05.2021.
//

#include "../astar.h"
#include "../map.h"

#include <gtest/gtest.h>
#include <iostream>

class SimpleGraph : public Graph<int> {
public:
    std::vector<std::unordered_map<int, int>> weights;
    std::vector<std::vector<int>> g;

    int get_cost(int a, int b) override {
        return weights[a][b];
    }

    double get_h_value(int goal, int current_coors) override {
        return 0.0;
    }

    std::vector<int> get_neighbours(int coors) override {
        return g[coors];
    }
};

class AStarTest : public ::testing::Test
{
protected:
    AStarTest(){} //constructor runs before each test
    virtual ~AStarTest(){} //destructor cleans up after tests
    virtual void SetUp(){} //sets up before each test (after constructor)
    virtual void TearDown(){} //clean up after each test, (before destructor)
};

TEST(AStarTest, kek) {
    ASSERT_TRUE(true);
}

TEST(AStarTest, Dijkstra) {
    Map map = Map::fromMovingAI("data/sample-moving-ai.map");
    auto path = astar(&map, Pos(1, 1), Pos(1, 3));
    auto result = count_path_len(&map, path);
    ASSERT_TRUE(std::abs(result - 4.0) < 1e-9);
    ASSERT_TRUE(path.size() > 2);
    ASSERT_TRUE(path[0].coordinates == Pos(1, 1));
    ASSERT_TRUE(path.back().coordinates == Pos(1, 3));
}