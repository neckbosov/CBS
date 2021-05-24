#include "graph.h"
#include "cbs.h"
#include "bcbs.h"
#include "afs_cbs.h"
#include "ecbs.h"
#include "task.h"
#include <iostream>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <cstring>
#include <cassert>
#include <random>


static void testECBS(std::string mapFilename, std::string scenFilename, int taskStart, int taskFinish) {
    if (mapFilename[0] == '.') {
        std::string my_dir = "C:/Users/pavlo/CLionProjects/CBS";
        mapFilename = my_dir + mapFilename.substr(2, mapFilename.size());
        scenFilename = my_dir + scenFilename.substr(2, scenFilename.size());
    }

    Map map = Map(mapFilename);
    std::vector<Task> tasks = Task::fromMovingAI(scenFilename);
    std::vector<Task> firstNTasks = std::vector<Task>();
    for (int i = taskStart; i < fmin(taskFinish, (int) tasks.size()); i++) {
        firstNTasks.push_back(tasks[i]);
    }
    auto cbs = ECBS(1.0, map.generate_raw_grid());
    std::vector<std::pair<Cell, Cell>> cell_tasks = std::vector<std::pair<Cell, Cell>>();
    for (const auto &task:firstNTasks) {
        cell_tasks.emplace_back(Cell({task.start.x, task.start.y}), Cell({task.finish.x, task.finish.y}));
    }
    auto paths = cbs.find_paths(cell_tasks);
    int id = 0;
    for (const auto &s: paths) {
        id += 1;
        std::cout << "id " << id << '\n';
        for (auto c: s) {
            std::cout << "{" << c.coordinates.x << ", " << c.coordinates.y << '}' << "@" << c.time << ' ';
        }
        std::cout << '\n';
    }
    std::cout << paths.size() << " == " << cell_tasks.size() << '\n';
    assert(paths.size() == cell_tasks.size());
    for (int i = 0; i < (int) paths.size(); i++) {
        assert(paths[i][0].coordinates == cell_tasks[i].first);
        assert(paths[i].back().coordinates == cell_tasks[i].second);
        assert(is_path_correct(&map, paths[i]));
    }
}

int main(int argc, char **argv) {
    std::string alg(argv[1]);
    Map map = Map(argv[2]);
    std::vector<Task> all_tasks = Task::fromMovingAI(argv[3]);
    int tasks_count = atoi(argv[4]);
    int test_num = atoi(argv[5]);
    std::vector<Task> tasks;
    auto gen = std::mt19937_64(test_num);
    std::sample(all_tasks.begin(), all_tasks.end(), std::back_inserter(tasks), tasks_count, gen);

//    vector<std::string> raw_grid;
//    raw_grid.emplace_back("...");
//    raw_grid.emplace_back("...");
//    raw_grid.emplace_back("...");
//    auto ecbs = ECBS(1.0, raw_grid);
//
//    auto start = Cell{0, 0};
//    auto goal = Cell{2, 2};
//    auto start2 = Cell{2, 0};
//    auto goal2 = Cell{1, 2};
//    vector<std::pair<Cell, Cell>> tasks = {{start,  goal},
//                                           {goal,   start},
//                                           {start2, goal2}};
//    auto solution = ecbs.find_paths(tasks);
//    int id = 0;
//    for (const auto &s: solution) {
//        id += 1;
//        std::cout << "id " << id << '\n';
//        for (auto c: s) {
//            std::cout << "{" << c.coordinates.x << ", " << c.coordinates.y << '}' << "@" << c.time << ' ';
//        }
//        std::cout << '\n';
//    }

}
