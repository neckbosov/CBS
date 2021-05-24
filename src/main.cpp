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
#include <cstdlib>
#include <cassert>
#include <random>
#include <filesystem>

using std::vector;

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

vector<Path<Cell>> run_CBS(const Map &map, const vector<Task> &tasks) {
    auto cbs = CBS(map.generate_raw_grid());
    auto cell_tasks = vector<std::pair<Cell, Cell>>();
    for (const auto &task : tasks) {
        cell_tasks.emplace_back(Cell({task.start.x, task.start.y}), Cell({task.finish.x, task.finish.y}));
    }
    return cbs.find_paths(cell_tasks);
}

vector<Path<Cell>> run_ECBS(const Map &map, const vector<Task> &tasks, double w) {
    auto ecbs = ECBS(w, map.generate_raw_grid());
    auto cell_tasks = vector<std::pair<Cell, Cell>>();
    for (const auto &task : tasks) {
        cell_tasks.emplace_back(Cell({task.start.x, task.start.y}), Cell({task.finish.x, task.finish.y}));
    }
    return ecbs.find_paths(cell_tasks);
}

vector<Path<Cell>> run_AFS(const Map &map, const vector<Task> &tasks) {
    auto afs = AFS_CBS(1.5, map.generate_raw_grid());
    auto cell_tasks = vector<std::pair<Cell, Cell>>();
    for (const auto &task : tasks) {
        cell_tasks.emplace_back(Cell({task.start.x, task.start.y}), Cell({task.finish.x, task.finish.y}));
    }
    return afs.find_paths(cell_tasks);
}

int main(int argc, char **argv) {
    std::filesystem::path res_path(argv[1]);
    std::string alg(argv[2]);
    Map map = Map(argv[3]);
    vector<Task> all_tasks = Task::fromMovingAI(argv[4]);
    int tasks_count = atoi(argv[5]);
    int test_num = atoi(argv[6]);
    vector<Task> tasks;
    vector<size_t> actors;
    auto gen = std::mt19937_64(test_num);
    vector<size_t> seq(all_tasks.size());
    std::iota(seq.begin(), seq.end(), 0);
    std::sample(seq.begin(), seq.end(), std::back_inserter(actors), tasks_count, gen);
    for (auto actor : actors) {
        tasks.push_back(all_tasks[actor]);
    }
    vector<Path<Cell>> paths;
    if (alg == "CBS") {
        paths = run_CBS(map, tasks);
    } else if (alg == "ECBS") {
        double w = atof(argv[7]);
        paths = run_ECBS(map, tasks, w);
    } else if (alg == "AFS") {
        paths = run_AFS(map, tasks);
    }
    std::ofstream out(res_path);
    out << "Actor,x,y,Time\n";
    for (int i = 0; i < tasks_count; i++) {
        for (auto cell: paths[i]) {
            out << actors[i] << ',' << cell.coordinates.x << ',' << cell.coordinates.y << ',' << cell.time << '\n';
        }
    }
    return 0;
}
