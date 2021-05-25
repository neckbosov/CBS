#include "graph.h"
#include "cbs.h"
#include "bcbs.h"
#include "afs_cbs.h"
#include "ecbs.h"
#include "task.h"
#include "util.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <random>
#include <filesystem>

using std::vector;

std::tuple<vector<Path<Cell>>, size_t, size_t> run_CBS(const Map &map, const vector<Task> &tasks) {
    auto cbs = CBS(map.generate_raw_grid());
    auto cell_tasks = vector<std::pair<Cell, Cell>>();
    for (const auto &task : tasks) {
        cell_tasks.emplace_back(Cell({task.start.x, task.start.y}), Cell({task.finish.x, task.finish.y}));
    }
    return cbs.find_paths(cell_tasks);
}

std::tuple<vector<Path<Cell>>, size_t, size_t> run_ECBS(const Map &map, const vector<Task> &tasks, double w) {
    auto ecbs = ECBS(w, map.generate_raw_grid());
    auto cell_tasks = vector<std::pair<Cell, Cell>>();
    for (const auto &task : tasks) {
        cell_tasks.emplace_back(Cell({task.start.x, task.start.y}), Cell({task.finish.x, task.finish.y}));
    }
    return ecbs.find_paths(cell_tasks);
}

vector<Path<Cell>> run_AFS(const Map &map, const vector<Task> &tasks, int timeout) {
    auto afs = AFS_CBS(1.5, map.generate_raw_grid());
    auto cell_tasks = vector<std::pair<Cell, Cell>>();
    for (const auto &task : tasks) {
        cell_tasks.emplace_back(Cell({task.start.x, task.start.y}), Cell({task.finish.x, task.finish.y}));
    }
    return afs.find_paths(cell_tasks, timeout);
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
    tasks.reserve(actors.size());
    for (auto actor : actors) {
        tasks.push_back(all_tasks[actor]);
    }
    vector<Path<Cell>> paths;
    size_t expanded = 0;
    size_t low_expanded = 0;
    if (alg == "CBS") {
        auto [cbs_paths,  hl_ex, ll_ex] = run_CBS(map, tasks);
        paths = cbs_paths;
        expanded = hl_ex;
        low_expanded = ll_ex;
    } else if (alg == "ECBS") {
        double w = atof(argv[7]);
        auto [ecbs_paths, hl_ex, ll_ex] = run_ECBS(map, tasks, w);
        paths = ecbs_paths;
        expanded = hl_ex;
        low_expanded = ll_ex;
    } else if (alg == "AFS") {
        int timeout = atoi(argv[7]);
        paths = run_AFS(map, tasks, timeout);
    }
    print_paths_to_file(paths, res_path.string(), expanded, low_expanded);


    return 0;
}
