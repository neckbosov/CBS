#include <vector>
#include <set>
#include "afs_cbs.h"
#include "bcbs.h"
#include <ctime>

using std::vector;

AFS_CBS::AFS_CBS(double w, vector<std::string> raw_grid) {
    w1 = w;
    size_t n = raw_grid.size();
    size_t m = raw_grid.back().size();
    grid = vector<vector<int>>(n, vector<int>(m));
    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < m; j++) {
            if (raw_grid[i][j] == '.') {
                grid[i][j] = 0;
            } else {
                grid[i][j] = 1;
            }
        }
    }
}

vector<Path<Cell>> AFS_CBS::find_paths(const vector<std::pair<Cell, Cell>> &tasks, long seconds_limit) {
    clock_t tStart = clock();
    size_t actors = tasks.size();
    auto low_graph = AStarGridGraph(grid);
    int id = 0;
    auto root_node = BCBSHighLevelNode(actors, id);
    id++;
    for (size_t i = 0; i < actors; i++) {
        auto[start, goal] = tasks[i];
        root_node.solution[i] = astar(&low_graph, start, goal);
    }
    root_node.update_cost();
    root_node.update_h();

    auto g = [](const BCBSHighLevelNode &a, const BCBSHighLevelNode &b) {
        return a.cost < b.cost || (a.cost == b.cost && a.id < b.id);
    };

    auto h_c = [](const BCBSHighLevelNode &a, const BCBSHighLevelNode &b) {
        return a.h < b.h || (a.h == b.h && a.id < b.id);
    };
    std::set<BCBSHighLevelNode, decltype(g)> open(g);
    std::set<BCBSHighLevelNode, decltype(h_c)> focal(h_c);
    open.insert(root_node);
    focal.insert(root_node);

    int iter = 0;
    std::vector<Path<Cell>> paths = std::vector<Path<Cell>>();
    double prev_cost = 0;
    double curr_w = w1;
    while ((!paths.empty() || (clock() - tStart) / CLOCKS_PER_SEC < seconds_limit) && !open.empty()) {
        if (iter != 0) {
            BCBSHighLevelNode min_open = *open.begin();
            curr_w = prev_cost / min_open.cost.value() - 1e-4;

            if (curr_w <= 1 + 1e-8) {
                return paths;
            }

            std::set<BCBSHighLevelNode, decltype(h_c)> focal_old = focal;
            for (const BCBSHighLevelNode& n: focal_old) {
                if (n.cost > curr_w * min_open.cost.value()) {
                    focal.erase(n);
                }
            }
        }

        auto paths_new = BCBS(grid, curr_w).find_paths(tasks, open, focal, id, tStart, seconds_limit);

        if (paths_new.empty()) {
            return paths;
        } else {
            paths = paths_new;
            prev_cost = 0;
            for (const auto &path: paths) {
                prev_cost += path.back().time;
            }
        }
        iter++;
    }

    return paths;
}
