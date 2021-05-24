from stats import run_algo
filename = 'result2.txt'
import os



def read_expanded(filename=filename):
    with open(filename, 'r') as path_file:
        hl_expanded, ll_expanded = list(map(int, path_file.read().split('\n')[0].split(' ')[1:]))
        return hl_expanded, ll_expanded

import tqdm
# test num takes index in range(0, num_repetitions)
def run(alg_name, map_name, scen_name, num_agents, num_repetitions, w=None):
    hl_expanded = 0
    ll_expanded = 0
    for i in tqdm.tqdm(range(num_repetitions)):
        run_algo(alg=alg_name, dest_file=filename,
                        map_path=os.path.join(os.curdir, 'data', 'maps', 'mapf', map_name),
                        scen_path=os.path.join(os.curdir, 'data', 'scens', 'mapf', scen_name),
                        tasks_count=num_agents,
                        test_num=i, w=w)
        hl, ll = read_expanded(filename)
        hl_expanded += hl
        ll_expanded += ll
    return hl_expanded / num_repetitions, ll_expanded / num_repetitions



# build success_rate stats

random_map, random_scen = 'random-32-32-20.map', 'random-32-32-20-even-10.scen'
expanded_list = []
N = 10
algo = 'ECBS'
w = 1.5

for num_actors in [3, 4, 5, 6, 7, 8, 10, 13, 14, 16, 18, 20, 22]:
    expanded = run(algo, random_map, random_scen, num_actors, N, w)
    expanded_list.append(expanded)
    print(f'{algo} expanded nodes for {num_actors}: {expanded}')

print(expanded_list)
