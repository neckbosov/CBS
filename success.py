from stats import run_algo
filename = 'result2.txt'
import os

from collections import defaultdict
def parse_path_file(path_file_content):
    agents_path = defaultdict(list)
    agent = None
    lines = path_file_content.split('\n')
    for line in lines:
        if line == '':
            break
        nums = list(map(int, line.split(' ')))
        if len(nums) == 1:
            agent = int(nums[0])
        else:
            x, y, time = nums
            agents_path[agent].append((x, y, time))
    return agents_path

def read_paths(filename=filename):
    with open(filename, 'r') as path_file:
        path_file_content = path_file.read()
        agents_path = parse_path_file(path_file_content)
    return dict(agents_path)

import tqdm
# test num takes index in range(0, num_repetitions)
def success_rate(alg_name, map_name, scen_name, num_agents, num_repetitions, w=None):
    rate = 0
    for i in tqdm.tqdm(range(num_repetitions)):
        done = run_algo(alg=alg_name, dest_file=filename,
                        map_path=os.path.join(os.curdir, 'data', 'maps', 'mapf', map_name),
                        scen_path=os.path.join(os.curdir, 'data', 'scens', 'mapf', scen_name),
                        tasks_count=num_agents,
                        test_num=i, w=w)
        rate += int(done)
    return rate / num_repetitions

# build success_rate stats
random_map, random_scen = 'random-32-32-10.map', 'random-32-32-10-even-1.scen'
rate_list = []
N = 10
for num_actors in [5, 10, 20, 30, 40, 50]:
    rate = success_rate('CBS', random_map, random_scen, num_actors, N)
    print(f'Num actors :{num_actors}, Rate: {rate:.2f}%')
    rate_list.append(rate)

print(rate_list)