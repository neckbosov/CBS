from stats import run_algo
filename = 'result_optimal.txt'
import os

import tqdm
from collections import defaultdict
def parse_path_file(path_file_content):
    agents_path = defaultdict(list)
    agent = None
    lines = path_file_content.split('\n')[1:]
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

# test num takes index in range(0, num_repetitions)
def optimal_rate(alg_name, map_name, scen_name, num_agents, num_repetitions, w=None):
    rate = 0
    count_done = 0
    for i in tqdm.tqdm(range(num_repetitions)):
        done = run_algo(alg=alg_name,
                        dest_file=filename,
                        map_path=os.path.join(os.curdir, 'data', 'maps', 'mapf', map_name),
                        scen_path=os.path.join(os.curdir, 'data', 'scens', 'mapf', scen_name),
                        tasks_count=num_agents,
                        test_num=i,
                        w=w)
        if not done:
            continue

        sub_optimal_paths = read_paths(filename)

        done = run_algo(alg='CBS',
                        dest_file=filename,
                        map_path=os.path.join(os.curdir, 'data', 'maps', 'mapf', map_name),
                        scen_path=os.path.join(os.curdir, 'data', 'scens', 'mapf', scen_name),
                        tasks_count=num_agents,
                        test_num=i)

        if not done:
            continue

        optimal_paths = read_paths(filename)

        if len(optimal_paths.keys()) == 0:
            continue

        rates = []
        for key in optimal_paths.keys():
            optimal_len = len(optimal_paths[key])
            sub_optimal_len = len(sub_optimal_paths[key])
            #print(optimal_len, sub_optimal_len)
            if optimal_len != 0:
                rates.append(sub_optimal_len / optimal_len)

        if rates:
            rate += sum(rates) / len(rates)
        count_done += 1

    return rate / count_done



# build success_rate stats
random_map, random_scen = 'random-32-32-20.map', 'random-32-32-20-even-10.scen'
rate_list = []
N = 10
for num_actors in [1, 3, 5, 7, 10, 20, 40]:
    rate = optimal_rate('ECBS', random_map, random_scen, num_actors, N, w=1.5)
    print(f'Num actors :{num_actors}, Rate: {rate:.2f}%')
    if rate <= 0.1:
        print("too low rate, stopping here")
        break
    rate_list.append(rate)

print(rate_list)
