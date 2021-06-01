import subprocess
import os
from typing import Optional

EXE = os.path.join(os.curdir, 'cmake-build-release', 'src', 'CBS_run')



def run_algo(alg: str, dest_file: str, map_path: str, scen_path: str, tasks_count: int, test_num: int,
             w: Optional[float] = None, timeout: int = 2 * 60, afs_stat_weight_file=None) -> bool:
    try:
        cmd = [EXE, dest_file, alg, map_path, scen_path, str(tasks_count), str(test_num)]
        if w is not None and alg == 'ECBS':
            cmd.append(str(w))
        elif alg == 'AFS':
            cmd.append(str(timeout))
        if afs_stat_weight_file is not None:
            cmd.append(str(afs_stat_weight_file))
        if alg != 'AFS':
            subprocess.run(cmd, timeout=timeout)
        else:
            subprocess.run(cmd, timeout=timeout * 2)
        return True
    except subprocess.TimeoutExpired:
        return False

if __name__ == '__main__':
    print(run_algo(alg='ECBS', dest_file='result.txt',
             map_path=os.path.join(os.curdir, 'data', 'maps', 'mapf', 'brc202d.map'),
             scen_path=os.path.join(os.curdir, 'data', 'scens', 'mapf', 'brc202d-even-1.scen'),
             tasks_count=1,
             test_num=100, w=1.5, afs_stat_weight_file='result_weights_time_afs.txt'))