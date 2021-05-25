import subprocess
import os
from typing import Optional

EXE = os.path.join(os.curdir, 'cmake-build-release', 'src', 'CBS_run')



def run_algo(alg: str, dest_file: str, map_path: str, scen_path: str, tasks_count: int, test_num: int,
             w: Optional[float] = None) -> bool:
    try:
        cmd = [EXE, dest_file, alg, map_path, scen_path, str(tasks_count), str(test_num)]
        if w is not None:
            cmd.append(str(w))
        subprocess.run(cmd, timeout=2 * 60)
        return True
    except subprocess.TimeoutExpired:
        return False

if __name__ == '__main__':
    print(run_algo(alg='ECBS', dest_file='result.txt',
             map_path=os.path.join(os.curdir, 'data', 'maps', 'mapf', 'brc202d.map'),
             scen_path=os.path.join(os.curdir, 'data', 'scens', 'mapf', 'brc202d-even-1.scen'),
             tasks_count=1,
             test_num=100, w=1.5))