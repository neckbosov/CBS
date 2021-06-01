# CBS

Зависимости: `g++`, `googletest`, `python`, `python-tqdm`, `jupyter`, `boost`

Сборка:

```bash
mkdir build && cd build && cmake .. && make -j6 && cd ..
```

Запуск тестов:

```bash
build/test/CBS_tst
```

Запуск основного приложения:

```bash
build/src/CBS_run results_path algo_name map_path scen_path tasks_count test_num
```

Аргументы:

* `results_path` - путь к файлу для записи результата
* `algo_name` - имя алгоритма (`CBS`, `ECBS`, `AFS`, `ASTAR`)
* `map_path` - путь к файлу карты movingai
* `scen_path` - путь к файлу сценария movingai
* `tasks_count` - количество акторов для теста
* `test_num` - id теста, используется для генерации заданий из файла

Для `AFS` и `ECBS` также последним аргументов необходимо передать параметр `w` алгоритма.

Бенчмарки находятся в файлах `Benchmarks.ipynb`, `Benchmarks-ECBS-CBS-opt-stat.ipynb`
и `BenchmarksAFS-stat-weight-time.ipynb`
