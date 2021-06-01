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
build/src/CBS_run
```

Бенчмарки находятся в файлах `Benchmarks.ipynb`, `Benchmarks-ECBS-CBS-opt-stat.ipynb` и `BenchmarksAFS-stat-weight-time.ipynb`
