# CBS и субоптимальные алгоритмы ECBS и AFS-CBS для решения задачи MAPF

В данном репозитории содержаться реализации следующих алгоритмов: A* (как обычный, так и для решения MAPF), AFS-CBS (и реализованный для него BCBS(w, 1)), CBS, ECBS. Все они находятся в папке `src`


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

Бенчмарки находятся в файлах `Benchmarks.ipynb`, `Benchmarks-ECBS-CBS-opt-stat.ipynb` и `BenchmarksAFS-stat-weight-time.ipynb`. Результаты, графики и выводы, а также более подробное описание содержится в отчете.
