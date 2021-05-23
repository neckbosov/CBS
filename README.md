# CBS

`Compare` должен возвращать **более далекую** вершину, так как в `priority_queue` обратный порядок.

`Coordinates` должен быть хэшируемым (нужно реальзовать `template<> struct hash<Kek>` для вашего типа координат `Kek`).

Запуск тестов из консоли:

```
cd cmake-build-debug
./test/CBS_tst
```

Для запуска в Clion надо выставить `cmake-build-debug` в working directory.