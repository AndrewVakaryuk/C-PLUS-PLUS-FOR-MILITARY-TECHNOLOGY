# Демо 2.4: debug_probe для локального і віддаленого GDB

Цей приклад використовується на занятті 2.4 для показу одного циклу
відлагодження:

```text
відтворити -> оглянути -> підтвердити -> виправити -> перевірити
```

Код дуже малий і має дві навмисні помилки під час виконання. Одна проявляється
як падіння на некоректних вхідних даних, друга добре видна через Valgrind на
коректних вхідних даних. Точні місця в коді не позначено, щоб демо проходило
через інструменти, а не через читання готової підказки.

## Локальний запуск

```bash
cmake --preset debug
cmake --build --preset debug

./build/debug/demos/lesson_2_4/debug_probe/debug_probe \
  demos/lesson_2_4/debug_probe/data/good.txt
```

Очікуваний вивід:

```text
seq 7
battery_v 24.6
satellites 12
health ready
```

## Консольний GDB

```bash
gdb ./build/debug/demos/lesson_2_4/debug_probe/debug_probe
```

Всередині GDB:

```text
run demos/lesson_2_4/debug_probe/data/bad_missing_field.txt
bt
frame 0
print text
up
print field_count
print fields[2]
quit
```

## Valgrind

```bash
valgrind --leak-check=full \
  ./build/debug/demos/lesson_2_4/debug_probe/debug_probe \
  demos/lesson_2_4/debug_probe/data/good.txt
```

Очікуваний сигнал: `definitely lost` memory block.

Проблемні вхідні дані також можна запускати через Valgrind, щоб побачити
некоректне читання:

```bash
valgrind \
  ./build/debug/demos/lesson_2_4/debug_probe/debug_probe \
  demos/lesson_2_4/debug_probe/data/bad_missing_field.txt
```

## Крос-збірка для Raspberry Pi

```bash
cmake --preset aarch64-debug
cmake --build --preset aarch64-debug
```

ARM64 виконуваний файл:

```text
build/aarch64-debug/demos/lesson_2_4/debug_probe/debug_probe
```

## Віддалений запуск на Raspberry Pi

На `satelite` потрібен `gdbserver`, але не компілятор:

```bash
ssh satelite
sudo apt update
sudo apt install -y gdbserver
exit
```

Скопіювати виконуваний файл і дані:

```bash
ssh satelite 'rm -rf /tmp/debug_probe'
ssh satelite 'mkdir -p /tmp/debug_probe/data'
scp build/aarch64-debug/demos/lesson_2_4/debug_probe/debug_probe \
  satelite:/tmp/debug_probe/
scp demos/lesson_2_4/debug_probe/data/*.txt satelite:/tmp/debug_probe/data/
```

Запустити на цільовому пристрої:

```bash
ssh satelite
cd /tmp/debug_probe
chmod +x debug_probe
./debug_probe data/good.txt
gdbserver :1234 ./debug_probe data/bad_missing_field.txt
```

Підключитись з комп'ютера/devcontainer. Використовується локальний
крос-скомпільований виконуваний файл з debug-символами:

```bash
gdb-multiarch build/aarch64-debug/demos/lesson_2_4/debug_probe/debug_probe
```

Всередині GDB:

```text
target remote satelite:1234
continue
bt
frame 0
print text
quit
```

## Core dump

Локально:

```bash
ulimit -c unlimited
./build/debug/demos/lesson_2_4/debug_probe/debug_probe \
  demos/lesson_2_4/debug_probe/data/bad_missing_field.txt
gdb ./build/debug/demos/lesson_2_4/debug_probe/debug_probe core
```

На `satelite`:

```bash
ssh satelite
cd /tmp/debug_probe
ulimit -c unlimited
./debug_probe data/bad_missing_field.txt
ls -lh core*
```

Якщо core забрав systemd-coredump:

```bash
coredumpctl list debug_probe
coredumpctl dump debug_probe > /tmp/debug_probe/debug_probe.core
```

Скопіювати core на комп'ютер і відкрити через локальний ARM64 виконуваний файл
з debug-символами:

```bash
scp 'satelite:/tmp/debug_probe/core*' /tmp/
CORE_FILE=$(ls -t /tmp/core* | head -1)
gdb-multiarch build/aarch64-debug/demos/lesson_2_4/debug_probe/debug_probe \
  "$CORE_FILE"
```

Альтернативно, якщо core було експортовано через `coredumpctl dump`:

```bash
scp satelite:/tmp/debug_probe/debug_probe.core /tmp/debug_probe.core
gdb-multiarch build/aarch64-debug/demos/lesson_2_4/debug_probe/debug_probe \
  /tmp/debug_probe.core
```
