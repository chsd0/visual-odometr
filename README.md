# Алгоритм визуального одометра с вертикальной ориентацией камеры с учётом углов ориентации

## Требования

- Python 3.8+ (рекомендуется Python 3.12)
- pip

---

## Быстрый старт

### 1. Создайте и активируйте виртуальное окружение

```sh
# Для Linux/macOS:
python3 -m venv myenv
source myenv/bin/activate
# Для Windows:
python -m venv myenv
myenv\Scripts\activate
```

### 2. Установите зависимости

```sh
pip install -r requirements.txt
```

### 3. Подготовьте входные данные
В корне проекта должны находиться два файла:

* `flight_video.mp4` — видео с камеры дрона (направленной вниз)
* `imu_log.csv` — CSV-файл с логом IMU 

### 4. Запуск алгоритма 

```sh
# Для Linux/macOS:
python3 algorithm.py
# Для Windows:
python algorithm.py
```
В результате выполнения появится файл trajectory_output.csv с рассчитанной траекторией.

### 5. Визуализация траектории
Для построения 3D-графика траектории выполните:
```sh
# Для Linux/macOS:
python3 visualization.py
# Для Windows:
python visualization.py
```
Откроется окно с 3D-графиком движения дрона. Старт отмечен зелёной точкой, финиш — красной.

---

## Полная цепочка команд

Для Linux/macOS:

```sh
python3 -m venv myenv
source myenv/bin/activate

pip install -r requirements.txt

python3 algorithm.py
python3 visualization.py
```

Для Windows:

```sh
python -m venv myenv
myenv\Scripts\activate

pip install -r requirements.txt

python algorithm.py
python visualization.py
```

---

## Пример результата работы алгоритма

<img width="799" height="779" alt="visualization" src="https://github.com/user-attachments/assets/d26b28ed-301f-4e80-a271-b13a168198ae" />
