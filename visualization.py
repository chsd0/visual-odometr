import pandas as pd
import matplotlib.pyplot as plt
import os

# --- НАСТРОЙКИ ---
TRAJECTORY_FILE = 'trajectory_output.csv' 

# --- ОСНОВНОЙ КОД ---

# Загружаем данные
if not os.path.exists(TRAJECTORY_FILE):
    print(f"Ошибка: файл '{TRAJECTORY_FILE}' не найден.")
    print("Убедитесь, что он лежит в той же папке, что и скрипт, или укажите правильный путь.")
    exit()

df = pd.read_csv(TRAJECTORY_FILE)
print("Файл с траекторией успешно загружен. Строим графики...")

# --- 2. 3D-график траектории ---
# Для 3D-графика нужна специальная директива
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# Рисуем саму траекторию
ax.plot(df['X_m'], df['Y_m'], df['Z_m'], marker='.', linestyle='-', markersize=2, label='Траектория дрона')
# Отмечаем старт и финиш
ax.scatter(df['X_m'].iloc[0], df['Y_m'].iloc[0], df['Z_m'].iloc[0], s=150, c='green', label='Старт')
ax.scatter(df['X_m'].iloc[-1], df['Y_m'].iloc[-1], df['Z_m'].iloc[-1], s=150, c='red', label='Финиш')

ax.set_title('3D Траектория движения')
ax.set_xlabel('X (м)')
ax.set_ylabel('Y (м)')
ax.set_zlabel('Высота Z (м)')
ax.legend()
x_min, x_max = df['X_m'].min(), df['X_m'].max()
y_min, y_max = df['Y_m'].min(), df['Y_m'].max()
z_min, z_max = df['Z_m'].min(), df['Z_m'].max()
x_range, y_range, z_range = x_max - x_min, y_max - y_min, z_max - z_min
max_range = max(x_range, y_range, z_range)
mid_x, mid_y, mid_z = (x_max+x_min)/2, (y_max+y_min)/2, (z_max+z_min)/2
ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)

plt.show()