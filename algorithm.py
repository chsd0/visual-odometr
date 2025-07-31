import cv2
import pandas as pd
import numpy as np
import math
import os

# --- КОНСТАНТЫ И НАСТРОЙКИ ---

# Входные файлы
VIDEO_FILE = 'flight_video.mp4'
IMU_LOG_FILE = 'imu_log.csv'

# Выходной файл
OUTPUT_FILE = 'trajectory_output.csv'

# Константы для конвертации
FEET_TO_METERS = 0.3048

# Параметры для визуальной одометрии
FEATURE_DETECTOR = cv2.ORB_create(nfeatures=2000)
MATCHER = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
MIN_MATCH_COUNT = 10  # Минимальное количество хороших совпадений для расчета смещения

# Частота записи в выходной файл (в герцах)
OUTPUT_FREQUENCY_HZ = 5
OUTPUT_INTERVAL_MS = 1000 / OUTPUT_FREQUENCY_HZ

# --- ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ---

def find_closest_imu_data(df_imu, time_ms):
    """Находит в DataFrame IMU строку, наиболее близкую по времени к заданному."""
    # Используем `searchsorted` для быстрого поиска ближайшего индекса
    idx = df_imu['time(millisecond)'].searchsorted(time_ms)
    
    # Обрабатываем крайние случаи
    if idx == 0:
        return df_imu.iloc[0]
    if idx == len(df_imu):
        return df_imu.iloc[-1]
    
    # Сравниваем две ближайшие записи и выбираем самую близкую
    before = df_imu.iloc[idx - 1]
    after = df_imu.iloc[idx]
    if abs(after['time(millisecond)'] - time_ms) < abs(before['time(millisecond)'] - time_ms):
        return after
    else:
        return before

def get_homography_for_undistortion(pitch_rad, roll_rad, K):
    """
    Создает матрицу гомографии для "выравнивания" изображения, снятого под углом.
    Мы поворачиваем картинку так, как будто камера смотрела строго вниз.
    """
    # Матрицы поворота вокруг осей X (тангаж) и Y (крен)
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(pitch_rad), -np.sin(pitch_rad)],
                   [0, np.sin(pitch_rad), np.cos(pitch_rad)]])
    
    Ry = np.array([[np.cos(roll_rad), 0, np.sin(roll_rad)],
                   [0, 1, 0],
                   [-np.sin(roll_rad), 0, np.cos(roll_rad)]])
    
    # Общая матрица поворота
    R = Rx @ Ry
    
    # Матрица гомографии: H = K * R * K_inv
    H = K @ R @ np.linalg.inv(K)
    
    return H

# --- ОСНОВНОЙ АЛГОРИТМ ---

def main():
    print("Запуск алгоритма визуального одометра...")

    # 1. Загрузка данных
    if not os.path.exists(VIDEO_FILE):
        print(f"Ошибка: Видеофайл '{VIDEO_FILE}' не найден.")
        return
    if not os.path.exists(IMU_LOG_FILE):
        print(f"Ошибка: Файл лога IMU '{IMU_LOG_FILE}' не найден.")
        return
        
    cap = cv2.VideoCapture(VIDEO_FILE)
    imu_df = pd.read_csv(IMU_LOG_FILE)
    
    # Проверка, что колонки существуют
    required_columns = ['time(millisecond)', ' pitch(degrees)', ' roll(degrees)', 'height_above_takeoff(feet)']
    for col in required_columns:
        if col not in imu_df.columns:
            print(f"Ошибка: В файле лога отсутствует необходимая колонка '{col}'.")
            return

    # 2. Подготовка
    # Получаем параметры видео
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Видео: {width}x{height} @ {fps:.2f} FPS")

    # Предполагаемые внутренние параметры камеры (камера не калибрована)
    # Для простоты, фокусное расстояние примем равным ширине кадра.
    fx = fy = width
    cx, cy = width / 2, height / 2
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    
    # Инициализация переменных состояния
    prev_frame_gray = None
    prev_kpts = None
    prev_descs = None
    
    total_x, total_y = 0.0, 0.0 # Координаты на плоскости XY
    
    results = []
    last_output_time_ms = -OUTPUT_INTERVAL_MS # Для немедленной записи первого кадра

    frame_count = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        current_time_ms = cap.get(cv2.CAP_PROP_POS_MSEC)
        
        # 3. Синхронизация с IMU
        imu_data = find_closest_imu_data(imu_df, current_time_ms)
        pitch_deg = imu_data[' pitch(degrees)']
        roll_deg = imu_data[' roll(degrees)']
        height_ft = imu_data['height_above_takeoff(feet)']
        
        # Конвертация в метры и радианы
        height_m = height_ft * FEET_TO_METERS
        pitch_rad = np.radians(pitch_deg)
        roll_rad = np.radians(roll_deg)

        # 4. Коррекция перспективы
        H_undistort = get_homography_for_undistortion(pitch_rad, roll_rad, K)
        frame_undistorted = cv2.warpPerspective(frame, H_undistort, (width, height))
        
        # Переводим в оттенки серого
        frame_gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)

        # 5. Визуальная одометрия
        if prev_frame_gray is None:
            # Первый кадр: просто находим точки и сохраняем
            prev_kpts, prev_descs = FEATURE_DETECTOR.detectAndCompute(frame_gray, None)
            prev_frame_gray = frame_gray
            prev_height_m = height_m
            frame_count += 1
            continue

        # Находим ключевые точки и дескрипторы на текущем кадре
        current_kpts, current_descs = FEATURE_DETECTOR.detectAndCompute(frame_gray, None)

        if prev_descs is None or current_descs is None or len(prev_descs) < MIN_MATCH_COUNT or len(current_descs) < MIN_MATCH_COUNT:
            # Если точек мало, пропускаем кадр
            prev_kpts, prev_descs = current_kpts, current_descs
            prev_frame_gray = frame_gray
            prev_height_m = height_m
            frame_count += 1
            continue
            
        # Сопоставление точек (matching)
        matches = MATCHER.knnMatch(prev_descs, current_descs, k=2)

        # Фильтр хороших совпадений по тесту Лоу
        good_matches = []
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                good_matches.append(m)

        dX_m, dY_m = 0.0, 0.0
        
        if len(good_matches) > MIN_MATCH_COUNT:
            # Получаем координаты совпавших точек
            prev_pts = np.float32([prev_kpts[m.queryIdx].pt for m in good_matches]).reshape(-1, 2)
            curr_pts = np.float32([current_kpts[m.trainIdx].pt for m in good_matches]).reshape(-1, 2)
            
            # Рассчитываем среднее смещение в пикселях
            pixel_displacements = curr_pts - prev_pts
            avg_pixel_displacement = np.mean(pixel_displacements, axis=0)
            
            dx_px, dy_px = avg_pixel_displacement[0], avg_pixel_displacement[1]
            
            # 6. Масштабирование
            # Используем среднюю высоту между двумя кадрами для большей точности
            avg_height = (height_m + prev_height_m) / 2.0
            if avg_height < 0.5: # Избегаем деления на ноль или слишком малые значения
                avg_height = 0.5

            # --- КОРРЕКЦИЯ МАСШТАБА ---
            # ПОДБОР КОЭФИЦИЕНТА
            SCALE_CORRECTION_FACTOR = 10.0 

            # Переводим пиксели в метры. Умножаем на наш калибровочный коэффициент.
            # Знак "-" нужен, т.к. движение камеры вправо вызывает смещение точек на изображении влево
            dX_m = -dx_px * (avg_height / fx) * SCALE_CORRECTION_FACTOR
            dY_m = -dy_px * (avg_height / fy) * SCALE_CORRECTION_FACTOR # "Вниз" по изображению - это движение "вперед" для дрона

        # 7. Интегрирование пути
        total_x += dX_m
        total_y += dY_m
        current_z = height_m # Текущую высоту Z берем напрямую

        # Вычисляем параметры вектора перемещения
        displacement_length = math.sqrt(dX_m**2 + dY_m**2)
        displacement_angle_rad = math.atan2(dY_m, dX_m)
        displacement_angle_deg = math.degrees(displacement_angle_rad)

        # 8. Запись результата с заданной частотой
        if current_time_ms - last_output_time_ms >= OUTPUT_INTERVAL_MS:
            results.append({
                'time_ms': current_time_ms,
                'X_m': total_x,
                'Y_m': total_y,
                'Z_m': current_z,
                'disp_length_m': displacement_length,
                'disp_angle_deg': displacement_angle_deg
            })
            last_output_time_ms = current_time_ms

        # Обновляем состояние для следующей итерации
        prev_kpts, prev_descs = current_kpts, current_descs
        prev_frame_gray = frame_gray
        prev_height_m = height_m
        
        frame_count += 1
        if frame_count % 100 == 0:
            print(f"Обработано кадров: {frame_count}, Время в видео: {current_time_ms/1000:.2f}с, Координаты: (X={total_x:.2f}, Y={total_y:.2f}, Z={current_z:.2f})")

    # 9. Сохранение итогового файла
    print("Сохранение результатов в файл...")
    output_df = pd.DataFrame(results)
    output_df.to_csv(OUTPUT_FILE, index=False, float_format='%.4f')

    # Очистка
    cap.release()
    cv2.destroyAllWindows()
    print(f"Готово! Маршрут сохранен в '{OUTPUT_FILE}'.")

if __name__ == '__main__':
    main()