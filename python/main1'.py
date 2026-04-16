import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from rtlsdr import RtlSdr
import sys

# ==========================================
# ВАШИ НАСТРОЙКИ
# ==========================================
SERIAL_PORT = 'COM3'  # Укажите ваш COM-порт от Arduino!
BAUD_RATE = 115200
CENTER_FREQ = 100.00e6  # Точная частота целевого передатчика в Герцах
SAMPLE_RATE = 2.048e6
GAIN = 20  # Фиксированное усиление

NUM_SAMPLES = 8192
AVERAGE_COUNT = 10


# ==========================================

def get_signal_power(sdr):
    powers = []
    # Сбрасываем старый эфирный мусор
    _ = sdr.read_samples(NUM_SAMPLES)

    for _ in range(AVERAGE_COUNT):
        samples = sdr.read_samples(NUM_SAMPLES)
        power_linear = np.mean(np.abs(samples) ** 2)
        if power_linear > 0:
            power_db = 10 * np.log10(power_linear)
            powers.append(power_db)

    return np.median(powers)


def main():
    print("Инициализация RTL-SDR...")
    try:
        sdr = RtlSdr()
        sdr.sample_rate = SAMPLE_RATE
        sdr.center_freq = CENTER_FREQ
        sdr.gain = GAIN
    except Exception as e:
        print(f"Ошибка RTL-SDR: {e}")
        sys.exit(1)

    print(f"Подключение к Arduino на порту {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Пауза на инициализацию порта
    except Exception as e:
        print(f"Ошибка COM-порта: {e}")
        sdr.close()
        sys.exit(1)

    angles = []
    powers = []

    print("\n" + "=" * 45)
    print("РАДАР ГОТОВ. ОЖИДАНИЕ КОМАНДЫ...")
    print("=" * 45)

    input("\n>>> Нажмите [ENTER] для старта сканирования: ")
    ser.reset_input_buffer()

    ser.write(b'S')
    print("Команда отправлена. Начало приема данных...\n")

    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                if line.startswith("READY:"):
                    angle = int(line.split(":")[1])
                    print(f"[{angle:03d}°] Ожидание 2 секунды...", end="\r")

                    # === ТА САМАЯ ПАУЗА В 2 СЕКУНДЫ ===
                    time.sleep(2)

                    print(f"[{angle:03d}°] Замер эфира...       ", end=" ")
                    power = get_signal_power(sdr)

                    angles.append(angle)
                    powers.append(power)

                    print(f"Мощность: {power:.2f} dB")

                    # ПИНАЕМ АРДУИНО ЕХАТЬ ДАЛЬШЕ
                    ser.write(b'K')

                elif line == "SCAN_FINISHED":
                    print("\nСканирование успешно завершено!")
                    break

    except KeyboardInterrupt:
        print("\nОстановлено пользователем.")
    finally:
        ser.close()
        sdr.close()

    if len(angles) > 0:
        plot_polar_data(angles, powers)


def plot_polar_data(angles_deg, powers_db):
    print("Отрисовка диаграммы направленности...")
    angles_rad = np.deg2rad(angles_deg)

    # Замыкаем график для красоты
    angles_rad = np.append(angles_rad, angles_rad[0])
    powers_db = np.append(powers_db, powers_db[0])

    plt.figure(figsize=(8, 8))
    ax = plt.subplot(111, projection='polar')
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)  # По часовой стрелке

    ax.plot(angles_rad, powers_db, color='red', linewidth=2, linestyle='solid', marker='o', markersize=4)
    ax.fill(angles_rad, powers_db, color='red', alpha=0.3)

    ax.set_title("Пеленгация источника (Трек SDR)", va='bottom', fontsize=14, fontweight='bold')

    # Поиск пика сигнала
    max_power_idx = np.argmax(powers_db[:-1])
    max_angle = angles_deg[max_power_idx]
    max_val = powers_db[max_power_idx]

    ax.annotate(f'ЦЕЛЬ: {max_angle}°',
                xy=(np.deg2rad(max_angle), max_val),xytext=(np.deg2rad(max_angle), max_val + 5),
                arrowprops=dict(facecolor='black', shrink=0.05),
                horizontalalignment='center',
                fontsize=12, fontweight='bold')

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()