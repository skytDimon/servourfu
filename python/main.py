import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from rtlsdr import RtlSdr
import sys

# ==========================================
# НАСТРОЙКИ СТЕНДА
# ==========================================
SERIAL_PORT = "COM3"
BAUD_RATE = 115200
FREQ_MIN = 90.0e6
FREQ_MAX = 110.0e6
SAMPLE_RATE = 2.048e6
DEFAULT_GAIN = 10.0

NUM_SAMPLES = 8192
AVERAGE_COUNT = 10
NARROW_BAND_HZ = 200000
SDR_RETRIES = 3
POWER_FLOOR_DB = -120.0
SCAN_OVERALL_TIMEOUT_S = 600
FREQ_SWEEP_STEP_HZ = 2.0e6
FREQ_SWEEP_AVG = 3
# ==========================================


def read_samples_safe(sdr, num_samples, retries=SDR_RETRIES):
    for attempt in range(retries):
        try:
            return sdr.read_samples(num_samples)
        except Exception:
            if attempt < retries - 1:
                time.sleep(0.5)
    return None


def compute_psd(samples, sample_rate):
    N = len(samples)
    window = np.hanning(N)
    windowed = samples * window
    fft_result = np.fft.fftshift(np.fft.fft(windowed))
    psd = np.abs(fft_result) ** 2 / (N * np.sum(window**2))
    freqs = np.fft.fftshift(np.fft.fftfreq(N, 1.0 / sample_rate))
    return freqs, psd


def get_signal_power_and_snr(sdr):
    powers_signal = []
    powers_noise = []
    _ = read_samples_safe(sdr, NUM_SAMPLES)

    for _ in range(AVERAGE_COUNT):
        samples = read_samples_safe(sdr, NUM_SAMPLES)
        if samples is None:
            continue

        freqs, psd = compute_psd(samples, sdr.sample_rate)
        mask_signal = np.abs(freqs) <= NARROW_BAND_HZ / 2

        if np.any(mask_signal):
            power_signal = np.mean(psd[mask_signal])
            if power_signal > 0:
                powers_signal.append(10 * np.log10(power_signal))

        far_mask = np.abs(freqs) > 500000
        if np.any(far_mask):
            power_noise = np.median(psd[far_mask])
            if power_noise > 0:
                powers_noise.append(10 * np.log10(power_noise))

    if len(powers_signal) == 0:
        return POWER_FLOOR_DB, POWER_FLOOR_DB - 10

    sig = np.median(powers_signal)
    noise = np.median(powers_noise) if len(powers_noise) > 0 else sig - 10
    snr = sig - noise

    return sig, snr


def find_signal_frequency(sdr):
    print(
        f"\nПоиск источника в диапазоне {FREQ_MIN / 1e6:.1f}–{FREQ_MAX / 1e6:.1f} МГц..."
    )
    print("-" * 50)

    sweep_freqs = np.arange(FREQ_MIN, FREQ_MAX + 1, FREQ_SWEEP_STEP_HZ)
    all_peak_freqs = []
    all_peak_powers = []

    for fc in sweep_freqs:
        try:
            sdr.center_freq = float(fc)
            time.sleep(0.3)
        except Exception:
            continue

        _ = read_samples_safe(sdr, NUM_SAMPLES)
        samples = read_samples_safe(sdr, NUM_SAMPLES)
        if samples is None:
            continue

        freqs, psd = compute_psd(samples, sdr.sample_rate)
        abs_freqs = fc + freqs

        in_band = (abs_freqs >= FREQ_MIN) & (abs_freqs <= FREQ_MAX)
        if not np.any(in_band):
            continue

        psd_db = 10 * np.log10(psd[in_band] + 1e-20)
        abs_in_band = abs_freqs[in_band]

        peak_idx = np.argmax(psd_db)
        peak_freq = abs_in_band[peak_idx]
        peak_power = psd_db[peak_idx]

        all_peak_freqs.append(peak_freq)
        all_peak_powers.append(peak_power)

        bar = "#" * max(1, int((peak_power + 120) / 3))
        print(
            f"  {fc / 1e6:6.1f} МГц → пик {peak_freq / 1e6:7.3f} МГц  {peak_power:6.1f} dB  {bar}"
        )

    if len(all_peak_powers) == 0:
        print("Сигнал не найден! Используем центр диапазона (100 МГц).")
        return 100.0e6

    best_idx = int(np.argmax(all_peak_powers))
    found_freq = all_peak_freqs[best_idx]
    found_power = all_peak_powers[best_idx]

    print("-" * 50)
    print(f"  НАЙДЕН СИГНАЛ: {found_freq / 1e6:.3f} МГц ({found_power:.1f} dB)")

    print(f"\nУточнение частоты вокруг {found_freq / 1e6:.3f} МГц...")
    fine_step = 0.5e6
    fine_range = 2.0e6
    fine_freqs = np.arange(
        found_freq - fine_range, found_freq + fine_range + 1, fine_step
    )
    fine_freqs = fine_freqs[(fine_freqs >= FREQ_MIN) & (fine_freqs <= FREQ_MAX)]

    best_fine_freq = found_freq
    best_fine_power = found_power

    for fc in fine_freqs:
        try:
            sdr.center_freq = float(fc)
            time.sleep(0.3)
        except Exception:
            continue

        powers = []
        last_peak_freq = fc
        _ = read_samples_safe(sdr, NUM_SAMPLES)
        for _ in range(FREQ_SWEEP_AVG):
            samples = read_samples_safe(sdr, NUM_SAMPLES)
            if samples is None:
                continue
            freqs, psd = compute_psd(samples, sdr.sample_rate)
            abs_freqs = fc + freqs
            in_band = (abs_freqs >= FREQ_MIN) & (abs_freqs <= FREQ_MAX)
            if np.any(in_band):
                psd_db = 10 * np.log10(psd[in_band] + 1e-20)
                powers.append(np.max(psd_db))
                peak_f_idx = int(np.argmax(psd_db))
                last_peak_freq = abs_freqs[in_band][peak_f_idx]

        if powers:
            avg_power = np.median(powers)
            if avg_power > best_fine_power:
                best_fine_power = avg_power
                best_fine_freq = last_peak_freq

    print(f"  УТОЧНЕНО: {best_fine_freq / 1e6:.3f} МГц ({best_fine_power:.1f} dB)")
    print("=" * 50)
    return best_fine_freq


def find_safe_gain(sdr, start_gain=30.0):
    print("Подбор безопасного усиления (проверка перегрузки ADC)...")
    gain_steps = [
        49.6,
        44.5,
        40.2,
        37.2,
        33.8,
        29.7,
        25.4,
        22.9,
        20.7,
        19.6,
        16.6,
        15.7,
        14.4,
        12.5,
        8.7,
        7.7,
        3.7,
        2.7,
        1.4,
        0.9,
        0.0,
    ]
    gain_steps = [g for g in gain_steps if g <= start_gain]

    for gain in gain_steps:
        try:
            sdr.gain = float(gain)
            time.sleep(0.5)
            samples = sdr.read_samples(2048)
            peak = np.max(np.abs(samples))
            if peak < 0.95:
                print(f"  Gain {gain:5.1f} dB — OK (пик {peak:.3f})")
                return gain
            else:
                print(f"  Gain {gain:5.1f} dB — перегрузка (пик {peak:.3f})")
        except Exception:
            continue

    sdr.gain = 0.0
    print("  Использован минимальный gain 0 dB")
    return 0.0


def wait_for_arduino_line(ser, prefix, timeout_s=60):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if ser.in_waiting > 0:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line.startswith(prefix):
                return line
            print(f"  [Arduino] {line}")
        time.sleep(0.05)
    return None


def run_calibration(ser):
    print("\n" + "=" * 55)
    print("  КАЛИБРОВКА ВРАЩЕНИЯ")
    print("=" * 55)
    print("  1. Поставьте антенну на отметку 0° (по лимбу)")
    print("  2. Нажмите ENTER — антенна сделает оборот")
    print("  3. Скажите где она остановилась")
    print("=" * 55)

    while True:
        input("  Антенна на 0°? Нажмите ENTER... ")

        ser.reset_input_buffer()
        ser.write(b"C")

        result = wait_for_arduino_line(ser, "CAL_DONE:", timeout_s=30)
        if result is None:
            print("  ОШИБКА: Arduino не ответила!")
            return

        total_ms = int(result.split(":")[1])
        print(f"  Ардуино: оборот занял {total_ms} мс")

        print("\n  Где сейчас антенна? (введите градусы от 0):")
        print("    Если перешла через 0 — введите >360 (например 380)")
        print("    Если не дошла — введите <360 (например 340)")

        while True:
            try:
                actual_deg = float(input("    Фактический угол: "))
                break
            except ValueError:
                print("    Введите число!")

        new_time = int(round(10.0 * total_ms / actual_deg))
        print(
            f"\n  Расчёт: {total_ms} мс / {actual_deg}° = {total_ms / actual_deg:.2f} мс/град"
        )
        print(f"  Новое timeFor10Deg = {new_time} мс")

        confirm = input(f"  Применить {new_time} мс? (Y/n): ").strip().upper()
        if confirm != "N":
            ser.reset_input_buffer()
            cmd = f"T{new_time}\n".encode("utf-8")
            ser.write(cmd)

            confirm_result = wait_for_arduino_line(ser, "TIME_SET:", timeout_s=5)
            if confirm_result and confirm_result.startswith("TIME_SET:"):
                confirmed_val = confirm_result.split(":")[1]
                print(f"  Ардуино подтвердила: timeFor10Deg = {confirmed_val} мс")
            else:
                print("  ОШИБКА: Arduino не подтвердила!")
                return

        retry = input("\n  Повторить калибровку? (y/N): ").strip().lower()
        if retry != "y":
            break

    print("  Калибровка завершена.\n")


def run_scan(ser, sdr, label):
    angles = []
    powers = []
    snrs = []

    ser.reset_input_buffer()
    ser.write(b"S")
    print(f"\n{label}")
    print("Команда отправлена. Ожидание SCAN_START...\n")

    scan_started = False
    deadline = time.time() + 10
    while time.time() < deadline:
        if ser.in_waiting > 0:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line == "SCAN_START":
                scan_started = True
                break
        time.sleep(0.05)

    if not scan_started:
        print("ОШИБКА: Arduino не ответила SCAN_START!")
        return angles, powers, snrs

    print("Сканирование начато.\n")

    scan_t0 = time.time()
    try:
        while True:
            if time.time() - scan_t0 > SCAN_OVERALL_TIMEOUT_S:
                print("\nОШИБКА: общий таймаут сканирования!")
                break

            if ser.in_waiting > 0:
                line = ser.readline().decode("utf-8", errors="ignore").strip()

                if line.startswith("READY:"):
                    try:
                        angle = int(line.split(":")[1].strip())
                    except (ValueError, IndexError):
                        print(f"  Ошибка парсинга угла: '{line}'")
                        ser.write(b"K")
                        continue

                    print(f"[{angle:03d}°] Ожидание 2 секунды...", end="\r")
                    time.sleep(2)

                    print(f"[{angle:03d}°] Замер эфира...       ", end=" ")
                    power, snr = get_signal_power_and_snr(sdr)

                    angles.append(angle)
                    powers.append(power)
                    snrs.append(snr)

                    print(f"Мощность: {power:.2f} dB  SNR: {snr:.1f} dB")
                    ser.write(b"K")

                elif line == "SCAN_FINISHED":
                    print("\nСканирование завершено!")
                    break

                elif line == "SCAN_TIMEOUT":
                    print("\nВНИМАНИЕ: Arduino сообщила о таймауте!")
                    break

                else:
                    print(f"  [Arduino] {line}")

            else:
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nОстановлено пользователем.")

    return angles, powers, snrs


def find_dip(angles_deg, powers_db, snrs_db):
    min_idx = int(np.argmin(powers_db))
    return angles_deg[min_idx], powers_db[min_idx], snrs_db[min_idx]


def find_peak_with_snr(angles_deg, powers_db, snrs_db):
    max_idx = int(np.argmax(powers_db))
    return angles_deg[max_idx], powers_db[max_idx], snrs_db[max_idx]


def print_methodology():
    print("\n" + "=" * 65)
    print("  МЕТОДИКА ПЕЛЕНГАЦИИ")
    print("=" * 65)
    print("""
  Метод: амплитудная пеленгация по минимуму (деструктивная
  интерференция) с уголковым отражателем

  Физика:
  - При отражении от проводящей стенки фаза волны сдвигается
    на 180 градусов
  - Антенна на расстоянии ~5 см от стенок отражателя
  - Длина волны на 100 МГц: лямбда = 300 см
  - Разность хода прямой и отражённой волны: ~10 см
  - Итоговый фазовый сдвиг: 180 + ~12 = ~192 градуса
    => почти противофаза => деструктивная интерференция

  Принцип определения направления:
  1. Скан БЕЗ отражателя: штыревая антенна всенаправленная,
     уровень сигнала примерно равномерен по углам
  2. Скан С отражателем: когда открытая сторона отражателя
     направлена на передатчик, прямой и отражённый сигналы
     складываются в противофазе => МИНИМУМ мощности (провал)
  3. Направление на источник = угол МИНИМУМА мощности
     в скане с отражателем

  Почему минимум, а не максимум:
  - Деструктивная интерференция даёт узкий глубокий провал
  - Затенение (закрытая сторона) — широкое и размытое
  - Провал чётче, чем максимум => выше точность пеленгации

  Оборудование:
  - RTL-SDR V3: приёмник, PSD через FFT, полоса +/-100 кГц
  - Arduino Uno: сервопривод 360 град, шаг 10 град
  - Уголковый отражатель: алюминий + металлизированный скотч
  - LED: индикация замера, кнопка Пуск/Сброс
""")


def main():
    print_methodology()

    print("Инициализация RTL-SDR...")
    try:
        sdr = RtlSdr()
        sdr.sample_rate = SAMPLE_RATE
        sdr.center_freq = 100.0e6
        sdr.gain = DEFAULT_GAIN
    except Exception as e:
        print(f"Ошибка RTL-SDR: {e}")
        sys.exit(1)

    chosen_gain = find_safe_gain(sdr, start_gain=30.0)
    print(f"Усиление установлено: {chosen_gain:.1f} dB\n")

    print(f"Подключение к Arduino ({SERIAL_PORT})...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=3)
        time.sleep(2)
    except Exception as e:
        print(f"Ошибка COM-порта: {e}")
        sdr.close()
        sys.exit(1)

    ready = False
    deadline = time.time() + 5
    while time.time() < deadline:
        if ser.in_waiting > 0:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line == "SYSTEM_READY":
                ready = True
                break
        time.sleep(0.1)

    if ready:
        print("Arduino: SYSTEM_READY получен.")
    else:
        print("Внимание: SYSTEM_READY не получен (возможно уже загружена).")

    print("\n" + "=" * 65)
    print("  РАДАР ГОТОВ. ОЖИДАНИЕ КОМАНДЫ...")
    print("=" * 65)
    print("  ENTER    — найти частоту и начать сканирование")
    print("  C+ENTER  — калибровка (расчёт timeFor10Deg)")
    print("  H+ENTER  — возврат антенны на 0°")
    print("  F+ENTER  — повторный поиск частоты")
    print("=" * 65)

    found_freq = None

    while True:
        user_input = input("\n>>> Команда: ").strip().upper()

        if user_input == "C":
            run_calibration(ser)
            continue
        elif user_input == "H":
            ser.reset_input_buffer()
            ser.write(b"H")
            result = wait_for_arduino_line(ser, "HOME_DONE", timeout_s=15)
            if result:
                print(f"  Ардуино: {result}")
            else:
                print("  Таймаут при возврате домой.")
            continue
        elif user_input == "F":
            found_freq = find_signal_frequency(sdr)
            print(f"Частота зафиксирована: {found_freq / 1e6:.3f} МГц")
            continue
        elif user_input == "":
            break
        else:
            print("Неизвестная команда. ENTER / C / H / F.")
            continue

    if found_freq is None:
        found_freq = find_signal_frequency(sdr)

    sdr.center_freq = float(found_freq)
    print(f"\nЧастота для сканирования: {found_freq / 1e6:.3f} МГц")

    # ==================================================
    # СКАН
    # ==================================================
    has_reflector = input("\nУстановлен отражатель? (y/N): ").strip().lower() == "y"
    label = (
        "СКАНИРОВАНИЕ С ОТРАЖАТЕЛЕМ" if has_reflector else "СКАНИРОВАНИЕ БЕЗ ОТРАЖАТЕЛЯ"
    )

    angles, powers, snrs = run_scan(ser, sdr, label)

    # Возврат домой
    ser.reset_input_buffer()
    ser.write(b"H")
    wait_for_arduino_line(ser, "HOME_DONE", timeout_s=15)

    ser.close()
    sdr.close()

    if len(angles) == 0:
        print("ОШИБКА: Нет данных!")
        sys.exit(1)

    # ==================================================
    # АНАЛИЗ
    # ==================================================
    max_angle, max_power, max_snr = find_peak_with_snr(angles, powers, snrs)
    min_angle, min_power, min_snr = find_dip(angles, powers, snrs)

    print("\n" + "=" * 65)
    print("  РЕЗУЛЬТАТЫ СКАНИРОВАНИЯ")
    print("=" * 65)
    print(f"  Частота:  {found_freq / 1e6:.3f} МГц")
    print(f"  Усиление: {chosen_gain:.1f} dB")
    print(f"  Отражатель: {'ДА' if has_reflector else 'НЕТ'}")
    print()

    if has_reflector:
        depth = max_power - min_power
        print(f"  Максимум: {max_angle}°  {max_power:.2f} dB  SNR {max_snr:.1f} dB")
        print(f"  МИНИМУМ:  {min_angle}°  {min_power:.2f} dB  SNR {min_snr:.1f} dB")
        print(f"  Глубина провала: {depth:.2f} dB")
        print()
        print(f"  >>> НАПРАВЛЕНИЕ НА ИСТОЧНИК: {min_angle}° <<<")
        print(f"  (провал = деструктивная интерференция: отражатель")
        print(f"   открытой стороной к передатчику)")
        final_angle = min_angle
    else:
        print(f"  Максимум: {max_angle}°  {max_power:.2f} dB  SNR {max_snr:.1f} dB")
        print(f"  Минимум:  {min_angle}°  {min_power:.2f} dB  SNR {min_snr:.1f} dB")
        print()
        print(f"  Без отражателя диаграмма примерно равномерна.")
        print(f"  Для определения направления запустите скан ещё раз")
        print(f"  с отражателем — направление = МИНИМУМ мощности.")
        final_angle = max_angle

    print("=" * 65)

    plot_polar(angles, powers, found_freq, final_angle, has_reflector)


def plot_polar(angles_deg, powers_db, found_freq, final_angle, has_reflector):
    print("Построение графика...")

    angles_rad = np.deg2rad(angles_deg)
    angles_rad_c = np.append(angles_rad, angles_rad[0])
    powers_db_c = np.append(powers_db, powers_db[0])

    plt.figure(figsize=(8, 8))
    ax = plt.subplot(111, projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)

    color = "red" if has_reflector else "blue"
    label_str = "С отражателем" if has_reflector else "Без отражателя"

    ax.plot(
        angles_rad_c, powers_db_c, color=color, linewidth=2, marker="o", markersize=3
    )
    ax.fill(angles_rad_c, powers_db_c, color=color, alpha=0.2)

    max_idx = int(np.argmax(powers_db))
    max_a = angles_deg[max_idx]
    max_p = powers_db[max_idx]
    ax.annotate(
        f"max {max_a}°",
        xy=(np.deg2rad(max_a), max_p),
        xytext=(np.deg2rad(max_a), max_p + 5),
        arrowprops=dict(facecolor="green", shrink=0.05),
        horizontalalignment="center",
        fontsize=10,
        color="green",
    )

    min_idx = int(np.argmin(powers_db))
    min_a = angles_deg[min_idx]
    min_p = powers_db[min_idx]
    ax.annotate(
        f"ЦЕЛЬ: {min_a}°" if has_reflector else f"min {min_a}°",
        xy=(np.deg2rad(min_a), min_p),
        xytext=(np.deg2rad(min_a), min_p - 5),
        arrowprops=dict(
            facecolor="red" if has_reflector else "gray",
            shrink=0.05,
            width=2 if has_reflector else 1,
        ),
        horizontalalignment="center",
        fontsize=13 if has_reflector else 10,
        fontweight="bold" if has_reflector else "normal",
        color="red" if has_reflector else "gray",
    )

    freq_str = f"{found_freq / 1e6:.3f} МГц"
    if has_reflector:
        ax.set_title(
            f"{label_str}\n{freq_str} → минимум = {final_angle}°",
            va="bottom",
            fontsize=12,
            fontweight="bold",
        )
    else:
        ax.set_title(
            f"{label_str}\n{freq_str}",
            va="bottom",
            fontsize=12,
            fontweight="bold",
        )

    plt.tight_layout()
    fname = f"pelenq_{'with' if has_reflector else 'without'}_reflector.png"
    plt.savefig(fname, dpi=150, bbox_inches="tight")
    print(f"  График сохранён: {fname}")
    plt.show()


if __name__ == "__main__":
    main()
