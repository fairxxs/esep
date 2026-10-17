"""
Симуляция энергопотребления поезда: сталь (180 т) против алюминия (145 т).
Хакатон Stadler Kazakhstan. Запуск: Google Colab или python3 train_energy_sim.py
Нужно: numpy, matplotlib (в Colab уже есть).

Модель (упрощённая, НЕ замер):
  - точечная масса, ровный путь (без уклонов);
  - сопротивление движению: качение (зависит от массы) + аэродинамика (не зависит от массы);
  - разгон с ограничением по силе тяги и мощности, движение с постоянной скоростью, торможение;
  - рекуперация: часть энергии торможения возвращается в сеть;
  - вспомогательные нагрузки (отопление, кондиционер) НЕ учитываются — они не зависят от материала кузова.
Все параметры — допущения, их можно менять в блоке НАСТРОЙКИ.
"""
import numpy as np
import matplotlib.pyplot as plt

# ---------------- НАСТРОЙКИ ----------------
ROT = 1.08            # коэффициент вращающихся масс
A_MAX = 0.8           # макс. ускорение при разгоне, м/с²
B_DEC = 0.8           # замедление при торможении, м/с²
P_MAX = 2.6e6         # макс. мощность тяги, Вт
CDA = 10.0            # аэродинамика (Cd*A), м²
RHO = 1.2             # плотность воздуха, кг/м³ (при -30 °C ~1.45 — сопротивление выше)
ROLL = 12.0           # сопротивление качению, Н на тонну
ROLL_V = 0.1          # добавка к качению, Н на тонну на км/ч
ETA_TR = 0.85         # КПД тяги (сеть -> колесо)
ETA_REG = 0.6         # доля энергии торможения, возвращаемая в сеть (0 = без рекуперации)
ANNUAL_KM = 245_000   # годовой пробег (из кейса)
DT = 0.5              # шаг расчёта, с

SCENARIOS = {
    # название: (длина маршрута км, число остановок, макс. скорость км/ч)
    "Кейс: региональный (Польша)": (90, 15, 140),
    "Казахстан: дальний маршрут":   (1000, 8, 120),
}
MASS_ALU, MASS_STEEL = 145, 180   # тонны
# -------------------------------------------


def resistance(m_t, v):
    """Сопротивление движению, Н. v в м/с."""
    v_kmh = v * 3.6
    return m_t * (ROLL + ROLL_V * v_kmh) + 0.5 * RHO * CDA * v ** 2


def run_segment(m_t, dist_m, v_max):
    """Один перегон: разгон -> движение -> торможение до остановки. Возвращает энергию из сети, Дж."""
    m = m_t * 1000 * ROT
    v, x, e_in, e_back = 0.0, 0.0, 0.0, 0.0
    braking = False
    while x < dist_m:
        # тормозной путь от текущей скорости
        brake_dist = v ** 2 / (2 * B_DEC)
        if not braking and dist_m - x <= brake_dist:
            braking = True
        r = resistance(m_t, v)
        if braking:
            f_brake = m * B_DEC - r                       # сколько должен дать тормоз
            if f_brake > 0:
                e_back += f_brake * v * DT * ETA_REG      # рекуперация
            v = max(v - B_DEC * DT, 0.0)
            if v == 0.0:
                break
        else:
            if v < v_max:
                f = min(m * A_MAX + r, P_MAX / max(v, 0.1))
            else:
                f = r                                     # держим скорость
            a = (f - r) / m
            e_in += f * v * DT / ETA_TR
            v = min(v + a * DT, v_max)
        x += v * DT
    return e_in - e_back


def route_energy_kwh_per_km(m_t, length_km, stops, v_kmh):
    seg = length_km * 1000 / (stops + 1)
    e = run_segment(m_t, seg, v_kmh / 3.6) * (stops + 1)
    return e / 3.6e6 / length_km


def main():
    print(f"{'Сценарий':32} {'Алюм.':>8} {'Сталь':>8} {'Разница':>9} {'Экономия/год':>14}")
    print(f"{'':32} {'кВт·ч/км':>8} {'кВт·ч/км':>8} {'%':>9} {'МВт·ч':>14}")
    for name, (L, n, vmax) in SCENARIOS.items():
        ea = route_energy_kwh_per_km(MASS_ALU, L, n, vmax)
        es = route_energy_kwh_per_km(MASS_STEEL, L, n, vmax)
        save = (es - ea) * ANNUAL_KM / 1000
        print(f"{name:32} {ea:8.2f} {es:8.2f} {100*(es-ea)/es:8.1f}% {save:14.0f}")
    print("Для сравнения: в кейсе Stadler заявлено 357 МВт·ч/год.\n")

    # График: как экономия зависит от расстояния между остановками
    spacing = np.array([3, 5, 8, 12, 20, 35, 60, 100, 150])
    saves = []
    for s in spacing:
        L = s * 20
        ea = route_energy_kwh_per_km(MASS_ALU, L, 19, 140)
        es = route_energy_kwh_per_km(MASS_STEEL, L, 19, 140)
        saves.append((es - ea) * ANNUAL_KM / 1000)
    plt.figure(figsize=(8, 4.5))
    plt.plot(spacing, saves, marker="o")
    plt.axhline(357, linestyle="--", label="Заявлено в кейсе: 357 МВт·ч")
    plt.xscale("log")
    plt.xlabel("Среднее расстояние между остановками, км")
    plt.ylabel("Экономия алюминия, МВт·ч в год")
    plt.title("Чем реже остановки, тем меньше выгода от лёгкого кузова (модель)")
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig("savings_vs_stop_spacing.png", dpi=150)
    print("График сохранён: savings_vs_stop_spacing.png")

    # Таблица чувствительности к рекуперации
    print("\nВлияние рекуперации (региональный сценарий):")
    global ETA_REG
    base = ETA_REG
    for r in (0.0, 0.3, 0.6):
        ETA_REG = r
        L, n, vmax = SCENARIOS["Кейс: региональный (Польша)"]
        ea = route_energy_kwh_per_km(MASS_ALU, L, n, vmax)
        es = route_energy_kwh_per_km(MASS_STEEL, L, n, vmax)
        print(f"  рекуперация {int(r*100):3d}%: экономия {(es-ea)*ANNUAL_KM/1000:6.0f} МВт·ч/год")
    ETA_REG = base


if __name__ == "__main__":
    main()
