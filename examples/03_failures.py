"""Пример 3: отказ двигателя в полёте и авторотация; топливо кончилось.
Показывает, чего ждать от пилота: держать обороты ротора шагом, держать курс педалями
(в авторотации реактивного момента нет, хвостовой винт продолжает толкать), выровнять
вертолёт у земли и притереть шагом перед самой землёй.
Запуск: python3 03_failures.py [light_piston|medium_turbine|heavy_twin]"""
import sys
from common import Helicopter, Sim, Autopilot, cold_start, clamp

LOW_FUEL = {"light_piston": 0.6}        # кг: столько, чтобы топливо кончилось за минуты
FLARE_H = 60.0      # м: с этой высоты гасим скорость
CUSHION_H = 30.0    # м: с этой высоты управляем вертикальной скоростью шагом


def autorotate(h, hdg):
    """Один шаг закона пилота в авторотации. h - вертолёт, hdg - курс, который держим."""
    s = h.get_state()
    agl = s["agl"]
    nr = h.nr
    gs = s["ground_speed"]
    base = clamp(0.5 + 3.0 * (nr - 1.0), 0.4, 1.0)         # шаг держит обороты около 100%
    if agl > FLARE_H:
        coll, cyc = base, 0.0
    else:
        v_target = 4.0 + 0.1 * agl                          # желаемая путевая скорость на этой высоте
        cyc = clamp(-0.05 * (gs - v_target), -0.9, 0.3)     # быстрее нужного: ручку на себя
        if agl < CUSHION_H and (nr > 0.8 or agl < 6.0):     # у земли тянем шаг независимо от оборотов
            vz_target = -(0.5 + 0.25 * agl)
            coll = clamp(0.50 + 0.12 * (vz_target - s["vz"]), 0.0, 1.0)
        else:
            coll = base
    h.turn("collective", coll)
    h.turn("cyclic_pitch", cyc)
    d = (hdg - s["heading"] + 180.0) % 360.0 - 180.0
    h.turn("pedals", clamp(0.05 * d - 0.03 * s["yaw_rate"], -1.0, 1.0))


def fly_and_fail(preset, title, how):
    print("=" * 10, title, "(%s)" % preset)
    heli = Helicopter(preset, timestamp=0.0, fuel_kg=(LOW_FUEL.get(preset, 2.5) if how == "fuel" else None))
    sim = Sim(heli)
    assert cold_start(sim)
    pilot = Autopilot(sim)
    pilot.alt = 100.0
    pilot.speed = 25.0
    if how == "engine":
        pilot.run(40.0)
        print("--- Игра сообщает: отказ двигателя (птица)")
        heli.fail_engine("bird strike")
    else:
        for i in range(int(300.0 / sim.dt)):         # летим, пока не кончится топливо
            if heli.engine.state != "running":
                break
            pilot.step()
            sim.tick()
    hdg = heli.get_state()["heading"]
    print("--- Авторотация")
    for i in range(int(120.0 / sim.dt)):
        if heli.on_ground or heli.crashed:
            break
        autorotate(heli, hdg)
        sim.tick()
    s = heli.get_state()
    if s["crashed"]:
        print("Итог: разбились,", s["crash_reason"])
    else:
        print("Итог: посадка, остаточная скорость по земле %.1f м/с, обороты %.0f%%" % (s["ground_speed"], s["nr_pct"]))


preset = sys.argv[1] if len(sys.argv) > 1 else "medium_turbine"
fly_and_fail(preset, "Отказ двигателя на высоте около 100 м", "engine")
fly_and_fail(preset, "Топливо кончилось в полёте", "fuel")
