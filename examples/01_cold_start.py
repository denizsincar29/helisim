"""Пример 1: запуск по чек-листу, шаг за шагом, с озвучкой событий.
Запуск: python3 01_cold_start.py [light_piston|medium_turbine|heavy_twin]"""
import sys
from common import Helicopter, Sim

preset = sys.argv[1] if len(sys.argv) > 1 else "medium_turbine"
heli = Helicopter(preset, timestamp=0.0)
sim = Sim(heli)
piston = heli.cfg.has_clutch
print("Вертолёт:", heli.cfg.name)

# Если нажать стартёр до включения питания - команда отклоняется с причиной:
heli.button("starter")
sim.tick()

print("--- 1. Питание и топливо")
heli.switch("battery", True)           # master: батарея + шина
heli.switch("fuel_valve", True)        # топливный кран
if piston:
    heli.switch("ignition", True)      # магнето (только у поршневого)
else:
    heli.switch("fuel_pump", True)     # подкачка (для запуска турбины)
heli.switch("avionics", True)
sim.run(1.0)

print("--- 2. Запуск двигателя")
heli.button("starter")                 # нажать и не отпускать: сам отсечётся по оборотам
sim.until(lambda: heli.engine.state == "running", timeout=60.0)
heli.switch("generator", True)
sim.run(1.0)

print("--- 3. Ротор")
heli.switch("rotor_brake", False)      # тормоз ротора снять
if piston:
    heli.switch("clutch", True)        # у поршневого - ещё и сцепление
sim.run(2.0)
heli.turn("throttle", 1.0)             # коррекция газа до упора: цель - 100% оборотов
sim.until(lambda: heli.nr >= 0.985, timeout=120.0)

s = heli.get_state()
print("--- Готов к взлёту")
print("Обороты ротора %.0f%%, обороты двигателя %.0f%%, топливо %.0f кг, напряжение %.1f В" %
      (s["nr_pct"], s["n1_pct"], s["fuel_kg"], s["voltage"]))
print("Дальше: collective плавно вверх, на земле вертолёт оторвётся сам (событие liftoff).")
