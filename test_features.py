"""Проверки: moved (delta-режим), pause/resume, край карты, футы."""
import time
from helisim import Helicopter, FT
from test_scenarios_helpers import airborne


def run(h, secs, dt=0.05, coll=0.6):
    t = h._last_ts
    evs = []
    for i in range(int(secs / dt)):
        t += dt
        h.turn("collective", coll)
        evs += h.update(t)
    return evs


print("== 1. moved: курс 90 (на восток), летим на север 20 м/с -> bearing 0, relative -90")
h = airborne(alt=200.0, speed=20.0)
h.heading = 90.0
h.ground_z = -5000.0
ev = [e for e in run(h, 2.0) if e.name == "moved"]
sx = sum(e.data["dx"] for e in ev); sy = sum(e.data["dy"] for e in ev)
d = ev[-1].data
print("событий %d, сумма dx=%.2f dy=%.2f | внутри x=%.2f y=%.2f" % (len(ev), sx, sy, h.x, h.y))
print("bearing=%.1f relative=%.1f elevation=%.2f" % (d["bearing"], d["relative_bearing"], d["elevation"]))
print("x,y в get_state (delta):", "x" in h.get_state())

print("\n== 2. absolute режим отдаёт x,y и position_changed")
h = airborne(alt=200.0, speed=20.0); h.cfg.position_mode = "absolute"; h.ground_z = -5000.0
names = set(e.name for e in run(h, 1.0))
print("position_changed" in names, "x" in h.get_state())

print("\n== 3. move_step=5 м: смещение копится, не теряется")
h = airborne(alt=200.0, speed=20.0); h.cfg.move_step = 5.0; h.ground_z = -5000.0
ev = [e for e in run(h, 2.0) if e.name == "moved"]
print("событий %d (раньше ~40), сумма dy=%.2f, остаток не отправлен=%.2f, внутри y=%.2f" %
      (len(ev), sum(e.data["dy"] for e in ev), h.y - h._rep["my"], h.y))

print("\n== 4. футы: moved в футах")
h = airborne(alt=200.0, speed=20.0); h.imperial = True; h.lk = FT; h.ground_z = -5000.0
ev = [e for e in run(h, 1.0) if e.name == "moved"]
print("сумма dy=%.2f фут (внутри %.2f м = %.2f фут)" % (sum(e.data["dy"] for e in ev), h.y, h.y * FT))

print("\n== 5. pause/resume с явными таймстампами")
h = airborne(alt=200.0, speed=0.0); h.ground_z = -5000.0
run(h, 1.0)
z0, t0 = h.z, h.t
h.pause("map reload")
t = h._last_ts
for i in range(100):
    t += 0.05; h.update(t)             # игра продолжает звать update
print("на паузе: время %.2f -> %.2f, z %.2f -> %.2f, paused=%s" % (t0, h.t, z0, h.z, h.get_state()["paused"]))
h.turn("collective", 0.9)               # команды при паузе принимаются
h.resume()
t += 30.0                               # долгая загрузка: часы ушли далеко вперёд
h.update(t)                             # привязка часов
t += 0.05; h.update(t)
print("после resume: время %.2f (прошло ~0.05 с, а не 30), z=%.3f" % (h.t - t0, h.z))
print([e.name for e in h.pop_events()])

print("\n== 6. pause/resume на системном таймере (update() без аргументов)")
h = airborne(alt=200.0); h.ground_z = -5000.0
h._last_ts = None
h.update(); time.sleep(0.1); h.update()
tt = h.t
h.pause(); time.sleep(0.5); h.update(); h.resume(); h.update(); time.sleep(0.1); h.update()
print("прошло симуляции %.2f с при реальных ~0.7 с (ожидаем ~0.1)" % (h.t - tt))

print("\n== 7. край карты")
h = airborne(alt=100.0, speed=20.0); h.ground_z = -5000.0
run(h, 0.5)
h.reached_map_edge()
print("pass: скорость %.1f (не изменилась), события %s" % (h.vy, [e.name for e in h.pop_events()]))
for i in range(5): h.reached_map_edge()   # каждый кадр - событие не спамится
print("5 вызовов подряд -> событий:", len(h.pop_events()))
h.reached_map_edge(nx=0.0, ny=-1.0, mode="block")
print("block с нормалью: vy=%.2f vx=%.2f" % (h.vy, h.vx))
h = airborne(alt=100.0, speed=20.0); h.vx = 5.0
h.reached_map_edge(mode="block")
print("block без нормали: v=(%.1f, %.1f), crashed=%s" % (h.vx, h.vy, h.crashed))
