"""Сценарные проверки: отказы, столкновения, отклонённые команды."""
from helisim import Helicopter


def airborne(preset="medium_turbine", alt=100.0, speed=0.0):
    """Быстро поднять вертолёт в воздух в рабочем состоянии (для тестов)."""
    h = Helicopter(preset, timestamp=0.0, z=alt)
    h.ground_z = 0.0
    h.on_ground = False
    for k in ("battery", "fuel_valve", "fuel_pump", "generator"):
        h.switch(k, True)
    h.switch("rotor_brake", False)
    if h.cfg.has_clutch:
        h.switch("ignition", True)
        h.switch("clutch", True)
    h.engine.state = "running"
    h.engine.n1 = 1.0
    h.nr = 1.0
    h.turn("throttle", 1.0)
    h.axis["throttle"] = 1.0
    h.vy = speed
    h.pop_events()
    return h


def fly(h, seconds, coll=None, dt=0.05):
    t = h._last_ts
    out = []
    for i in range(int(seconds / dt)):
        t += dt
        if coll is not None:
            h.turn("collective", coll)
        out += h.update(t)
    return out


def names(evs, skip=("rotor_rpm_changed", "engine_rpm_changed", "torque_changed", "altitude_changed",
                      "position_changed", "attitude_changed", "speed_changed", "vertical_speed_changed",
                      "heading_changed", "fuel_changed")):
    return [e for e in evs if e.name not in skip]


print("== 1. стартёр без питания / сцепление там, где его нет")
h = Helicopter("medium_turbine", timestamp=0.0)
h.press("starter")
h.switch("clutch", True)
for e in h.pop_events():
    print(e)

print("\n== 2. отказ двигателя на 100 м: авторотация (шаг вниз)")
h = airborne(alt=100.0, speed=20.0)
h.engine.set_state("failed")
# пилот держит обороты ~100% шагом (как в реальной авторотации)
t = h._last_ts
for i in range(int(30.0 / 0.05)):
    t += 0.05
    h.turn("collective", max(0.0, min(1.0, 0.25 + 6.0 * (h.nr - 1.0))))
    for e in names(h.update(t)):
        print(e)
    if h.crashed or h.vz > -0.1 and i > 100:
        break
s = h.get_state()
print("через 31 с: nr=%.0f%% vz=%.1f высота=%.0f crashed=%s" % (s["nr_pct"], s["vz"], s["z"], s["crashed"]))

print("\n== 3. отказ двигателя, шаг НЕ снижают (ротор садится)")
h = airborne(alt=100.0, speed=0.0)
h.engine.set_state("failed")
for e in names(fly(h, 8.0, coll=0.6)):
    print(e)
print("nr=%.0f%% vz=%.1f" % (h.nr * 100, h.vz))

print("\n== 4. стена на скорости 15 м/с (игра сообщает reached_wall)")
h = airborne(alt=5.0, speed=15.0)
h.reached_wall(0.0, -1.0)
for e in h.pop_events():
    print(e)

print("\n== 5. лёгкий контакт со стеной 1.5 м/с -> bump, не авария")
h = airborne(alt=5.0, speed=1.5)
h.reached_wall(0.0, -1.0)
for e in h.pop_events():
    print(e)

print("\n== 6. падение: reached_surface на скорости снижения 9 м/с")
h = airborne(alt=5.0)
h.vz = -9.0
h.reached_surface(0.0)
for e in h.pop_events():
    print(e)

print("\n== 7. жёсткая посадка 4 м/с (не авария, повреждение)")
h = airborne(alt=5.0)
h.vz = -4.0
h.reached_surface(0.0)
print([e.name + str(e.data) for e in h.pop_events()], "здоровье планера %.2f" % h.airframe_health)

print("\n== 8. вертикальное снижение 10 м/с с мощностью (вихревое кольцо)")
h = airborne(alt=300.0)
h.vz = -10.0
ev = fly(h, 6.0, coll=0.55)
for e in names(ev):
    if e.name == "warning":
        print(e)
print("vz=%.1f nr=%.0f%%" % (h.vz, h.nr * 100))

print("\n== 9. превышение Vne")
h = airborne(alt=300.0, speed=70.0)
for e in names(fly(h, 5.0, coll=0.55)):
    print(e)

print("\n== 10. вертолёт сам по себе после крушения не шевелится")
h = airborne(alt=5.0, speed=15.0)
h.reached_wall(0.0, -1.0)
h.pop_events()
print(fly(h, 2.0), h.get_state()["crashed"])
