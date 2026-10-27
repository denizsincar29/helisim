"""Демо: запуск, взлёт, висение, полёт, посадка. Скрипт играет роль игры и пилота."""
import sys
import math
from helisim import Helicopter, HelicopterConfig, angle_diff

QUIET = ("moved", "rotor_rpm_changed", "engine_rpm_changed", "torque_changed", "altitude_changed",
         "position_changed", "heading_changed", "attitude_changed", "speed_changed",
         "vertical_speed_changed", "fuel_changed")


class World(object):
    """Плоская земля на z=0 и стена на y=wall_y (если задана)."""

    def __init__(self, heli, wall_y=None):
        self.h = heli
        self.wall_y = wall_y

    def check(self):
        h = self.h
        if not h.on_ground and h.z <= 0.0:
            h.reached_surface(0.0)
        if self.wall_y is not None and h.y >= self.wall_y:
            h.reached_wall(0.0, -1.0)


def run(preset, verbose_all=False, wall_y=None, fly_speed_pitch=0.35):
    h = Helicopter(preset, timestamp=0.0)
    w = World(h, wall_y)
    pos = [0.0, 0.0]
    dt = 0.05
    ts = 0.0
    st = {"phase": "init", "t0": 0.0, "hdg": 0.0}

    def phase(name):
        st["phase"] = name
        st["t0"] = ts
        print("---- %6.1fs  фаза: %s" % (ts, name))

    def tick():
        nonlocal ts
        ts += dt
        evs = h.update(ts)
        w.check()
        evs += h.pop_events()
        for e in evs:
            if e.name == "moved":
                pos[0] += e.data["dx"]
                pos[1] += e.data["dy"]
            if verbose_all or e.name not in QUIET:
                print(e)
        return evs

    piston = h.cfg.has_clutch
    phase("предстартовая подготовка")
    h.switch("battery", True)
    h.switch("fuel_valve", True)
    h.switch("fuel_pump", True)
    h.switch("avionics", True)
    h.switch("anti_collision", True)
    if piston:
        h.switch("ignition", True)
    for i in range(20):
        tick()
    phase("запуск двигателя")
    h.button("starter")
    while h.engine.state != "running" and ts < 60:
        tick()
    h.switch("generator", True)
    h.switch("rotor_brake", False)
    for i in range(40):
        tick()
    phase("выход на режим")
    if piston:
        h.switch("clutch", True)
        for i in range(100):
            tick()
    h.turn("throttle", 1.0)
    while h.nr < 0.985 and ts < 120:
        tick()
    print("    обороты НВ %.1f%%, n1 %.1f%%" % (h.nr * 100, h.engine.n1 * 100))
    for i in range(40):
        tick()

    # --- простой автопилот ---
    target_alt = 15.0
    target_hdg = 0.0
    v_target = 0.0
    coll_base = 0.55
    phase("взлёт и висение на %d м" % target_alt)
    leg_end = 0.0
    landed = False
    t_land = None
    while ts < 400 and not h.crashed:
        s = h.get_state()
        # высота -> общий шаг
        err = target_alt - s["agl"]
        vz_t = clamp(err * 0.4, -1.5, 3.0) if target_alt > 0.5 else -1.2 if s["agl"] > 3 else -0.5
        coll = coll_base + 0.12 * (vz_t - s["vz"])
        # интегратор базового шага
        coll_base += 0.01 * (vz_t - s["vz"]) * dt
        coll_base = clamp(coll_base, 0.3, 0.9)
        if st["phase"].startswith("взлёт") and s["agl"] < 0.3:
            coll = min(coll + 0.0, 1.0)
        h.turn("collective", clamp(coll, 0.0, 1.0))
        # курс -> педали
        he = angle_diff(target_hdg, s["heading"])
        h.turn("pedals", clamp(0.04 * he - 0.03 * s["yaw_rate"], -1, 1))
        psi = math.radians(s["heading"])
        vfwd = s["vx"] * math.sin(psi) + s["vy"] * math.cos(psi)
        vrt = s["vx"] * math.cos(psi) - s["vy"] * math.sin(psi)
        h.turn("cyclic_pitch", clamp(0.06 * (v_target - vfwd), -0.5, 0.5))
        h.turn("cyclic_roll", clamp(-0.06 * vrt, -0.5, 0.5))
        tick()
        if st["phase"].startswith("взлёт") and ts - st["t0"] > 25:
            phase("полёт вперёд (нос вниз)")
            v_target = 20.0
        if st["phase"].startswith("полёт") and ts - st["t0"] > 25:
            phase("разворот на 90 градусов вправо")
            target_hdg = 90.0
        if st["phase"].startswith("разворот") and ts - st["t0"] > 15:
            phase("торможение, захват висения")
            v_target = 0.0
        if st["phase"].startswith("торможение") and ts - st["t0"] > 20:
            phase("посадка")
            target_alt = 0.0
        if st["phase"] == "посадка" and h.on_ground and t_land is None:
            t_land = ts
            h.turn("collective", 0.0)
        if t_land is not None and ts - t_land > 3:
            break
    if h.crashed:
        print("!!! КРУШЕНИЕ:", h.crash_reason)
    else:
        phase("остановка")
        h.turn("collective", 0.0)
        h.turn("throttle", 0.0)
        for i in range(100):
            tick()
        h.button("engine_stop")
        h.switch("rotor_brake", True)
        for i in range(400):
            tick()
    s = h.get_state()
    print("ИТОГ: t=%.0fs x=%.0f y=%.0f z=%.1f двигатель=%s топливо=%.1f кг" %
          (s["t"], pos[0], pos[1], s["z"], s["engine_state"], s["fuel_kg"]))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


if __name__ == "__main__":
    run(sys.argv[1] if len(sys.argv) > 1 else "medium_turbine")
