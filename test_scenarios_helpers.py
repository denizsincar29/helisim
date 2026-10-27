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



def airborne_like(h, alt):
    g = airborne(h.cfg.name, alt=alt)
    return g
