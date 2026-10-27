"""Общее для примеров: текстовая озвучка событий, обёртка с часами и простой автопилот.
Вывод - чистый текст, по строке на событие (удобно для экранного диктора)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from helisim import Helicopter, HelicopterConfig, PRESETS, SWITCH_INFO, BUTTON_INFO, FT  # noqa: E402,F401

STATE_RU = {"off": "остановлен", "cranking": "прокрутка", "starting": "запуск",
            "running": "работает", "shutdown": "останов", "failed": "отказ"}
WARN_RU = {
    "low_rotor_rpm": "низкие обороты ротора", "high_rotor_rpm": "высокие обороты ротора",
    "overtorque": "превышен крутящий момент", "overtemp": "перегрев двигателя",
    "low_fuel": "мало топлива", "low_battery": "батарея разряжена",
    "vortex_ring": "вихревое кольцо", "retreating_blade_stall": "срыв на отступающей лопасти",
    "overspeed": "превышена максимальная скорость", "rotor_brake_on": "тормоз ротора включён",
    "hard_landing": "жёсткая посадка",
}
REJECT_RU = {"no_electric_power": "нет питания", "engine_running": "двигатель уже работает",
             "engine_not_running": "двигатель не работает", "not_installed": "на этом вертолёте нет",
             "unknown": "неизвестная команда", "crashed": "вертолёт разбит"}
CRASH_RU = {"hard_landing": "жёсткое падение", "tilted_landing": "посадка с большим креном",
            "fast_landing": "посадка на большой скорости", "wall_impact": "удар о препятствие",
            "rotor_strike": "удар винтом", "tail_strike": "удар хвостом", "water": "падение в воду",
            "rotor_overspeed": "раскрутка ротора", "structural_failure": "разрушение конструкции"}
TREND_RU = {"climb": "набор высоты", "descent": "снижение", "level": "горизонтальный полёт"}


def phrase(e):
    """Событие -> фраза для озвучки или None, если событие не для голоса."""
    n, d = e.name, e.data
    if n == "switch":
        return "%s: %s" % (SWITCH_INFO[d["name"]], "включено" if d["on"] else "выключено")
    if n == "button" and d["pressed"]:
        return "Кнопка: %s" % BUTTON_INFO[d["name"]]
    if n == "command_rejected":
        return "Не выполнено, %s" % REJECT_RU.get(d["reason"], d["reason"])
    if n == "engine_state":
        return "Двигатель: %s" % STATE_RU[d["new"]]
    if n == "engine_started":
        return "Двигатель запущен"
    if n == "starter_cutout":
        return "Стартёр отключился"
    if n == "hot_start":
        return "Горячий запуск! Уберите газ"
    if n == "start_failed":
        return "Запуск сорван"
    if n == "engine_flameout":
        return "Двигатель заглох"
    if n == "engine_failure":
        return "Отказ двигателя"
    if n == "fuel_exhausted":
        return "Топливо кончилось"
    if n == "warning":
        return "%s: %s" % ("Внимание" if d["active"] else "Отбой", WARN_RU.get(d["id"], d["id"]))
    if n == "liftoff":
        return "Оторвались от земли"
    if n == "touchdown":
        return "Касание, вертикальная %.1f%s" % (d["sink"], " жёсткое" if d["hard"] else "")
    if n == "bump":
        return "Касание препятствия"
    if n == "crashed":
        return "Крушение: %s" % CRASH_RU.get(d["reason"], d["reason"])
    if n == "map_edge":
        return "Край карты"
    if n == "paused":
        return "Пауза"
    if n == "resumed":
        return "Продолжили"
    if n == "vertical_trend_changed":
        return TREND_RU[d["trend"]]
    return None


class Sim(object):
    """Игровой цикл в миниатюре: часы с шагом dt, проверка земли, озвучка, накопление смещения."""

    def __init__(self, heli, dt=0.05, speak=True):
        self.h = heli
        self.dt = dt
        self.ts = 0.0
        self.speak = speak
        self.pos = [0.0, 0.0]
        self.ground_check = True
        heli.update(self.ts)                   # привязка часов

    def say(self, text):
        print("[%6.1f с] %s" % (self.ts, text))

    def tick(self):
        self.ts += self.dt
        evs = self.h.update(self.ts)
        h = self.h
        if self.ground_check and not h.on_ground and h.z <= h.ground_z:
            h.reached_surface()                # "игра" сообщает о касании земли
            evs += h.pop_events()
        for e in evs:
            if e.name == "moved":
                self.pos[0] += e.data["dx"]
                self.pos[1] += e.data["dy"]
            elif self.speak:
                p = phrase(e)
                if p:
                    self.say(p)
        return evs

    def run(self, seconds):
        for i in range(int(seconds / self.dt + 0.5)):
            self.tick()
            if self.h.crashed:
                break

    def until(self, cond, timeout=120.0):
        t_end = self.ts + timeout
        while not cond() and self.ts < t_end and not self.h.crashed:
            self.tick()
        return cond()


def cold_start(sim, piston=None):
    """Полный запуск по чек-листу до 100% оборотов ротора. Возвращает True при успехе."""
    h = sim.h
    if piston is None:
        piston = h.cfg.has_clutch
    h.switch("battery", True)
    h.switch("fuel_valve", True)
    if piston:
        h.switch("ignition", True)
    else:
        h.switch("fuel_pump", True)
    h.switch("avionics", True)
    sim.run(1.0)
    h.button("starter")                        # нажать и не отпускать: сам отсечётся
    if not sim.until(lambda: h.engine.state == "running", 60.0):
        return False
    h.switch("generator", True)
    h.switch("rotor_brake", False)
    if piston:
        h.switch("clutch", True)
    sim.run(2.0)
    h.turn("throttle", 1.0)
    return sim.until(lambda: h.nr >= 0.985, 120.0)


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


class Autopilot(object):
    """Простой «пилот»: высота, курс, скорость вперёд. Работает в любых единицах API."""

    def __init__(self, sim):
        self.sim = sim
        self.alt = 0.0          # м
        self.hdg = 0.0
        self.speed = 0.0        # м/с вперёд
        self.base = 0.55
        self.enabled = True

    def step(self):
        if not self.enabled:
            return
        h = self.sim.h
        s = h.get_state()
        k = FT if s["units"]["length"] == "ft" else 1.0
        agl = s["agl"] / k
        vz = s["vz"] / k
        err = self.alt - agl
        vz_t = clamp(err * 0.4, -1.5, 3.0) if self.alt > 0.5 else (-1.2 if agl > 3.0 else -0.5)
        self.base = clamp(self.base + 0.01 * (vz_t - vz) * self.sim.dt, 0.3, 0.9)
        h.turn("collective", clamp(self.base + 0.12 * (vz_t - vz), 0.0, 1.0))
        d = (self.hdg - s["heading"] + 180.0) % 360.0 - 180.0
        h.turn("pedals", clamp(0.04 * d - 0.03 * s["yaw_rate"], -1.0, 1.0))
        import math
        psi = math.radians(s["heading"])
        vx = s["vx"] / k
        vy = s["vy"] / k
        vf = vx * math.sin(psi) + vy * math.cos(psi)
        vr = vx * math.cos(psi) - vy * math.sin(psi)
        h.turn("cyclic_pitch", clamp(0.06 * (self.speed - vf), -0.5, 0.5))
        h.turn("cyclic_roll", clamp(-0.06 * vr, -0.5, 0.5))

    def run(self, seconds):
        for i in range(int(seconds / self.sim.dt + 0.5)):
            self.step()
            self.sim.tick()
            if self.sim.h.crashed:
                break
