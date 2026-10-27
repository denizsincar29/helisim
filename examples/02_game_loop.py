"""Пример 2: скелет игры. Футы, режим delta (вертолёт не знает карту), столкновения от игры,
край карты с переходом на другую карту и пауза на время загрузки.
Запуск: python3 02_game_loop.py"""
import math
from common import Helicopter, Sim, Autopilot, cold_start, phrase, FT

MAP_LEN_FT = 1200.0                 # карта - полоса 0..1200 футов по Y


class Game(object):
    def __init__(self):
        self.map_no = 1
        self.px = 0.0               # позиция игрока ЗДЕСЬ, у игры; у вертолёта её нет
        self.py = 0.0
        self.heli = Helicopter("medium_turbine", timestamp=0.0, imperial_units=True)
        self.sim = Sim(self.heli, speak=False)
        self.sim.ground_check = False         # землю проверяет игра сама (ниже)
        self.pilot = Autopilot(self.sim)

    def handle(self, events):
        for e in events:
            if e.name == "moved":
                self.px += e.data["dx"]       # футы
                self.py += e.data["dy"]
                if self.py >= MAP_LEN_FT:
                    self.change_map()
            else:
                p = phrase(e)
                if p:
                    print("[%6.1f с] карта %d: %s" % (self.sim.ts, self.map_no, p))

    def change_map(self):
        h = self.heli
        h.reached_map_edge()                  # событие для озвучки; физика не меняется (режим pass)
        self.handle(h.pop_events())
        h.pause("map reload")                 # заморозить, пока грузится
        self.sim.ts += 3.0                    # загрузка заняла 3 секунды часов
        self.map_no += 1
        self.py -= MAP_LEN_FT                 # игрок появляется в начале следующей карты
        h.set_ground_level(0.0)               # высота поверхности новой карты (футы)
        h.resume(self.sim.ts)                 # время загрузки полёту не засчитывается
        print("[%6.1f с] перешли на карту %d, позиция игрока Y=%.0f фут" % (self.sim.ts, self.map_no, self.py))

    def frame(self):
        self.pilot.step()
        s = self.sim
        s.ts += s.dt
        evs = self.heli.update(s.ts)
        # столкновение с землёй определяет игра (по футам или по своей геометрии)
        if not self.heli.on_ground and self.heli.get_state()["agl"] <= 0.0:
            self.heli.reached_surface(0.0)    # в футах
            evs += self.heli.pop_events()
        self.handle(evs)


g = Game()
print("Единицы:", g.heli.get_state()["units"])
assert cold_start(g.sim)
g.pilot.alt = 30.0                            # метры - автопилот считает в метрах внутри
g.pilot.speed = 0.0
g.sim.speak = True
g.sim.run(0.0)
for i in range(int(25 / g.sim.dt)):
    g.frame()
print("--- Летим на север 30 м/с")
g.pilot.speed = 30.0
for i in range(int(60 / g.sim.dt)):
    g.frame()
    if g.map_no == 2 and g.py > 300:
        break
print("--- Торможение и посадка")
g.pilot.speed = 0.0
g.pilot.alt = 0.0
for i in range(int(90 / g.sim.dt)):
    g.frame()
    if g.heli.on_ground and g.sim.ts > 100:
        break
s = g.heli.get_state()
print("Итог: карта %d, Y=%.0f фут, высота %.1f фут, на земле: %s" % (g.map_no, g.py, s["agl"], s["on_ground"]))
