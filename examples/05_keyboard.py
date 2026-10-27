"""Пример 5: управление с клавиатуры. Раскладка и обработчик клавиш; нажатия «проигрываются» по сценарию.
Тумблеры - по нажатию. Рычаги (шаг-газ, газ) двигаются шагом и остаются там, где оставили.
Циклик и педали - пока клавиша зажата, пружина возвращает в центр при отпускании.
Запуск: python3 05_keyboard.py"""
from common import Helicopter, Sim, SWITCH_INFO, phrase

TOGGLES = {"b": "battery", "f": "fuel_valve", "p": "fuel_pump", "m": "ignition", "g": "generator",
           "r": "rotor_brake", "c": "clutch", "l": "landing_light", "x": "sas"}
BUTTONS = {"s": "starter", "e": "engine_stop"}
LEVERS = {"pageup": ("collective", +0.05), "pagedown": ("collective", -0.05),
          "t": ("throttle", +0.25), "y": ("throttle", -0.25)}
SPRING = {"up": ("cyclic_pitch", +0.5), "down": ("cyclic_pitch", -0.5),
          "left": ("cyclic_roll", -0.5), "right": ("cyclic_roll", +0.5),
          "comma": ("pedals", -0.5), "period": ("pedals", +0.5)}


class Keyboard(object):
    def __init__(self, heli):
        self.h = heli

    def down(self, key):
        h = self.h
        if key in TOGGLES:
            h.toggle(TOGGLES[key])
        elif key in BUTTONS:
            h.button(BUTTONS[key])
        elif key in LEVERS:
            axis, delta = LEVERS[key]
            h.step(axis, delta)
        elif key in SPRING:
            axis, v = SPRING[key]
            h.turn(axis, v)

    def up(self, key):
        if key in SPRING:
            self.h.turn(SPRING[key][0], 0.0)      # пружина: ручка/педали в центр


heli = Helicopter("medium_turbine", timestamp=0.0)
sim = Sim(heli)
kb = Keyboard(heli)

# сценарий: (время, действие, клавиша)
script = [
    (0.5, "down", "r"),      # тормоз ротора: он включён на стоянке, выключаем
    (1.0, "down", "b"), (1.5, "down", "f"), (2.0, "down", "p"),
    (3.0, "down", "s"),                          # стартёр: нажать и не отпускать
    (25.0, "down", "g"),
    (27.0, "down", "t"), (27.2, "down", "t"), (27.4, "down", "t"), (27.6, "down", "t"),  # газ до упора
]
# после выхода на обороты - шаг вверх шажками, потом зажать "вперёд" на 2 секунды
for i in range(14):
    script.append((50.0 + i * 0.4, "down", "pageup"))
script += [(60.0, "down", "up"), (62.0, "up", "up"),
           (70.0, "down", "pagedown"), (70.4, "down", "pagedown")]

script.sort(key=lambda a: a[0])
i = 0
while sim.ts < 75.0 and not heli.crashed:
    while i < len(script) and script[i][0] <= sim.ts:
        t, what, key = script[i]
        sim.say("Клавиша %s: %s" % (key, "нажата" if what == "down" else "отпущена"))
        if what == "down":
            kb.down(key)
        else:
            kb.up(key)
        i += 1
    sim.tick()

s = heli.get_state()
print("Итог: на земле=%s, высота %.1f м, скорость %.1f м/с, обороты ротора %.0f%%, шаг %.2f" %
      (s["on_ground"], s["agl"], s["ground_speed"], s["nr_pct"], s["controls"]["collective"]))
