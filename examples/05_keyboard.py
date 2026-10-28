"""Example 5: keyboard control. A key layout and a key handler; the key presses are "played" from a script.
Toggles act on key press. Levers (collective, throttle) move in steps and stay where they were left.
Cyclic and pedals act while the key is held; a spring returns them to center on release.
Run: python3 05_keyboard.py"""
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
            self.h.turn(SPRING[key][0], 0.0)      # spring: stick/pedals back to center


heli = Helicopter("medium_turbine", timestamp=0.0)
sim = Sim(heli)
kb = Keyboard(heli)

# script: (time, action, key)
script = [
    (0.5, "down", "r"),      # rotor brake: it is on when parked, so switch it off
    (1.0, "down", "b"), (1.5, "down", "f"), (2.0, "down", "p"),
    (3.0, "down", "s"),                          # starter: press and do not release
    (25.0, "down", "g"),
    (27.0, "down", "t"), (27.2, "down", "t"), (27.4, "down", "t"), (27.6, "down", "t"),  # throttle all the way up
]
# after the rpm is up: collective in small steps, then hold "forward" for 2 seconds
for i in range(14):
    script.append((50.0 + i * 0.4, "down", "pageup"))
script += [(60.0, "down", "up"), (62.0, "up", "up"),
           (70.0, "down", "pagedown"), (70.4, "down", "pagedown")]

script.sort(key=lambda a: a[0])
i = 0
while sim.ts < 75.0 and not heli.crashed:
    while i < len(script) and script[i][0] <= sim.ts:
        t, what, key = script[i]
        sim.say("Key %s: %s" % (key, "pressed" if what == "down" else "released"))
        if what == "down":
            kb.down(key)
        else:
            kb.up(key)
        i += 1
    sim.tick()

s = heli.get_state()
print("Result: on the ground=%s, height %.1f m, speed %.1f m/s, rotor rpm %.0f%%, collective %.2f" %
      (s["on_ground"], s["agl"], s["ground_speed"], s["nr_pct"], s["controls"]["collective"]))
