"""Example 2: a skeleton of a game. Feet, delta mode (the helicopter does not know the map), collisions
reported by the game, the map edge with a transition to another map, and a pause while loading.
Run: python3 02_game_loop.py"""
import math
from common import Helicopter, Sim, Autopilot, cold_start, phrase, FT

MAP_LEN_FT = 1200.0                 # the map is a strip 0..1200 feet along Y


class Game(object):
    def __init__(self):
        self.map_no = 1
        self.px = 0.0               # the player position lives HERE, in the game; the helicopter has none
        self.py = 0.0
        self.heli = Helicopter("medium_turbine", timestamp=0.0, imperial_units=True)
        self.sim = Sim(self.heli, speak=False)
        self.sim.ground_check = False         # the game checks the ground itself (below)
        self.pilot = Autopilot(self.sim)

    def handle(self, events):
        for e in events:
            if e.name == "moved":
                self.px += e.data["dx"]       # feet
                self.py += e.data["dy"]
                if self.py >= MAP_LEN_FT:
                    self.change_map()
            else:
                p = phrase(e)
                if p:
                    print("[%6.1f s] map %d: %s" % (self.sim.ts, self.map_no, p))

    def change_map(self):
        h = self.heli
        h.reached_map_edge()                  # an event for speech; physics is unchanged (pass mode)
        self.handle(h.pop_events())
        h.pause("map reload")                 # freeze while loading
        self.sim.ts += 3.0                    # the load took 3 seconds of clock time
        self.map_no += 1
        self.py -= MAP_LEN_FT                 # the player appears at the start of the next map
        h.set_ground_level(0.0)               # surface height of the new map (feet)
        h.resume(self.sim.ts)                 # the load time does not count towards the flight
        print("[%6.1f s] moved to map %d, player position Y=%.0f ft" % (self.sim.ts, self.map_no, self.py))

    def frame(self):
        self.pilot.step()
        s = self.sim
        s.ts += s.dt
        evs = self.heli.update(s.ts)
        # the game detects ground contact (in feet, or with its own geometry)
        if not self.heli.on_ground and self.heli.get_state()["agl"] <= 0.0:
            self.heli.reached_surface(0.0)    # in feet
            evs += self.heli.pop_events()
        self.handle(evs)


g = Game()
print("Units:", g.heli.get_state()["units"])
assert cold_start(g.sim)
g.pilot.alt = 30.0                            # meters - the autopilot works in meters inside
g.pilot.speed = 0.0
g.sim.speak = True
g.sim.run(0.0)
for i in range(int(25 / g.sim.dt)):
    g.frame()
print("--- Flying north at 30 m/s")
g.pilot.speed = 30.0
for i in range(int(60 / g.sim.dt)):
    g.frame()
    if g.map_no == 2 and g.py > 300:
        break
print("--- Braking and landing")
g.pilot.speed = 0.0
g.pilot.alt = 0.0
for i in range(int(90 / g.sim.dt)):
    g.frame()
    if g.heli.on_ground and g.sim.ts > 100:
        break
s = g.heli.get_state()
print("Result: map %d, Y=%.0f ft, height %.1f ft, on the ground: %s" % (g.map_no, g.py, s["agl"], s["on_ground"]))
