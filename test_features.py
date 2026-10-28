"""Feature checks: moved (delta mode), pause/resume, map edge, feet. Prints what happens (no assertions)."""
import time
from helisim import Helicopter, FT
from testutil import airborne


def run(h, secs, dt=0.05, coll=0.6):
    t = h._last_ts
    evs = []
    for i in range(int(secs / dt)):
        t += dt
        h.turn("collective", coll)
        evs += h.update(t)
    return evs


print("== 1. moved: heading 90 (east), flying north at 20 m/s -> bearing 0, relative -90")
h = airborne(alt=200.0, speed=20.0)
h.heading = 90.0
h.ground_z = -5000.0
ev = [e for e in run(h, 2.0) if e.name == "moved"]
sx = sum(e.data["dx"] for e in ev); sy = sum(e.data["dy"] for e in ev)
d = ev[-1].data
print("events %d, sum dx=%.2f dy=%.2f | internal x=%.2f y=%.2f" % (len(ev), sx, sy, h.x, h.y))
print("bearing=%.1f relative=%.1f elevation=%.2f" % (d["bearing"], d["relative_bearing"], d["elevation"]))
print("x,y in get_state (delta):", "x" in h.get_state())

print("\n== 2. absolute mode returns x,y and position_changed")
h = airborne(alt=200.0, speed=20.0); h.cfg.position_mode = "absolute"; h.ground_z = -5000.0
names = set(e.name for e in run(h, 1.0))
print("position_changed" in names, "x" in h.get_state())

print("\n== 3. move_step=5 m: displacement accumulates and is not lost")
h = airborne(alt=200.0, speed=20.0); h.cfg.move_step = 5.0; h.ground_z = -5000.0
ev = [e for e in run(h, 2.0) if e.name == "moved"]
print("events %d (before: ~40), sum dy=%.2f, not yet sent=%.2f, internal y=%.2f" %
      (len(ev), sum(e.data["dy"] for e in ev), h.y - h._rep["my"], h.y))

print("\n== 4. feet: moved in feet")
h = airborne(alt=200.0, speed=20.0); h.imperial = True; h.lk = FT; h.ground_z = -5000.0
ev = [e for e in run(h, 1.0) if e.name == "moved"]
print("sum dy=%.2f ft (internal %.2f m = %.2f ft)" % (sum(e.data["dy"] for e in ev), h.y, h.y * FT))

print("\n== 5. pause/resume with explicit timestamps")
h = airborne(alt=200.0, speed=0.0); h.ground_z = -5000.0
run(h, 1.0)
z0, t0 = h.z, h.t
h.pause("map reload")
t = h._last_ts
for i in range(100):
    t += 0.05; h.update(t)             # the game keeps calling update
print("while paused: time %.2f -> %.2f, z %.2f -> %.2f, paused=%s" % (t0, h.t, z0, h.z, h.get_state()["paused"]))
h.turn("collective", 0.9)               # commands are accepted while paused
h.resume()
t += 30.0                               # a long load: the clock jumped far ahead
h.update(t)                             # clock anchoring
t += 0.05; h.update(t)
print("after resume: time %.2f (~0.05 s passed, not 30), z=%.3f" % (h.t - t0, h.z))
print([e.name for e in h.pop_events()])

print("\n== 6. pause/resume on the system timer (update() without arguments)")
h = airborne(alt=200.0); h.ground_z = -5000.0
h._last_ts = None
h.update(); time.sleep(0.1); h.update()
tt = h.t
h.pause(); time.sleep(0.5); h.update(); h.resume(); h.update(); time.sleep(0.1); h.update()
print("simulated %.2f s over ~0.7 s of real time (expected ~0.1)" % (h.t - tt))

print("\n== 7. map edge")
h = airborne(alt=100.0, speed=20.0); h.ground_z = -5000.0
run(h, 0.5)
h.reached_map_edge()
print("pass: speed %.1f (unchanged), events %s" % (h.vy, [e.name for e in h.pop_events()]))
for i in range(5): h.reached_map_edge()   # called every frame - the event is not spammed
print("5 calls in a row -> events:", len(h.pop_events()))
h.reached_map_edge(nx=0.0, ny=-1.0, mode="block")
print("block with a normal: vy=%.2f vx=%.2f" % (h.vy, h.vx))
h = airborne(alt=100.0, speed=20.0); h.vx = 5.0
h.reached_map_edge(mode="block")
print("block without a normal: v=(%.1f, %.1f), crashed=%s" % (h.vx, h.vy, h.crashed))
