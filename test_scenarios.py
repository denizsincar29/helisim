"""Scenario checks: failures, collisions, rejected commands. Prints what happens (no assertions)."""
from helisim import Helicopter
from testutil import airborne, fly, names



print("== 1. starter without power / a clutch where there is none")
h = Helicopter("medium_turbine", timestamp=0.0)
h.press("starter")
h.switch("clutch", True)
for e in h.pop_events():
    print(e)

print("\n== 2. engine failure at 100 m: autorotation (collective down)")
h = airborne(alt=100.0, speed=20.0)
h.engine.set_state("failed")
# the pilot holds ~100% rpm with the collective (as in a real autorotation)
t = h._last_ts
for i in range(int(30.0 / 0.05)):
    t += 0.05
    h.turn("collective", max(0.0, min(1.0, 0.45 + 4.0 * (h.nr - 1.0))))
    for e in names(h.update(t)):
        print(e)
    if h.crashed or h.z < 3.0:
        break
s = h.get_state()
print("near the ground (t=%.0f s): nr=%.0f%% vz=%.1f height=%.0f crashed=%s" % (s["t"], s["nr_pct"], s["vz"], s["z"], s["crashed"]))

print("\n== 3. engine failure, collective NOT lowered (the rotor decays)")
h = airborne(alt=100.0, speed=0.0)
h.engine.set_state("failed")
for e in names(fly(h, 8.0, coll=0.6)):
    print(e)
print("nr=%.0f%% vz=%.1f" % (h.nr * 100, h.vz))

print("\n== 4. wall at 15 m/s (the game reports reached_wall)")
h = airborne(alt=5.0, speed=15.0)
h.reached_wall(0.0, -1.0)
for e in h.pop_events():
    print(e)

print("\n== 5. light wall contact at 1.5 m/s -> bump, not a crash")
h = airborne(alt=5.0, speed=1.5)
h.reached_wall(0.0, -1.0)
for e in h.pop_events():
    print(e)

print("\n== 6. impact: reached_surface at a 9 m/s sink rate")
h = airborne(alt=5.0)
h.vz = -9.0
h.reached_surface(0.0)
for e in h.pop_events():
    print(e)

print("\n== 7. hard landing at 4 m/s (not a crash, causes damage)")
h = airborne(alt=5.0)
h.vz = -4.0
h.reached_surface(0.0)
print([e.name + str(e.data) for e in h.pop_events()], "airframe health %.2f" % h.airframe_health)

print("\n== 8. vertical descent at 10 m/s with power (vortex ring)")
h = airborne(alt=300.0)
h.vz = -10.0
ev = fly(h, 6.0, coll=0.55)
for e in names(ev):
    if e.name == "warning":
        print(e)
print("vz=%.1f nr=%.0f%%" % (h.vz, h.nr * 100))

print("\n== 9. Vne exceeded")
h = airborne(alt=300.0, speed=70.0)
for e in names(fly(h, 5.0, coll=0.55)):
    print(e)

print("\n== 10. after a crash the helicopter does not move any more")
h = airborne(alt=5.0, speed=15.0)
h.reached_wall(0.0, -1.0)
h.pop_events()
print(fly(h, 2.0), h.get_state()["crashed"])
