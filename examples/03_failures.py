"""Example 3: an engine failure in flight and an autorotation; running out of fuel.
Shows what is expected of the pilot: hold rotor rpm with the collective, hold the heading with the pedals
(there is no torque in an autorotation, but the tail rotor keeps pushing), flare near the ground,
and cushion with the collective just before touchdown.
Run: python3 03_failures.py [light_piston|medium_turbine|heavy_twin]"""
import sys
from common import Helicopter, Sim, Autopilot, cold_start, clamp

LOW_FUEL = {"light_piston": 0.6}        # kg: so little that the fuel runs out within minutes
FLARE_H = 60.0      # m: from this height we bleed off speed
CUSHION_H = 30.0    # m: from this height we control the sink rate with the collective


def autorotate(h, hdg):
    """One step of the autorotation pilot law. h is the helicopter, hdg is the heading to hold."""
    s = h.get_state()
    agl = s["agl"]
    nr = h.nr
    gs = s["ground_speed"]
    base = clamp(0.5 + 3.0 * (nr - 1.0), 0.4, 1.0)         # the collective holds rpm near 100%
    if agl > FLARE_H:
        coll, cyc = base, 0.0
    else:
        v_target = 4.0 + 0.1 * agl                          # the desired ground speed at this height
        cyc = clamp(-0.05 * (gs - v_target), -0.9, 0.3)     # faster than wanted: stick back
        if agl < CUSHION_H and (nr > 0.8 or agl < 6.0):     # near the ground we pull collective regardless of rpm
            vz_target = -(0.5 + 0.25 * agl)
            coll = clamp(0.50 + 0.12 * (vz_target - s["vz"]), 0.0, 1.0)
        else:
            coll = base
    h.turn("collective", coll)
    h.turn("cyclic_pitch", cyc)
    d = (hdg - s["heading"] + 180.0) % 360.0 - 180.0
    h.turn("pedals", clamp(0.05 * d - 0.03 * s["yaw_rate"], -1.0, 1.0))


def fly_and_fail(preset, title, how):
    print("=" * 10, title, "(%s)" % preset)
    heli = Helicopter(preset, timestamp=0.0, fuel_kg=(LOW_FUEL.get(preset, 2.5) if how == "fuel" else None))
    sim = Sim(heli)
    assert cold_start(sim)
    pilot = Autopilot(sim)
    pilot.alt = 100.0
    pilot.speed = 25.0
    if how == "engine":
        pilot.run(40.0)
        print("--- The game reports: engine failure (bird strike)")
        heli.fail_engine("bird strike")
    else:
        for i in range(int(300.0 / sim.dt)):         # fly until the fuel runs out
            if heli.engine.state != "running":
                break
            pilot.step()
            sim.tick()
    hdg = heli.get_state()["heading"]
    print("--- Autorotation")
    for i in range(int(120.0 / sim.dt)):
        if heli.on_ground or heli.crashed:
            break
        autorotate(heli, hdg)
        sim.tick()
    s = heli.get_state()
    if s["crashed"]:
        print("Result: crashed,", s["crash_reason"])
    else:
        print("Result: landed, remaining ground speed %.1f m/s, rotor rpm %.0f%%" % (s["ground_speed"], s["nr_pct"]))


preset = sys.argv[1] if len(sys.argv) > 1 else "medium_turbine"
fly_and_fail(preset, "Engine failure at about 100 m", "engine")
fly_and_fail(preset, "Out of fuel in flight", "fuel")
