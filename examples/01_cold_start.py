"""Example 1: a start-up by the checklist, step by step, with spoken events.
Run: python3 01_cold_start.py [light_piston|medium_turbine|heavy_twin]"""
import sys
from common import Helicopter, Sim

preset = sys.argv[1] if len(sys.argv) > 1 else "medium_turbine"
heli = Helicopter(preset, timestamp=0.0)
sim = Sim(heli)
piston = heli.cfg.has_clutch
print("Helicopter:", heli.cfg.name)

# If the starter is pressed before the power is on, the command is rejected with a reason:
heli.button("starter")
sim.tick()

print("--- 1. Power and fuel")
heli.switch("battery", True)           # master: battery + bus
heli.switch("fuel_valve", True)        # fuel valve
if piston:
    heli.switch("ignition", True)      # magneto (piston only)
else:
    heli.switch("fuel_pump", True)     # boost pump (for starting a turbine)
heli.switch("avionics", True)
sim.run(1.0)

print("--- 2. Engine start")
heli.button("starter")                 # press and do not release: it cuts out by itself at the right rpm
sim.until(lambda: heli.engine.state == "running", timeout=60.0)
heli.switch("generator", True)
sim.run(1.0)

print("--- 3. Rotor")
heli.switch("rotor_brake", False)      # release the rotor brake
if piston:
    heli.switch("clutch", True)        # a piston helicopter also needs the clutch
sim.run(2.0)
heli.turn("throttle", 1.0)             # throttle fully open: the goal is 100% rpm
sim.until(lambda: heli.nr >= 0.985, timeout=120.0)

s = heli.get_state()
print("--- Ready for takeoff")
print("Rotor rpm %.0f%%, engine rpm %.0f%%, fuel %.0f kg, voltage %.1f V" %
      (s["nr_pct"], s["n1_pct"], s["fuel_kg"], s["voltage"]))
print("Next: raise the collective smoothly; the helicopter lifts off by itself (the liftoff event).")
