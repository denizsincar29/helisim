"""Shared by the examples: spoken text for events, a wrapper with a clock, and a simple autopilot.
The output is plain text, one line per event (friendly to screen readers)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from helisim import Helicopter, HelicopterConfig, PRESETS, SWITCH_INFO, BUTTON_INFO, FT  # noqa: E402,F401

STATE_TEXT = {"off": "stopped", "cranking": "cranking", "starting": "starting",
              "running": "running", "shutdown": "shutting down", "failed": "failed"}
WARN_TEXT = {
    "low_rotor_rpm": "low rotor rpm", "high_rotor_rpm": "high rotor rpm",
    "overtorque": "torque limit exceeded", "overtemp": "engine over temperature",
    "low_fuel": "low fuel", "low_battery": "low battery",
    "vortex_ring": "vortex ring state", "retreating_blade_stall": "retreating blade stall",
    "overspeed": "never-exceed speed exceeded", "rotor_brake_on": "rotor brake is on",
    "hard_landing": "hard landing",
}
REJECT_TEXT = {"no_electric_power": "no electric power", "engine_running": "the engine is already running",
             "engine_not_running": "the engine is not running", "not_installed": "not installed on this helicopter",
             "unknown": "unknown command", "crashed": "the helicopter is wrecked"}
CRASH_TEXT = {"hard_landing": "hard impact with the ground", "tilted_landing": "landing with a large bank angle",
            "fast_landing": "landing at high speed", "wall_impact": "collision with an obstacle",
            "rotor_strike": "rotor strike", "tail_strike": "tail strike", "water": "water impact",
            "rotor_overspeed": "rotor overspeed", "structural_failure": "structural failure"}
TREND_TEXT = {"climb": "climbing", "descent": "descending", "level": "level flight"}


def phrase(e):
    """Event -> a phrase for speech, or None if the event is not meant to be spoken."""
    n, d = e.name, e.data
    if n == "switch":
        return "%s: %s" % (SWITCH_INFO[d["name"]], "on" if d["on"] else "off")
    if n == "button" and d["pressed"]:
        return "Button: %s" % BUTTON_INFO[d["name"]]
    if n == "command_rejected":
        return "Command rejected, %s" % REJECT_TEXT.get(d["reason"], d["reason"])
    if n == "engine_state":
        return "Engine: %s" % STATE_TEXT[d["new"]]
    if n == "engine_started":
        return "Engine started"
    if n == "starter_cutout":
        return "Starter cut out"
    if n == "hot_start":
        return "Hot start! Retard the throttle"
    if n == "start_failed":
        return "Start failed"
    if n == "engine_flameout":
        return "Engine flameout"
    if n == "engine_failure":
        return "Engine failure"
    if n == "fuel_exhausted":
        return "Out of fuel"
    if n == "warning":
        return "%s: %s" % ("Warning" if d["active"] else "Clear", WARN_TEXT.get(d["id"], d["id"]))
    if n == "liftoff":
        return "Lifted off"
    if n == "touchdown":
        return "Touchdown, sink rate %.1f%s" % (d["sink"], " (hard)" if d["hard"] else "")
    if n == "bump":
        return "Bumped an obstacle"
    if n == "crashed":
        return "Crash: %s" % CRASH_TEXT.get(d["reason"], d["reason"])
    if n == "map_edge":
        return "Map edge"
    if n == "paused":
        return "Paused"
    if n == "resumed":
        return "Resumed"
    if n == "vertical_trend_changed":
        return TREND_TEXT[d["trend"]]
    return None


class Sim(object):
    """A miniature game loop: a clock with step dt, a ground check, speech, accumulation of displacement."""

    def __init__(self, heli, dt=0.05, speak=True):
        self.h = heli
        self.dt = dt
        self.ts = 0.0
        self.speak = speak
        self.pos = [0.0, 0.0]
        self.ground_check = True
        heli.update(self.ts)                   # anchor the clock

    def say(self, text):
        print("[%6.1f s] %s" % (self.ts, text))

    def tick(self):
        self.ts += self.dt
        evs = self.h.update(self.ts)
        h = self.h
        if self.ground_check and not h.on_ground and h.z <= h.ground_z:
            h.reached_surface()                # the "game" reports touching the ground
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
    """Full start-up by the checklist up to 100% rotor rpm. Returns True on success."""
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
    h.button("starter")                        # press and do not release: it cuts out by itself
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
    """A simple "pilot": altitude, heading, forward speed. Works in any API units."""

    def __init__(self, sim):
        self.sim = sim
        self.alt = 0.0          # m
        self.hdg = 0.0
        self.speed = 0.0        # m/s forward
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
