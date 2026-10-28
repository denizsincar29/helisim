"""
helisim.py - a realistic helicopter flight model with no graphics and no sound.

Contract with the game:
  heli = Helicopter(config)                 # config: preset name, HelicopterConfig, or None
  heli.switch("battery", True)              # toggle switches
  heli.button("starter")                    # push buttons
  heli.turn("collective", 0.6)              # axes (the target; the real position follows with a delay, like a pilot's hand)
  events = heli.update(timestamp_sec)       # every X seconds; returns a list of Event
  heli.reached_surface(ground_z)            # the game reports collisions
  heli.reached_wall(nx, ny, part)
  heli.get_state()                          # snapshot for sound / speech

Units: meters, m/s, degrees, kg, Newtons, Watts. Axes: x - east, y - north, z - up.
Heading: 0 = north, clockwise. Pitch: + nose up. Roll: + right.

The code is deliberately flat (plain classes, dict, float; no dataclasses/generators/lambdas)
so that it can be ported to NVGT/AngelScript almost line by line.
"""
import math
import time

G = 9.80665
RHO0 = 1.225
DEG = math.pi / 180.0
FT = 3.280839895                       # feet per meter

# event data keys whose values are converted when imperial_units is on
_LEN_KEYS = ("x", "y", "z", "agl", "dx", "dy", "dz", "distance")
_SPEED_KEYS = ("ground_speed", "airspeed", "vz", "speed", "sink", "vertical_speed", "vx", "vy")


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def density_ratio(alt):
    """Air density relative to sea level (ISA)."""
    a = clamp(alt, -400.0, 11000.0)
    return (1.0 - 2.2558e-5 * a) ** 4.2559


def angle_diff(a, b):
    d = (a - b + 180.0) % 360.0 - 180.0
    return d


# ---------------------------------------------------------------- configuration

_DEFAULTS = dict(
    name="medium_turbine",
    engine_type="turbine",         # "turbine" | "piston"
    has_clutch=False,              # piston engines: the clutch must be engaged manually
    needs_ignition=False,          # piston engines: magneto (the ignition switch)
    start_needs_pump=True,         # turbine: a boost pump is needed to start
    rotor_diameter=10.2,
    rotor_rpm=394.0,               # nominal main rotor rpm
    rotor_dir=1,                   # +1 counter-clockwise (torque yaws the nose right), -1 clockwise
    rotor_inertia=1200.0,          # kg*m^2
    yaw_inertia=2200.0,
    empty_mass=840.0, mtow=1450.0, payload_kg=80.0, fuel_capacity=280.0,
    max_power_kw=313.0, max_cont_power_kw=270.0, power_lapse=0.85,
    idle_n1=0.62, idle_power_frac=0.12, idle_nr=0.55,
    thrust_ratio=1.55,             # max main rotor thrust / (mtow*g) at sea level, 100% rpm
    min_coll=0.04,                 # fraction of max thrust at zero collective
    heave_damp=4.0,                # inflow in a climb reduces blade angle of attack (damps vertical speed)
    drivetrain_eff=0.85,           # drivetrain/accessory efficiency: engine shaft -> rotor
    kappa=1.2, profile_power_kw=25.0, tail_power_frac=0.08, tail_k=1.2,
    gov_gain=40.0, brake_decel=0.08,
    yaw_damp=2.5, weathercock=3.0, max_yaw_rate=110.0,
    cda_h=1.25, cda_v=1.6, vne=62.0,
    sfc=0.33, idle_fuel_kgh=35.0, low_fuel_frac=0.15,
    cyclic_mode="attitude",        # "attitude" (stick = angle) | "rate" (stick = rotation rate)
    max_cmd_pitch=25.0, max_cmd_roll=30.0, max_pitch_rate=45.0, max_roll_rate=60.0,
    att_gain=2.5, att_tau=0.30, speed_instability=0.15,
    # engine
    light_off_n1=0.15, starter_rate=0.035, starter_cutout=0.45, self_sustain_n1=0.35,
    start_accel=0.045, n1_tau=2.5, n1_decay=0.04, start_throttle_max=0.15,
    egt_limit=810.0,
    # electrics
    battery_ah=17.0, gen_amps=100.0, starter_amps=150.0, starter_min_v=19.0, gen_min_n1=0.6,
    # ground and damage
    ground_mu=0.4, ground_effect=True,
    crash_sink=5.5, hard_sink=3.0, crash_tilt=22.0, crash_ground_speed=10.0, wall_crash_speed=4.0,
    nr_low=0.94, nr_high=1.07, nr_overspeed=1.22,
    # control actuator speeds (fraction of travel per second)
    collective_rate=0.6, throttle_rate=0.35, cyclic_rate=2.5, pedal_rate=2.5,
    # thresholds for change events
    rpm_step=0.02, n1_step=0.02, torque_step=5.0, alt_step=1.0, pos_step=2.0,
    heading_step=5.0, att_step=2.0, speed_step=2.0, vs_step=1.0, fuel_step_frac=0.02,
    max_substep=0.02,
    position_mode="delta",         # "delta": the helicopter does not know the map and sends "moved" events (displacement);
                                   # "absolute": also position_changed + x,y in get_state()
    move_step=0.0,                 # m: accumulate displacement until this threshold (0 = a moved event on every update with motion)
    map_edge_mode="pass",          # "pass": the map edge does not affect physics; "block": cancels speed towards the edge
    max_dt=1.0,                    # protection against time jumps (s); use pause()/resume() for pauses
    imperial_units=False,          # True: feet and ft/s at the API boundary (always SI inside)
    auto_ground=False,             # True: the helicopter detects z<=ground_z itself (for tests without a game)
)

PRESETS = {
    "medium_turbine": dict(thrust_ratio=1.4, kappa=1.3, profile_power_kw=45.0, drivetrain_eff=0.82,
                           heave_damp=5.0),
    "medium_turbine_agile": dict(name="medium_turbine_agile"),   # the previous "lively" tuning: fast climb
    "light_piston": dict(
        name="light_piston", engine_type="piston", has_clutch=True, needs_ignition=True,
        start_needs_pump=False,
        rotor_diameter=7.7, rotor_rpm=530.0, rotor_inertia=110.0, yaw_inertia=420.0,
        empty_mass=400.0, mtow=622.0, payload_kg=80.0, fuel_capacity=60.0,
        max_power_kw=98.0, max_cont_power_kw=93.0, power_lapse=1.0,
        idle_n1=0.40, idle_power_frac=0.10, idle_nr=0.45,
        profile_power_kw=8.0, yaw_damp=3.5, weathercock=4.0,
        cda_h=0.8, cda_v=1.0, vne=50.0, sfc=0.30, idle_fuel_kgh=6.0,
        max_pitch_rate=70.0, max_roll_rate=90.0, att_tau=0.18, att_gain=3.0,
        light_off_n1=0.12, starter_rate=0.08, starter_cutout=0.30, self_sustain_n1=0.0,
        start_accel=0.12, n1_tau=1.2, n1_decay=0.1, egt_limit=700.0,
        battery_ah=14.0, gen_amps=30.0, gen_min_n1=0.5,
        crash_sink=4.5, hard_sink=2.5, wall_crash_speed=3.0,
        collective_rate=0.8, throttle_rate=0.5,
    ),
    "heavy_twin": dict(
        name="heavy_twin", rotor_diameter=21.3, rotor_rpm=192.0, rotor_dir=-1,
        rotor_inertia=40000.0, yaw_inertia=60000.0,
        empty_mass=7200.0, mtow=12000.0, payload_kg=300.0, fuel_capacity=1870.0,
        max_power_kw=2600.0, max_cont_power_kw=2200.0, power_lapse=0.9,
        idle_n1=0.65, thrust_ratio=1.5, profile_power_kw=150.0, tail_power_frac=0.10,
        yaw_damp=2.0, weathercock=2.0, cda_h=3.2, cda_v=8.0, vne=69.0,
        sfc=0.28, idle_fuel_kgh=220.0,
        max_cmd_pitch=18.0, max_cmd_roll=25.0, max_pitch_rate=20.0, max_roll_rate=25.0,
        att_gain=1.6, att_tau=0.6,
        starter_rate=0.02, start_accel=0.03, n1_tau=4.0, egt_limit=880.0,
        battery_ah=50.0, gen_amps=400.0, starter_amps=300.0,
        ground_mu=0.25, crash_sink=5.0, hard_sink=2.8, wall_crash_speed=3.5,
        collective_rate=0.4, throttle_rate=0.25, cyclic_rate=1.5, pedal_rate=1.8,
    ),
}


class HelicopterConfig(object):
    def __init__(self, preset="medium_turbine", **overrides):
        merged = dict(_DEFAULTS)
        merged.update(PRESETS[preset])
        for k in overrides:
            if k not in merged:
                raise KeyError("unknown config key: " + k)
            merged[k] = overrides[k]
        for k in merged:
            setattr(self, k, merged[k])
        self.recalc()

    def recalc(self):
        """Derived values. Call after changing parameters by hand."""
        self.rotor_radius = self.rotor_diameter / 2.0
        self.area = math.pi * self.rotor_radius ** 2
        self.omega0 = self.rotor_rpm * 2.0 * math.pi / 60.0
        self.t_max = self.thrust_ratio * self.mtow * G
        self.max_power_w = self.max_power_kw * 1000.0
        self.max_cont_w = self.max_cont_power_kw * 1000.0
        self.profile_power_w = self.profile_power_kw * 1000.0
        # reference hover (85% mtow, sea level) -> neutral tail rotor thrust
        t_h = 0.85 * self.mtow * G
        vh = math.sqrt(t_h / (2.0 * RHO0 * self.area))
        p_main = self.kappa * t_h * vh + self.profile_power_w
        q_h = p_main / self.omega0
        self.tail_neutral_acc = q_h / self.yaw_inertia / DEG
        self.tail_power_w = self.tail_power_frac * p_main
        self.ke_scale = self.rotor_inertia * self.omega0 ** 2


# ---------------------------------------------------------------- controls

SWITCH_INFO = {
    "battery": "Battery (master)", "generator": "Generator", "fuel_pump": "Fuel pump",
    "fuel_valve": "Fuel valve", "ignition": "Ignition", "avionics": "Avionics",
    "nav_lights": "Navigation lights", "landing_light": "Landing light",
    "anti_collision": "Anti-collision beacon", "rotor_brake": "Rotor brake",
    "clutch": "Clutch", "sas": "Stability augmentation system",
}
BUTTON_INFO = {"starter": "Starter", "engine_stop": "Engine stop"}
AXIS_INFO = {
    "collective": (0.0, 1.0, "Collective"),
    "throttle": (0.0, 1.0, "Throttle (twist grip)"),
    "cyclic_roll": (-1.0, 1.0, "Cyclic roll, right +"),
    "cyclic_pitch": (-1.0, 1.0, "Cyclic pitch, forward +"),
    "pedals": (-1.0, 1.0, "Pedals, right +"),
}


class Event(object):
    def __init__(self, t, name, data):
        self.t = t
        self.name = name
        self.data = data

    def __repr__(self):
        return "[%7.2f] %s %s" % (self.t, self.name, self.data)


# ---------------------------------------------------------------- electrics

class Electrical(object):
    def __init__(self, heli):
        self.h = heli
        self.charge = heli.cfg.battery_ah
        self.voltage = 0.0
        self.gen_online = False

    def step(self, dt):
        h = self.h
        c = h.cfg
        sw = h.sw
        e = h.engine
        self.gen_online = (sw["battery"] and sw["generator"] and e.state == "running"
                           and e.n1 >= c.gen_min_n1)
        has_bat = sw["battery"] and self.charge > 0.02
        h.bus_power = has_bat or self.gen_online
        loads = 0.0
        if h.bus_power:
            loads = 2.0
            if sw["avionics"]:
                loads += 6.0
            if sw["nav_lights"]:
                loads += 2.0
            if sw["landing_light"]:
                loads += 10.0
            if sw["anti_collision"]:
                loads += 3.0
            if sw["fuel_pump"]:
                loads += 4.0
            if h.starter_active:
                loads += c.starter_amps
        gen = c.gen_amps if self.gen_online else 0.0
        if sw["battery"]:
            self.charge = clamp(self.charge + (gen - loads) * dt / 3600.0, 0.0, c.battery_ah)
        if self.gen_online:
            self.voltage = 27.5
        elif h.bus_power:
            self.voltage = 21.0 + 7.0 * self.charge / c.battery_ah
            if h.starter_active:
                self.voltage -= 3.0
        else:
            self.voltage = 0.0


# ---------------------------------------------------------------- engine

class Engine(object):
    """States: off, cranking, starting, running, shutdown, failed."""

    def __init__(self, heli):
        self.h = heli
        self.state = "off"
        self.n1 = 0.0
        self.egt = 15.0
        self.health = 1.0
        self.hot_start_warned = False

    def set_state(self, new):
        if new != self.state:
            old = self.state
            self.state = new
            self.h._ev("engine_state", {"old": old, "new": new})

    def power_available(self, rho_ratio):
        if self.state != "running":
            return 0.0
        c = self.h.cfg
        x = clamp((self.n1 - c.idle_n1) / (1.0 - c.idle_n1), 0.0, 1.0)
        frac = c.idle_power_frac + (1.0 - c.idle_power_frac) * x
        return c.max_power_w * c.drivetrain_eff * frac * (rho_ratio ** c.power_lapse) * self.health

    def step(self, dt):
        h = self.h
        c = h.cfg
        thr = h.axis["throttle"]
        fuel_ok = h.sw["fuel_valve"] and h.fuel_kg > 0.0
        ign_ok = (not c.needs_ignition) or h.sw["ignition"]
        pump_ok = (not c.start_needs_pump) or (h.sw["fuel_pump"] and h.bus_power)
        st = self.state

        # starter
        can_crank = (h.btn["starter"] and h.bus_power and h.elec.voltage >= c.starter_min_v
                     and st in ("off", "cranking", "starting") and self.n1 < c.starter_cutout)
        if h.btn["starter"] and self.n1 >= c.starter_cutout:
            h.btn["starter"] = False
            h._ev("starter_cutout", {"n1": self.n1})
        h.starter_active = can_crank

        if st == "off":
            self.n1 = max(self.n1 - c.n1_decay * dt, 0.0)
            if can_crank:
                self.set_state("cranking")
        elif st == "cranking":
            if can_crank:
                self.n1 += c.starter_rate * dt
            else:
                self.n1 -= c.n1_decay * dt
                if self.n1 <= 0.005:
                    self.n1 = 0.0
                    self.set_state("off")
            if (self.n1 >= c.light_off_n1 and can_crank and fuel_ok and ign_ok and pump_ok
                    and thr <= c.start_throttle_max):
                self.set_state("starting")
                self.hot_start_warned = False
        elif st == "starting":
            if not (fuel_ok and ign_ok):
                h._ev("start_failed", {"reason": "no_fuel_or_ignition"})
                self.set_state("shutdown")
            else:
                rate = c.start_accel
                if can_crank:
                    rate += c.starter_rate
                elif self.n1 < c.self_sustain_n1:
                    rate = -c.n1_decay          # without the starter, below the threshold it cannot sustain
                    if self.n1 <= c.light_off_n1 * 0.9:
                        h._ev("start_failed", {"reason": "hung_start"})
                        self.set_state("shutdown")
                self.n1 += rate * dt
                if thr > c.start_throttle_max:
                    self.health -= 0.05 * dt
                    if not self.hot_start_warned:
                        self.hot_start_warned = True
                        h._ev("hot_start", {"throttle": thr})
                if self.n1 >= c.idle_n1:
                    self.n1 = c.idle_n1
                    self.set_state("running")
                    h._ev("engine_started", {"n1": self.n1})
        elif st == "running":
            target = c.idle_n1 + (1.0 - c.idle_n1) * thr
            self.n1 += (target - self.n1) * (1.0 - math.exp(-dt / c.n1_tau))
            if not (fuel_ok and ign_ok):
                h._ev("engine_flameout", {"reason": "fuel" if not fuel_ok else "ignition"})
                self.set_state("shutdown")
            elif h.btn["engine_stop"]:
                h.btn["engine_stop"] = False
                self.set_state("shutdown")
        elif st in ("shutdown", "failed"):
            self.n1 = max(self.n1 - c.n1_decay * 2.0 * dt, 0.0)
            if st == "shutdown" and self.n1 <= 0.02:
                self.n1 = 0.0
                self.set_state("off")
        if st != "running":
            h.btn["engine_stop"] = False

        # temperature
        if self.state == "running":
            load = h.p_eng_w / max(c.max_power_w * c.drivetrain_eff, 1.0)
            egt_t = 15.0 + 300.0 * self.n1 + 450.0 * load
        elif self.state == "starting":
            frac = clamp((self.n1 - c.light_off_n1) / max(c.idle_n1 - c.light_off_n1, 0.01), 0.0, 1.0)
            egt_t = 15.0 + 520.0 * frac + (300.0 if thr > c.start_throttle_max else 0.0)
        else:
            egt_t = 15.0
        tau = 3.0 if egt_t > self.egt else 25.0
        self.egt += (egt_t - self.egt) * (1.0 - math.exp(-dt / tau))
        if self.egt > c.egt_limit:
            self.health -= 0.01 * dt * (self.egt - c.egt_limit) / 50.0

        # fuel consumption
        flow = 0.0
        if self.state == "running":
            flow = c.idle_fuel_kgh / 3600.0 + c.sfc * (h.p_eng_w / c.drivetrain_eff / 1000.0) / 3600.0
        elif self.state == "starting":
            flow = 0.5 * c.idle_fuel_kgh / 3600.0
        h.fuel_flow_kgh = flow * 3600.0
        if flow > 0.0:
            h.fuel_kg -= flow * dt
            if h.fuel_kg <= 0.0:
                h.fuel_kg = 0.0
                h._ev("fuel_exhausted", {})
        if self.health <= 0.0 and self.state != "failed":
            self.health = 0.0
            h._ev("engine_failure", {"reason": "damage"})
            self.set_state("failed")


# ---------------------------------------------------------------- helicopter

class Helicopter(object):
    def __init__(self, config=None, timestamp=None, x=0.0, y=0.0, z=0.0, heading=0.0,
                 fuel_kg=None, ground_z=None, imperial_units=None):
        """x, y, z, ground_z are in API units (meters or feet, see imperial_units).
        Public attributes (self.x, self.z, self.vx ...) are always in meters; the game should use get_state() and events."""
        if config is None:
            config = HelicopterConfig()
        elif isinstance(config, str):
            config = HelicopterConfig(config)
        self.cfg = config
        c = config
        self.imperial = c.imperial_units if imperial_units is None else bool(imperial_units)
        self.lk = FT if self.imperial else 1.0         # API units per meter
        x = x / self.lk
        y = y / self.lk
        z = z / self.lk
        if ground_z is not None:
            ground_z = ground_z / self.lk
        self.t = 0.0                       # simulation time, s
        self.paused = False
        self._reanchor = False
        self._edge_t = -999.0
        self._last_ts = timestamp
        self.payload_kg = c.payload_kg
        self.fuel_kg = c.fuel_capacity if fuel_kg is None else fuel_kg
        self.fuel_flow_kgh = 0.0
        self.x = x
        self.y = y
        self.z = z
        self.ground_z = z if ground_z is None else ground_z
        self.heading = heading % 360.0
        self.pitch = 0.0
        self.roll = 0.0
        self.q = 0.0                       # pitch rate, deg/s (+ nose up)
        self.p = 0.0                       # roll rate
        self.r = 0.0                       # yaw rate (+ right)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wind = [0.0, 0.0, 0.0]        # reserved for the future (east, north, up; m/s)
        self.on_ground = True
        self.crashed = False
        self.crash_reason = ""
        self.nr = 0.0                      # main rotor rpm, fraction of nominal
        self.p_eng_w = 0.0
        self.thrust = 0.0
        self.torque_pct = 0.0
        self.bus_power = False
        self.starter_active = False
        self.airframe_health = 1.0
        self.overspeed_t = 0.0
        self._ot_t = 0.0
        self._vrs = 0.0
        self._rbs = 0.0
        self._events = []
        self.warn = {}
        self._subs = {}
        self.sw = {}
        for k in SWITCH_INFO:
            self.sw[k] = False
        self.sw["rotor_brake"] = True      # parked: the brake is on
        self.sw["sas"] = True
        self.btn = {"starter": False, "engine_stop": False}
        self._btn_timer = {}
        self.axis = {}
        self.axis_target = {}
        for k in AXIS_INFO:
            self.axis[k] = 0.0
            self.axis_target[k] = 0.0
        self.elec = Electrical(self)
        self.engine = Engine(self)
        self._rep = {}
        self._reset_reports()

    # ------------------------------------------------------------ events
    def _out(self, data):
        """Convert lengths and speeds to API units (returns a copy of the dict)."""
        if self.lk == 1.0:
            return data
        d = dict(data)
        for k in d:
            if k in _LEN_KEYS or k in _SPEED_KEYS:
                d[k] = d[k] * self.lk
        return d

    def _ev(self, name, data):
        e = Event(self.t, name, self._out(data))
        self._events.append(e)
        subs = self._subs.get(name, []) + self._subs.get("*", [])
        for fn in subs:
            fn(e)

    def subscribe(self, name, fn):
        """name is an event name or "*" for all of them. Alternative: just read the list returned by update()."""
        if name not in self._subs:
            self._subs[name] = []
        self._subs[name].append(fn)

    def pop_events(self):
        ev = self._events
        self._events = []
        return ev

    def _warn(self, wid, active):
        if self.warn.get(wid, False) != active:
            self.warn[wid] = active
            self._ev("warning", {"id": wid, "active": active})

    # ------------------------------------------------------------ control
    def switch(self, name, on):
        if name not in self.sw:
            return self._reject(name, "unknown")
        c = self.cfg
        if (name == "ignition" and not c.needs_ignition) or (name == "clutch" and not c.has_clutch):
            return self._reject(name, "not_installed")
        if self.sw[name] != bool(on):
            self.sw[name] = bool(on)
            self._ev("switch", {"name": name, "on": bool(on)})
            self.elec.step(0.0)             # bus power is available immediately, it does not wait for the next update()
        return True

    def toggle(self, name):
        if name not in self.sw:
            return self._reject(name, "unknown")
        return self.switch(name, not self.sw[name])

    def press(self, name):
        if name not in self.btn:
            return self._reject(name, "unknown")
        if self.crashed:
            return self._reject(name, "crashed")
        if name == "starter":
            if not self.bus_power:
                return self._reject(name, "no_electric_power")
            if self.engine.state == "running":
                return self._reject(name, "engine_running")
        if name == "engine_stop" and self.engine.state != "running":
            return self._reject(name, "engine_not_running")
        if not self.btn[name]:
            self.btn[name] = True
            self._ev("button", {"name": name, "pressed": True})
        return True

    def release(self, name):
        if name in self.btn and self.btn[name]:
            self.btn[name] = False
            self._ev("button", {"name": name, "pressed": False})

    def button(self, name, hold=0.0):
        """Press; if hold>0, release after hold simulated seconds.
        The starter can simply be pressed and never released - it cuts out by itself at the right rpm."""
        ok = self.press(name)
        if ok and hold > 0.0:
            self._btn_timer[name] = self.t + hold
        return ok

    def turn(self, name, value):
        """Set an axis (stick/lever/pedals). The real position follows the target at a finite speed."""
        if name not in AXIS_INFO:
            return self._reject(name, "unknown")
        lo = AXIS_INFO[name][0]
        hi = AXIS_INFO[name][1]
        self.axis_target[name] = clamp(value, lo, hi)
        return True

    def step(self, name, delta):
        """Move an axis by delta (handy for keyboard control)."""
        if name not in AXIS_INFO:
            return self._reject(name, "unknown")
        return self.turn(name, self.axis_target[name] + delta)

    def joystick(self, roll, pitch):
        """Cyclic: roll + is right, pitch + is forward (nose down)."""
        self.turn("cyclic_roll", roll)
        self.turn("cyclic_pitch", pitch)

    def center(self):
        self.turn("cyclic_roll", 0.0)
        self.turn("cyclic_pitch", 0.0)
        self.turn("pedals", 0.0)

    def _reject(self, name, reason):
        self._ev("command_rejected", {"name": name, "reason": reason})
        return False

    def controls_info(self):
        """List of all controls, for building menus and speech."""
        out = []
        for k in SWITCH_INFO:
            if (k == "ignition" and not self.cfg.needs_ignition) or (k == "clutch" and not self.cfg.has_clutch):
                continue
            out.append({"name": k, "kind": "switch", "label": SWITCH_INFO[k], "value": self.sw[k]})
        for k in BUTTON_INFO:
            out.append({"name": k, "kind": "button", "label": BUTTON_INFO[k], "value": self.btn[k]})
        for k in AXIS_INFO:
            out.append({"name": k, "kind": "axis", "label": AXIS_INFO[k][2], "min": AXIS_INFO[k][0],
                        "max": AXIS_INFO[k][1], "value": self.axis[k], "target": self.axis_target[k]})
        return out

    # ------------------------------------------------------------ update
    def update(self, timestamp=None):
        if timestamp is None:
            timestamp = time.monotonic()
        if self.paused:
            return self.pop_events()        # time does not run; commands only queue events
        if self._last_ts is None or self._reanchor:
            self._last_ts = timestamp        # first call, or first after resume: only anchor the clock
            self._reanchor = False
            return self.pop_events()
        dt = timestamp - self._last_ts
        self._last_ts = timestamp
        if dt <= 0.0:
            return self.pop_events()
        if dt > self.cfg.max_dt:
            dt = self.cfg.max_dt             # lag spike: do not blow up the integrator (for pauses use pause()/resume())
        if self.crashed:
            self.t += dt
            return self.pop_events()
        n = int(math.ceil(dt / self.cfg.max_substep))
        h = dt / n
        for i in range(n):
            self._step(h)
            if self.crashed:
                break
        self._emit_changes()
        return self.pop_events()

    def pause(self, reason=""):
        """Freeze the helicopter (map reload, cutscene, menu). Simulation time stands still."""
        if not self.paused:
            self.paused = True
            self._ev("paused", {"reason": reason})

    def resume(self, timestamp=None):
        """Resume. The paused time does not count. Pass timestamp if you keep the clock yourself."""
        if self.paused:
            self.paused = False
            if timestamp is None:
                self._reanchor = True        # anchor to the clock on the next update()
            else:
                self._last_ts = timestamp
                self._reanchor = False
            self._ev("resumed", {})

    def reset_clock(self, timestamp=None):
        """Re-anchor the clock (without pausing), e.g. after a long load."""
        if timestamp is None:
            self._reanchor = True
        else:
            self._last_ts = timestamp
            self._reanchor = False

    def _step(self, dt):
        self.t += dt
        self._move_axes(dt)
        for name in list(self._btn_timer.keys()):
            if self.t >= self._btn_timer[name]:
                del self._btn_timer[name]
                self.release(name)
        self.elec.step(dt)
        self.engine.step(dt)
        self._flight(dt)
        self._check_warnings(dt)

    def _move_axes(self, dt):
        c = self.cfg
        rates = {"collective": c.collective_rate, "throttle": c.throttle_rate,
                 "cyclic_roll": c.cyclic_rate, "cyclic_pitch": c.cyclic_rate, "pedals": c.pedal_rate}
        for k in self.axis:
            d = self.axis_target[k] - self.axis[k]
            m = rates[k] * dt
            self.axis[k] += clamp(d, -m, m)

    def mass(self):
        return self.cfg.empty_mass + self.payload_kg + self.fuel_kg

    # ------------------------------------------------------------ aerodynamics
    def _induced(self, T, rho, vc, vxd):
        c = self.cfg
        vh = math.sqrt(max(T, 0.0) / (2.0 * rho * c.area))
        if vh < 0.05:
            return 0.0, vh
        w = clamp(vxd / (0.8 * vh + 0.5), 0.0, 1.0)
        if vc >= 0.0:
            vi_ax = -vc / 2.0 + math.sqrt(vc * vc / 4.0 + vh * vh)
        else:
            x = vc / vh
            if x >= -2.0:
                vi_ax = vh * (1.0 - 1.125 * x - 1.372 * x * x - 1.718 * x ** 3 - 0.655 * x ** 4)
            else:
                vi_ax = -vc / 2.0 - math.sqrt(vc * vc / 4.0 - vh * vh)
        if w <= 0.0:
            return vi_ax, vh
        vi = vh
        for i in range(6):                 # Glauert: forward speed reduces the induced velocity
            d = math.sqrt(vxd * vxd + (vc + vi) ** 2)
            d = max(d, 0.3 * vh)
            vi = 0.5 * vi + 0.5 * vh * vh / d
        return (1.0 - w) * vi_ax + w * vi, vh

    def _flight(self, dt):
        c = self.cfg
        mass = self.mass()
        weight = mass * G
        rr = density_ratio(self.z)
        rho = RHO0 * rr
        nr = self.nr
        coll = self.axis["collective"]
        ped = self.axis["pedals"]
        cp = self.axis["cyclic_pitch"]
        cr = self.axis["cyclic_roll"]

        # rotor disc orientation (thrust vector)
        psi = self.heading * DEG
        th = self.pitch * DEG
        ph = self.roll * DEG
        fx = math.sin(psi)
        fy = math.cos(psi)
        rx = math.cos(psi)
        ry = -math.sin(psi)
        ufw = -math.sin(th) * math.cos(ph)
        urt = math.sin(ph)
        uz = math.cos(th) * math.cos(ph)
        ux = ufw * fx + urt * rx
        uy = ufw * fy + urt * ry
        nn = math.sqrt(ux * ux + uy * uy + uz * uz)
        ux /= nn
        uy /= nn
        uz /= nn

        # velocity relative to the air
        vax = self.vx - self.wind[0]
        vay = self.vy - self.wind[1]
        vaz = self.vz - self.wind[2]
        vc = vax * ux + vay * uy + vaz * uz
        v2 = vax * vax + vay * vay + vaz * vaz
        vxd = math.sqrt(max(v2 - vc * vc, 0.0))
        agl = self.z - self.ground_z
        ge = clamp(1.0 - agl / c.rotor_diameter, 0.0, 1.0) if c.ground_effect else 0.0

        # thrust
        t0 = c.t_max * (c.min_coll + (1.0 - c.min_coll) * coll) * nr * nr * rr
        stall_lo = 1.0 if nr >= 0.75 else (nr / 0.75) ** 2
        vh0 = math.sqrt(t0 / (2.0 * rho * c.area)) if t0 > 1.0 else 0.0
        vrs = 0.0                           # vortex ring state
        if vc < 0.0 and vh0 > 1.0:
            x = -vc / vh0
            if x < 1.0:
                bump = clamp((x - 0.3) / 0.5, 0.0, 1.0)
            else:
                bump = clamp(2.0 - x, 0.0, 1.0)
            vrs = bump * clamp(1.0 - vxd / (0.9 * vh0 + 1.0), 0.0, 1.0)
        loadf = max(t0 / weight, 0.6)       # retreating blade stall
        vcrit = 1.02 * c.vne * math.sqrt(rr) / (1.0 + 0.4 * (loadf - 1.0))
        rbs = clamp((vxd - vcrit) / (0.15 * c.vne), 0.0, 1.0)
        self._vrs = vrs
        self._rbs = rbs
        infl = clamp(1.0 - c.heave_damp * vc / (c.omega0 * c.rotor_radius * max(nr, 0.3)), 0.5, 1.3)
        T = t0 * stall_lo * infl * (1.0 + 0.12 * ge) * (1.0 - 0.4 * vrs) * (1.0 - 0.3 * rbs)
        T = max(T, 0.0)
        self.thrust = T

        # power required by the rotor
        vi, vh = self._induced(T, rho, vc, vxd)
        vi *= (1.0 - 0.35 * ge)
        mu = vxd / max(c.omega0 * c.rotor_radius * nr, 1.0)
        p_ind = c.kappa * T * (vc + vi)
        p_prof = c.profile_power_w * rr * nr ** 3 * (1.0 + 4.5 * mu * mu) * (1.0 + 1.5 * rbs)
        p_tail = c.tail_power_w * (1.0 - c.rotor_dir * c.tail_k * ped) * nr ** 3
        p_load = p_ind + p_prof + p_tail

        # engine -> rotor (rpm governor + freewheel unit)
        eng = self.engine
        drives = eng.state == "running" and (self.sw["clutch"] or not c.has_clutch)
        target_nr = c.idle_nr + (1.0 - c.idle_nr) * self.axis["throttle"]
        p_av = eng.power_available(rr)
        p_eng = 0.0
        if drives:
            p_eng = p_av * clamp(c.gov_gain * (target_nr - nr), 0.0, 1.0)
        p_brake = c.ke_scale * c.brake_decel * nr if self.sw["rotor_brake"] else 0.0
        self.p_eng_w = p_eng
        dnr = (p_eng - p_load - p_brake) / (c.ke_scale * max(nr, 0.05))
        self.nr = max(nr + dnr * dt, 0.0)
        self.torque_pct = 100.0 * p_eng / (c.max_cont_w * c.drivetrain_eff * max(nr, 0.2))

        # yaw
        omega = max(nr, 0.2) * c.omega0
        torque_acc = max(p_eng - p_tail, 0.0) / omega / c.yaw_inertia / DEG
        tail_acc = c.tail_neutral_acc * (1.0 - c.rotor_dir * c.tail_k * ped) * nr * nr
        yaw_acc = c.rotor_dir * (torque_acc - tail_acc)
        vright = vax * rx + vay * ry
        yaw_acc += c.weathercock * vright * clamp(math.sqrt(vax * vax + vay * vay) / 25.0, 0.0, 1.0)
        thrust_up = T * uz
        normal = max(weight - thrust_up, 0.0) if self.on_ground else 0.0
        damp = c.yaw_damp * (1.0 + 5.0 * normal / weight)
        yaw_acc -= damp * self.r
        self.r = clamp(self.r + yaw_acc * dt, -c.max_yaw_rate, c.max_yaw_rate)
        self.heading = (self.heading + self.r * dt) % 360.0

        # pitch/roll
        lightness = 1.0
        if self.on_ground:
            lightness = clamp((thrust_up / weight - 0.85) / 0.15, 0.0, 1.0)
        auth = nr * nr * lightness if self.on_ground else nr * nr
        vfwd = vax * fx + vay * fy
        mu_term = 0.0 if self.sw["sas"] else c.speed_instability * vfwd
        if c.cyclic_mode == "attitude":
            cmd_p = (-cp * c.max_cmd_pitch + mu_term) * lightness
            cmd_r = cr * c.max_cmd_roll * lightness
            q_t = clamp(c.att_gain * (cmd_p - self.pitch), -c.max_pitch_rate, c.max_pitch_rate)
            p_t = clamp(c.att_gain * (cmd_r - self.roll), -c.max_roll_rate, c.max_roll_rate)
        else:
            q_t = -cp * c.max_pitch_rate + mu_term
            p_t = cr * c.max_roll_rate
        q_t += 40.0 * rbs                   # stall: pitch-up and roll-off
        p_t -= 30.0 * rbs
        p_t += 15.0 * vrs * math.sin(7.0 * self.t)
        k = 1.0 - math.exp(-dt / c.att_tau)
        self.q += (q_t * auth - self.q) * k
        self.p += (p_t * auth - self.p) * k
        self.pitch = clamp(self.pitch + self.q * dt, -80.0, 80.0)
        self.roll = clamp(self.roll + self.p * dt, -80.0, 80.0)
        if self.on_ground and c.cyclic_mode == "rate":
            self.pitch -= self.pitch * (1.0 - lightness) * 3.0 * dt
            self.roll -= self.roll * (1.0 - lightness) * 3.0 * dt

        # forces
        hs = math.sqrt(vax * vax + vay * vay)
        fxn = T * ux - 0.5 * rho * c.cda_h * hs * vax
        fyn = T * uy - 0.5 * rho * c.cda_h * hs * vay
        fzn = T * uz - weight - 0.5 * rho * c.cda_v * abs(vaz) * vaz

        if self.on_ground and fzn > 0.0:
            self.on_ground = False
            self._ev("liftoff", {"agl": agl, "nr": self.nr})
        if self.on_ground:
            normal = max(weight - thrust_up, 0.0)
            fr = c.ground_mu * normal / mass * dt
            nvx = self.vx + fxn / mass * dt
            nvy = self.vy + fyn / mass * dt
            sp = math.sqrt(nvx * nvx + nvy * nvy)
            if sp <= fr:
                self.vx = 0.0
                self.vy = 0.0
            else:
                kf = (sp - fr) / sp
                self.vx = nvx * kf
                self.vy = nvy * kf
            self.vz = 0.0
            self.z = self.ground_z
        else:
            self.vx += fxn / mass * dt
            self.vy += fyn / mass * dt
            self.vz += fzn / mass * dt
            self.z += self.vz * dt
        self.x += self.vx * dt
        self.y += self.vy * dt

        if c.auto_ground and not self.on_ground and self.z <= self.ground_z:
            self.reached_surface(self.ground_z)

    # ------------------------------------------------------------ warnings and damage
    def airspeed(self):
        vax = self.vx - self.wind[0]
        vay = self.vy - self.wind[1]
        return math.sqrt(vax * vax + vay * vay)

    def _check_warnings(self, dt):
        c = self.cfg
        e = self.engine
        asp = self.airspeed()
        self._warn("low_rotor_rpm", (not self.on_ground) and 0.3 < self.nr < c.nr_low)
        self._warn("high_rotor_rpm", self.nr > c.nr_high)
        if self.torque_pct > 100.0 and self.nr > 0.97:
            self._ot_t += dt                  # a short peak (spool-up, a collective yank) is normal
        else:
            self._ot_t = max(self._ot_t - 2.0 * dt, 0.0)
        self._warn("overtorque", self._ot_t > 3.0)
        self._warn("overtemp", e.egt > c.egt_limit)
        self._warn("low_fuel", self.fuel_kg < c.low_fuel_frac * c.fuel_capacity and e.state == "running")
        self._warn("low_battery", self.sw["battery"] and self.elec.charge < 0.25 * c.battery_ah)
        self._warn("vortex_ring", self._vrs > 0.15)
        self._warn("retreating_blade_stall", self._rbs > 0.08)
        self._warn("overspeed", asp > c.vne)
        self._warn("rotor_brake_on", self.sw["rotor_brake"] and e.state == "running"
                   and self.axis["throttle"] > 0.3)
        if asp > 1.15 * c.vne:
            self.overspeed_t += dt
            if self.overspeed_t > 3.0:
                self._crash("structural_failure", {"airspeed": asp})
        else:
            self.overspeed_t = max(self.overspeed_t - dt, 0.0)
        if self.nr > c.nr_overspeed and not self.crashed:
            self._crash("rotor_overspeed", {"nr": self.nr})

    # ------------------------------------------------------------ collisions (reported by the game)
    def _crash(self, reason, data):
        if self.crashed:
            return
        self.crashed = True
        self.crash_reason = reason
        data = dict(data)
        data["reason"] = reason
        if self.cfg.position_mode == "absolute":
            data["x"] = self.x
            data["y"] = self.y
        data["z"] = self.z
        self.vx = self.vy = self.vz = 0.0
        self.p = self.q = self.r = 0.0
        self.nr = 0.0
        self.p_eng_w = 0.0
        self.engine.state = "failed"
        self.starter_active = False
        self._ev("crashed", data)

    def reached_surface(self, ground_z=None):
        """Game: the gear/fuselage touched a surface. If already on the ground, only updates the ground height."""
        if self.crashed:
            return
        c = self.cfg
        if ground_z is not None:
            self.ground_z = ground_z / self.lk
        if self.on_ground:
            self.z = self.ground_z
            return
        sink = max(-self.vz, 0.0)
        hs = math.sqrt(self.vx * self.vx + self.vy * self.vy)
        tilt = max(abs(self.pitch), abs(self.roll))
        info = {"sink": sink, "ground_speed": hs, "tilt": tilt}
        if sink > c.crash_sink:
            self._crash("hard_landing", info)
        elif tilt > c.crash_tilt and sink > 0.5:
            self._crash("tilted_landing", info)
        elif hs > c.crash_ground_speed:
            self._crash("fast_landing", info)
        else:
            self.on_ground = True
            self.z = self.ground_z
            self.vz = 0.0
            hard = sink > c.hard_sink
            if hard:
                self.airframe_health -= 0.3 * (sink - c.hard_sink) / (c.crash_sink - c.hard_sink)
                self._warn("hard_landing", True)
                self._warn("hard_landing", False)
            info["hard"] = hard
            self._ev("touchdown", info)

    def surface_lost(self):
        """Game: the surface under the helicopter is gone (a cliff, a roof edge) - it is airborne again."""
        if self.on_ground and not self.crashed:
            self.on_ground = False
            self._ev("left_surface", {})

    def set_ground_level(self, ground_z):
        """Game: height of the surface under the helicopter (for ground effect and AGL height)."""
        self.ground_z = ground_z / self.lk

    def set_wind(self, east, north, up=0.0):
        """Wind in API units (for the future; speeds are already computed relative to the air)."""
        self.wind = [east / self.lk, north / self.lk, up / self.lk]

    def reached_wall(self, nx=None, ny=None, part="fuselage"):
        """Game: hit a wall/obstacle. (nx, ny) is the wall normal pointing towards the helicopter.
        part: "fuselage" | "rotor" | "tail"."""
        if self.crashed:
            return
        c = self.cfg
        hs = math.sqrt(self.vx * self.vx + self.vy * self.vy)
        if nx is None or ny is None:
            closing = hs
            if hs > 1e-6:
                nx = -self.vx / hs
                ny = -self.vy / hs
            else:
                nx = ny = 0.0
        else:
            closing = -(self.vx * nx + self.vy * ny)
        info = {"speed": closing, "part": part}
        if part == "rotor" and self.nr > 0.15:
            self._crash("rotor_strike", info)
        elif part == "tail" and closing > 1.5 and self.nr > 0.3:
            self._crash("tail_strike", info)
        elif closing > c.wall_crash_speed:
            self._crash("wall_impact", info)
        elif closing > 0.0:
            self.vx += 1.2 * closing * nx
            self.vy += 1.2 * closing * ny
            self.airframe_health -= 0.02 * closing
            self._ev("bump", info)

    def reached_map_edge(self, nx=None, ny=None, mode=None):
        """Game: the helicopter is at the map edge. The helicopter does not know the map.
        mode "pass" (the config default): physics is unchanged, only a map_edge event is emitted -
          the game moves the player to another map itself (the helicopter does not need to know).
        mode "block": cancels the velocity component towards the edge (the normal (nx, ny) points into the map;
          without a normal it cancels all horizontal speed). Call it every frame while pushing against the edge."""
        if self.crashed:
            return
        mode = mode or self.cfg.map_edge_mode
        if mode == "block":
            if nx is None or ny is None:
                self.vx = 0.0
                self.vy = 0.0
            else:
                closing = -(self.vx * nx + self.vy * ny)
                if closing > 0.0:
                    self.vx += closing * nx
                    self.vy += closing * ny
        if self.t - self._edge_t >= 1.0:       # do not spam the event if it is called every frame
            self._edge_t = self.t
            self._ev("map_edge", {"mode": mode})

    def fail_engine(self, reason="failure"):
        """Game: an engine failure from outside (a hit, a bird strike, a script trigger). Autorotation follows."""
        if self.engine.state != "failed" and not self.crashed:
            self.engine.health = 0.0
            self._ev("engine_failure", {"reason": reason})
            self.engine.set_state("failed")

    def reached_ceiling(self):
        if self.crashed:
            return
        if self.vz > 1.0 and self.nr > 0.15:
            self._crash("rotor_strike", {"speed": self.vz, "part": "rotor"})
        elif self.vz > 0.0:
            self.vz = -0.2 * self.vz
            self._ev("bump", {"speed": self.vz, "part": "rotor"})

    def reached_water(self):
        self._crash("water", {"speed": math.sqrt(self.vx ** 2 + self.vy ** 2 + self.vz ** 2)})

    def collision(self, kind, **kw):
        """Single entry point: kind = surface|wall|ceiling|water|rotor|tail."""
        if kind == "surface":
            self.reached_surface(kw.get("ground_z"))
        elif kind == "wall":
            self.reached_wall(kw.get("nx"), kw.get("ny"), kw.get("part", "fuselage"))
        elif kind == "rotor":
            self.reached_wall(kw.get("nx"), kw.get("ny"), "rotor")
        elif kind == "tail":
            self.reached_wall(kw.get("nx"), kw.get("ny"), "tail")
        elif kind == "ceiling":
            self.reached_ceiling()
        elif kind == "water":
            self.reached_water()

    def repair(self):
        """After a crash: restore to working order on the spot (standing on the ground)."""
        self.crashed = False
        self.crash_reason = ""
        self.airframe_health = 1.0
        self.engine = Engine(self)
        self.nr = 0.0
        self.on_ground = True
        self.z = self.ground_z
        self.pitch = self.roll = 0.0
        self.warn = {}
        self.sw["rotor_brake"] = True

    # ------------------------------------------------------------ changes -> events
    def _reset_reports(self):
        self._rep = {"nr": 0.0, "n1": 0.0, "torque": 0.0, "z": self.z, "x": self.x, "y": self.y,
                     "heading": self.heading, "pitch": 0.0, "roll": 0.0, "gs": 0.0, "vs": 0.0,
                     "fuel": self.fuel_kg, "trend": "level", "bus": self.bus_power,
                     "mx": self.x, "my": self.y, "mz": self.z}

    def _emit_changes(self):
        c = self.cfg
        r = self._rep
        if abs(self.nr - r["nr"]) >= c.rpm_step:
            r["nr"] = self.nr
            self._ev("rotor_rpm_changed", {"nr": self.nr})
        if abs(self.engine.n1 - r["n1"]) >= c.n1_step:
            r["n1"] = self.engine.n1
            self._ev("engine_rpm_changed", {"n1": self.engine.n1})
        if abs(self.torque_pct - r["torque"]) >= c.torque_step:
            r["torque"] = self.torque_pct
            self._ev("torque_changed", {"torque_pct": self.torque_pct})
        if abs(self.z - r["z"]) >= c.alt_step:
            r["z"] = self.z
            self._ev("altitude_changed", {"z": self.z, "agl": self.z - self.ground_z})
        # displacement since the last report: the game moves the player by it; the helicopter does not know the map
        mdx = self.x - r["mx"]
        mdy = self.y - r["my"]
        mdz = self.z - r["mz"]
        hd = math.sqrt(mdx * mdx + mdy * mdy)
        if (hd >= 1e-4 or abs(mdz) >= 1e-4) and hd >= c.move_step:
            if hd >= 1e-4:
                bearing = math.atan2(mdx, mdy) / DEG % 360.0
            else:
                bearing = self.heading
            self._ev("moved", {"dx": mdx, "dy": mdy, "dz": mdz, "distance": hd,
                               "bearing": bearing,
                               "relative_bearing": angle_diff(bearing, self.heading),
                               "elevation": math.atan2(mdz, hd) / DEG,
                               "heading": self.heading})
            r["mx"] = self.x
            r["my"] = self.y
            r["mz"] = self.z
        if c.position_mode == "absolute":
            dx = self.x - r["x"]
            dy = self.y - r["y"]
            if dx * dx + dy * dy >= c.pos_step * c.pos_step:
                r["x"] = self.x
                r["y"] = self.y
                self._ev("position_changed", {"x": self.x, "y": self.y})
        if abs(angle_diff(self.heading, r["heading"])) >= c.heading_step:
            r["heading"] = self.heading
            self._ev("heading_changed", {"heading": self.heading})
        if abs(self.pitch - r["pitch"]) >= c.att_step or abs(self.roll - r["roll"]) >= c.att_step:
            r["pitch"] = self.pitch
            r["roll"] = self.roll
            self._ev("attitude_changed", {"pitch": self.pitch, "roll": self.roll})
        gs = math.sqrt(self.vx * self.vx + self.vy * self.vy)
        if abs(gs - r["gs"]) >= c.speed_step:
            r["gs"] = gs
            self._ev("speed_changed", {"ground_speed": gs, "airspeed": self.airspeed()})
        if abs(self.vz - r["vs"]) >= c.vs_step:
            r["vs"] = self.vz
            self._ev("vertical_speed_changed", {"vz": self.vz})
        trend = r["trend"]
        if self.vz > 0.6:
            nt = "climb"
        elif self.vz < -0.6:
            nt = "descent"
        elif abs(self.vz) < 0.3:
            nt = "level"
        else:
            nt = trend
        if nt != trend:
            r["trend"] = nt
            self._ev("vertical_trend_changed", {"trend": nt, "vz": self.vz})
        if abs(self.fuel_kg - r["fuel"]) >= c.fuel_step_frac * c.fuel_capacity:
            r["fuel"] = self.fuel_kg
            self._ev("fuel_changed", {"fuel_kg": self.fuel_kg})
        if self.bus_power != r["bus"]:
            r["bus"] = self.bus_power
            self._ev("bus_power_changed", {"on": self.bus_power})

    # ------------------------------------------------------------ state snapshot
    def get_state(self):
        c = self.cfg
        gs = math.sqrt(self.vx * self.vx + self.vy * self.vy)
        k = self.lk
        st = {
            "units": {"length": "ft" if self.imperial else "m", "speed": "ft/s" if self.imperial else "m/s"},
            "t": self.t, "paused": self.paused,
            "z": self.z * k, "agl": (self.z - self.ground_z) * k,
            "heading": self.heading, "pitch": self.pitch, "roll": self.roll,
            "pitch_rate": self.q, "roll_rate": self.p, "yaw_rate": self.r,
            "vx": self.vx * k, "vy": self.vy * k, "vz": self.vz * k, "ground_speed": gs * k,
            "airspeed": self.airspeed() * k, "vertical_speed": self.vz * k,
            "on_ground": self.on_ground, "crashed": self.crashed, "crash_reason": self.crash_reason,
            "engine_state": self.engine.state, "n1_pct": self.engine.n1 * 100.0,
            "nr_pct": self.nr * 100.0, "torque_pct": self.torque_pct, "egt": self.engine.egt,
            "engine_health": self.engine.health, "airframe_health": self.airframe_health,
            "fuel_kg": self.fuel_kg, "fuel_flow_kgh": self.fuel_flow_kgh,
            "mass": self.mass(), "thrust": self.thrust,
            "bus_power": self.bus_power, "voltage": self.elec.voltage,
            "battery_pct": 100.0 * self.elec.charge / c.battery_ah,
            "generator_online": self.elec.gen_online,
            "controls": dict(self.axis), "targets": dict(self.axis_target),
            "switches": dict(self.sw), "buttons": dict(self.btn),
            "warnings": [k for k in self.warn if self.warn[k]],
        }
        if c.position_mode == "absolute":
            st["x"] = self.x * k
            st["y"] = self.y * k
        return st
