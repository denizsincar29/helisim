"""Example 4: your own parameters and your own preset; how altitude and load change the characteristics.
Run: python3 04_custom_preset.py"""
from common import Helicopter, HelicopterConfig, PRESETS, clamp


def hover_collective(config, alt, fuel_kg=None):
    """The collective at which the helicopter hovers at altitude alt (meters above sea level), by bisection."""
    lo, hi = 0.0, 1.0
    for i in range(14):
        m = (lo + hi) / 2.0
        h = Helicopter(config, timestamp=0.0, z=alt, fuel_kg=fuel_kg)
        h.ground_z = alt - 5000.0          # the ground is far below: ground effect does not interfere
        h.on_ground = False
        for n in ("battery", "fuel_valve", "fuel_pump", "generator"):
            h.switch(n, True)
        h.switch("rotor_brake", False)
        if h.cfg.has_clutch:
            h.switch("ignition", True)
            h.switch("clutch", True)
        h.engine.state = "running"
        h.engine.n1 = 1.0
        h.nr = 1.0
        h.axis["throttle"] = 1.0
        h.axis["collective"] = m
        h.axis_target["throttle"] = 1.0
        t = 0.0
        vz = 0.0
        n = 0
        for k in range(240):
            t += 0.05
            h.turn("collective", m)
            h.update(t)
            if k >= 140:
                vz += h.vz
                n += 1
        if vz / n > 0:
            hi = m
        else:
            lo = m
    return m, h.p_eng_w / 1000.0, h.nr


# 1. Any parameter can be overridden when creating the config. The keys are in _DEFAULTS in helisim.py.
heavier = HelicopterConfig("medium_turbine", payload_kg=300.0, name="medium_loaded")
print("Load 80 kg -> 300 kg (medium, sea level and 2000 m):")
for label, cfg in (("light   ", HelicopterConfig("medium_turbine")), ("loaded  ", heavier)):
    for alt in (0.0, 2000.0):
        m, p, nr = hover_collective(cfg, alt)
        print("  %s, altitude %4.0f m: hover collective %.2f, power %.0f kW, rpm %.0f%%" % (label, alt, m, p, nr * 100))

# 2. Your own preset: only the differences from the defaults. Register it and use it by name.
PRESETS["cargo_turbine"] = dict(
    name="cargo_turbine", mtow=1800.0, empty_mass=1000.0, payload_kg=250.0, fuel_capacity=350.0,
    max_power_kw=420.0, max_cont_power_kw=360.0, rotor_diameter=11.0, rotor_inertia=1600.0,
    thrust_ratio=1.4, kappa=1.3, profile_power_kw=50.0, drivetrain_eff=0.82, heave_damp=5.0,
    max_pitch_rate=30.0, max_roll_rate=40.0, att_tau=0.4, vne=58.0, cda_h=1.5,
)
print("Own preset cargo_turbine:")
heli = Helicopter("cargo_turbine", timestamp=0.0)
print("  gross mass with payload and full tanks: %.0f kg, rotor diameter %.1f m" % (heli.mass(), heli.cfg.rotor_diameter))
m, p, nr = hover_collective("cargo_turbine", 0.0)
print("  hover collective %.2f, power %.0f kW of %.0f" % (m, p, heli.cfg.max_power_kw * heli.cfg.drivetrain_eff))

# 3. Parameters of a live helicopter can be changed too; then call recalc():
heli.cfg.rotor_diameter = 12.0
heli.cfg.recalc()
print("After changing the diameter: rotor disc area %.1f sq m" % heli.cfg.area)
