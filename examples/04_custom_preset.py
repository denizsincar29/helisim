"""Пример 4: свои параметры и свой пресет; как меняются характеристики с высотой и загрузкой.
Запуск: python3 04_custom_preset.py"""
from common import Helicopter, HelicopterConfig, PRESETS, clamp


def hover_collective(config, alt, fuel_kg=None):
    """Шаг, при котором вертолёт висит на высоте alt (метры над уровнем моря), бисекцией."""
    lo, hi = 0.0, 1.0
    for i in range(14):
        m = (lo + hi) / 2.0
        h = Helicopter(config, timestamp=0.0, z=alt, fuel_kg=fuel_kg)
        h.ground_z = alt - 5000.0          # земли далеко внизу: эффект земли не мешает
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


# 1. Любой параметр можно переопределить при создании конфига. Ключи - в _DEFAULTS в helisim.py.
heavier = HelicopterConfig("medium_turbine", payload_kg=300.0, name="medium_loaded")
print("Загрузка 80 кг -> 300 кг (средний, уровень моря и 2000 м):")
for label, cfg in (("налегке ", HelicopterConfig("medium_turbine")), ("загружен", heavier)):
    for alt in (0.0, 2000.0):
        m, p, nr = hover_collective(cfg, alt)
        print("  %s, высота %4.0f м: шаг на висении %.2f, мощность %.0f кВт, обороты %.0f%%" % (label, alt, m, p, nr * 100))

# 2. Свой пресет: только отличия от значений по умолчанию. Регистрируем и пользуемся по имени.
PRESETS["cargo_turbine"] = dict(
    name="cargo_turbine", mtow=1800.0, empty_mass=1000.0, payload_kg=250.0, fuel_capacity=350.0,
    max_power_kw=420.0, max_cont_power_kw=360.0, rotor_diameter=11.0, rotor_inertia=1600.0,
    thrust_ratio=1.4, kappa=1.3, profile_power_kw=50.0, drivetrain_eff=0.82, heave_damp=5.0,
    max_pitch_rate=30.0, max_roll_rate=40.0, att_tau=0.4, vne=58.0, cda_h=1.5,
)
print("Свой пресет cargo_turbine:")
heli = Helicopter("cargo_turbine", timestamp=0.0)
print("  полная масса с грузом и полными баками: %.0f кг, диаметр винта %.1f м" % (heli.mass(), heli.cfg.rotor_diameter))
m, p, nr = hover_collective("cargo_turbine", 0.0)
print("  шаг на висении %.2f, мощность %.0f кВт из %.0f" % (m, p, heli.cfg.max_power_kw * heli.cfg.drivetrain_eff))

# 3. Параметры можно менять и у живого вертолёта, потом вызвать recalc():
heli.cfg.rotor_diameter = 12.0
heli.cfg.recalc()
print("После правки диаметра: площадь диска %.1f кв. м" % heli.cfg.area)
