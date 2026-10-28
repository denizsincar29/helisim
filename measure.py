"""Measure preset characteristics: python3 measure.py"""
import math
from helisim import Helicopter, density_ratio
import testutil as T

def hover_coll(h, alt):
    """Find the collective that hovers (bisection on the mean vz over 4 s)."""
    lo, hi = 0.0, 1.0
    for _ in range(14):
        m = (lo + hi) / 2
        g = T.airborne_like(h, alt); g.axis["collective"] = m; g.ground_z = -5000.0
        t = g._last_ts; vz = 0.0; n = 0
        for i in range(240):
            t += 0.05; g.turn("collective", m); g.update(t)
            if i >= 140: vz += g.vz; n += 1
        if vz / n > 0: hi = m
        else: lo = m
    return m, g

from helisim import PRESETS
for p in PRESETS:
    h = T.airborne_like(Helicopter(p, timestamp=0.0), 50.0)
    c = h.cfg
    out = {}
    m, g = hover_coll(Helicopter(p, timestamp=0.0), 50.0)
    out["hover_coll_50m"] = round(m, 2)
    out["hover_P_pct"] = round(100 * g.p_eng_w / c.max_power_w, 0)
    out["nr_hover"] = round(g.nr * 100, 1)
    # max speed at full forward stick
    g = T.airborne_like(Helicopter(p, timestamp=0.0), 400.0)
    t = g._last_ts
    for i in range(int(90 / 0.05)):
        t += 0.05
        g.turn("cyclic_pitch", 1.0)
        g.turn("collective", min(1.0, 0.55 + 0.1 * (0 - g.vz)))
        g.update(t)
    out["vmax_ms"] = round(g.airspeed(), 1); out["vne"] = c.vne
    out["vmax_vs_vne"] = round(g.airspeed() / c.vne, 2)
    out["warn_at_vmax"] = g.get_state()["warnings"]
    # max climb (full collective, from a hover) at 50 m
    g = T.airborne_like(Helicopter(p, timestamp=0.0), 50.0)
    t = g._last_ts
    g.ground_z = -5000.0; peak = 0.0
    for i in range(int(12 / 0.05)):
        t += 0.05; g.turn("collective", 1.0); g.update(t); peak = max(peak, g.vz)
    out["climb_peak/12s_ms"] = (round(peak, 1), round(g.vz, 1)); out["nr_full"] = round(g.nr * 100, 1)
    # yaw rate at full pedal
    def yawtest(ped):
        g = T.airborne_like(Helicopter(p, timestamp=0.0), 50.0); g.ground_z = -5000.0; g.axis["collective"] = m
        t = g._last_ts
        for i in range(int(8 / 0.05)):
            t += 0.05; g.turn("collective", m); g.turn("pedals", ped); g.update(t)
        return g
    r_right = yawtest(1.0).r
    g = yawtest(-1.0)
    out["yaw_full_right/left_dps"] = (round(r_right), round(g.r)); out["yaw_pedal0_dps"] = round(yawtest(0.0).r)
    # hover ceiling: the altitude where collective 1.0 no longer holds
    ceil = None
    for alt in range(0, 6001, 500):
        g = T.airborne_like(Helicopter(p, timestamp=0.0), float(alt)); g.ground_z = float(alt) - 5000
        t = g._last_ts
        for i in range(int(12 / 0.05)):
            t += 0.05; g.turn("collective", 1.0); g.update(t)
        if g.vz < 0.3 and ceil is None: ceil = alt
    out["hover_ceiling_m(full coll)"] = ceil
    # rotor spin-up time from a cold start to 98%: see the demo
    print(p, out)
