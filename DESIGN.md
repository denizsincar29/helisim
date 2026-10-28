# helisim - design notes

How to use the module: see `README.md` and `examples/`. This file describes how the model is built, what events
exist, how it was tuned, and what to watch for when porting it to NVGT.

Files: `helisim.py` (the module), `testutil.py` (helpers), `test_scenarios.py` and `test_features.py` (scenario scripts
that print what happens), `demo_flight.py` (start-up, takeoff, flight, landing for every preset), `measure.py`
(measures the presets).

## How the game talks to the module

```python
heli = Helicopter("medium_turbine", timestamp=t0, z=ground)   # or HelicopterConfig(preset, **overrides)
heli.switch("battery", True)        # toggles: battery generator fuel_pump fuel_valve ignition* avionics
heli.toggle("rotor_brake")          #   nav_lights landing_light anti_collision rotor_brake clutch* sas
heli.button("starter")              # buttons: starter (cuts out by itself at the right rpm), engine_stop
heli.turn("collective", 0.6)        # axes: collective, throttle (0..1); cyclic_roll, cyclic_pitch, pedals (-1..1)
heli.step("pedals", -0.1)           # move by a delta (handy for a keyboard)
heli.joystick(roll, pitch)          # the cyclic in one call
events = heli.update(timestamp)     # every X seconds; a list of Event(t, name, data)
heli.get_state()                    # a dict for sound and speech
heli.controls_info()                # the list of controls with labels, for building menus
```

`*` means "only if the helicopter has it": ignition and clutch exist on the piston helicopter only.
Axes move towards the requested value at a limited speed, like a pilot's hand: `axis` is the real position and
`targets` is the requested one.

### Messages from the game
`reached_surface(ground_z)`, `reached_wall(nx, ny, part)` (`part`: fuselage, rotor or tail), `reached_ceiling()`,
`reached_water()`, `surface_lost()`, `set_ground_level(z)`, or the single entry point `collision(kind, ...)`.
The module decides the outcome from speed and angle: `touchdown`, `bump` or `crashed`
(`crashed=True`, the cause is in `crash_reason`: hard_landing, tilted_landing, fast_landing, wall_impact,
rotor_strike, tail_strike, water, rotor_overspeed, structural_failure).
`fail_engine(reason)` fails the engine from outside. `repair()` restores the helicopter after a crash.
`auto_ground=True` makes the helicopter catch `z <= ground_z` itself, for tests without a game.

## Events

Discrete: `switch`, `button`, `command_rejected{name, reason}`, `paused`, `resumed`, `map_edge`,
`engine_state{old, new}`, `engine_started`, `starter_cutout`, `start_failed`, `hot_start`, `engine_flameout`,
`engine_failure`, `fuel_exhausted`, `liftoff`, `touchdown`, `left_surface`, `bump`, `crashed`, `bus_power_changed`.

Warnings: `warning{id, active}`. The ids are low_rotor_rpm, high_rotor_rpm, overtorque, overtemp, low_fuel,
low_battery, vortex_ring, retreating_blade_stall, overspeed, rotor_brake_on and hard_landing.

Changes, sent when a value moves by a threshold from the config: `moved`, `heading_changed`, `attitude_changed`,
`speed_changed`, `vertical_speed_changed`, `vertical_trend_changed{climb|descent|level}`, `altitude_changed`,
`rotor_rpm_changed`, `engine_rpm_changed`, `torque_changed`, `fuel_changed`.

## What is modelled

- Electrics: a battery, a generator, loads, and a voltage sag during a start. Without power the starter is rejected.
- Engine: off, cranking, starting, running, shutdown, failed. A turbine needs the fuel valve, the boost pump and the starter, and it can have a hung start or a hot start. A piston engine needs the magneto and a manually engaged clutch. Exhaust gas temperature is tracked, and overheating uses up the engine's health.
- Rotor: an energy balance (rotor inertia), an rpm governor with a freewheel unit, and a rotor brake. Because of the freewheel unit an autorotation happens by itself, with no special code.
- Thrust comes from collective, rpm and air density (ISA by altitude). Ground effect, and the induced velocity from momentum theory (climb and descent with Johnson's fit, forward flight with Glauert's). Profile power grows with speed. Inflow in a climb reduces thrust and in a descent increases it, which damps the vertical speed.
- Vortex ring state (a descent with power at low speed), retreating blade stall (speed and load factor), low and high rotor rpm, and Vne.
- Yaw: reactive torque follows the engine power, and the tail rotor is neutral at a hover. This means the pedals must be adjusted when the collective changes. There is also damping and weathercock stability.
- Fuel burn follows the power, and the mass changes accordingly.

## Position: the helicopter does not know the map

By default `position_mode="delta"`: there are no absolute coordinates in the API, neither in `get_state()` nor in events.
Instead the helicopter sends a **`moved`** event with the displacement since the previous report:
`dx`, `dy` (east and north), `dz`, `distance` (horizontal), `bearing` (the compass direction of motion, 0 is north),
`relative_bearing` (the direction of motion relative to the nose, from -180 to 180), `elevation` (the climb or descent angle, in degrees)
and `heading` (where the nose points). The game moves the player itself: `player.x += dx`, or "walk `distance`
at the angle `relative_bearing`".
The helicopter keeps its own height `z` (it needs it for air density and for height above the surface); `dz` in `moved`
is there so that you can keep your own height in sync.
`move_step` (meters) accumulates the displacement and sends it less often; nothing is lost.
`position_mode="absolute"` brings back `x, y` in `get_state()` and the `position_changed` event, for tests and debugging.

```python
for e in heli.update(now):
    if e.name == "moved":
        player.move(e.data["dx"], e.data["dy"]); player.z += e.data["dz"]
    elif e.name == "crashed":
        ...
```

### Map edge
`heli.reached_map_edge(nx=None, ny=None, mode=None)`. The helicopter does not know the map, and it does not need to
know that it moved to another map. With `mode="pass"` (the default, `map_edge_mode` in the config) physics is
unchanged and only a `map_edge` event is emitted, for sound or speech; the game changes the map and the position
itself (after the change, call `set_ground_level` for the new surface).
With `mode="block"` the helicopter cancels the speed towards the edge (the normal `(nx, ny)` points into the map;
without a normal all horizontal speed is cancelled); call it every frame while the helicopter pushes against the edge.
The `map_edge` event is sent at most once per second.

## Pause
`pause(reason)` and `resume(timestamp=None)`: while paused, `update()` computes nothing (commands are still accepted and
only queue events), simulation time stands still, and so do the button timers. After `resume()` the clock is re-anchored
on the next `update()` (or immediately if you pass `timestamp`), so a long map load does not turn into a jump in the physics.
This works with the system timer (`update()` without an argument) and with your own timestamps. `reset_clock()` re-anchors
the clock without pausing. `max_dt` (1 s) only protects against a single lag spike; it does not replace a pause.

## Units
Inside, everything is SI. `imperial_units=True` (in the config, or `Helicopter(..., imperial_units=True)`) switches
the API boundary: positions and heights in feet, speeds in ft/s. This applies to the inputs (`x, y, z, ground_z` in the
constructor, `reached_surface(ground_z)`, `set_ground_level`, `set_wind`) and to the outputs (`get_state()` and event data:
x y z agl dx dy dz distance vz ground_speed airspeed speed sink). `get_state()["units"]` says which units are in use.
Mass and fuel stay in kg, and angles in degrees. The public attributes `heli.x`, `heli.z` and `heli.vx` are internal and always in meters.

## Presets and configuration
`light_piston` (like a Robinson R22), `medium_turbine` (like a Bell 206), `medium_turbine_agile` (the same with a lively
climb), `heavy_twin` (like a Mi-8, the rotor turns clockwise). Any parameter can be overridden:
`HelicopterConfig("medium_turbine", mtow=1600, idle_nr=0.6, crash_sink=4.0)`. The keys are in `_DEFAULTS` at the
top of `helisim.py`. A new preset is a new entry in `PRESETS` that lists only the differences from `_DEFAULTS`.
Cyclic mode: `cyclic_mode="attitude"` (the stick is an angle, convenient on a keyboard) or `"rate"` (the stick is a
rotation rate, closer to reality).

## Tuning (the result of `measure.py`, which re-measures every preset)
Hover collective at 50 m with full tanks, power used in the hover, full-stick speed as a fraction of Vne, peak climb, hover ceiling:

- `light_piston`: 0.56, 56 percent, 1.05, 8 m/s, about 3500 m.
- `medium_turbine`: 0.60, 55 percent, 0.98, 10 m/s, about 2000 m.
- `medium_turbine_agile` (the previous tuning): 0.53, 46 percent, 0.95, 14 m/s, about 3500 m.
- `heavy_twin`: 0.52, 54 percent, 0.93, 12 m/s, about 3500 m.

An autorotation with the collective chosen to hold rpm gives a sink rate of 5 to 7.5 m/s and rotor rpm around 100 percent.

The knobs: `thrust_ratio` (thrust reserve), `drivetrain_eff` (drivetrain efficiency), `kappa` and `profile_power_kw` (power required),
`heave_damp` (damping of the vertical speed), `cda_h` (full-stick speed), `tail_*` and `yaw_damp` (the pedals),
`nr_overspeed` (the rotor overspeed threshold that destroys the rotor).

## Known simplifications, and what could come next
- Wind: the `wind` field already takes part in every speed computation, so you only have to set it.
- Not modelled: dynamic rollover on the ground, a tail rotor failure, damage to individual parts, air temperature, weight and balance limits.
- The numbers were chosen so that the behavior resembles the prototypes. They were not checked against flight manuals.

## Porting to NVGT
The code is deliberately flat: plain classes, `dict` maps to `dictionary`, floats, arrays, no dataclasses, generators or lambdas.
`Event.data` is a `dictionary<string, variant>`, and `update()` returns an array of events (you can skip porting the `subscribe` callbacks).
Replace `time.monotonic()` with `ticks() / 1000.0`. Replace the config constructor that takes `**overrides` with a dictionary of overrides.
