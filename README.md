# helisim

A realistic helicopter flight model with no graphics and no sound. It is a Python prototype that is meant to be
ported to NVGT. The helicopter knows nothing about your map: the game operates the controls, calls
`update(time)` every tick, receives events, and reports collisions back to the helicopter.

Requires Python 3.8 or newer. No external dependencies.

Russian copy of this file: [README.ru.md](README.ru.md).

## What is in the repository

- `helisim.py` - the module itself, a single file.
- `README.md` - this file. `DESIGN.md` - details of the model, the event list, tuning, notes for the NVGT port.
- `examples/` - five runnable examples. The output is plain text, one line per event, so it is friendly to screen readers.
- `demo_flight.py` - a complete flight by an automatic pilot: start-up, takeoff, hover, flight, turn, landing, shutdown.
- `test_scenarios.py`, `test_features.py` - scenario scripts for failures, collisions, pauses and units. They print
  what happens; they do not use assertions. `testutil.py` holds their shared helpers.
- `measure.py` - measures the characteristics of every preset.

## Quick start

```python
from helisim import Helicopter

heli = Helicopter("medium_turbine", timestamp=0.0)   # or "light_piston", "heavy_twin"
heli.switch("battery", True)
heli.switch("fuel_valve", True)
heli.switch("fuel_pump", True)
heli.button("starter")

t = 0.0
while True:
    t += 0.05
    for e in heli.update(t):           # the list of events produced during this tick
        print(e.name, e.data)
```

Clock: `update(timestamp)` takes seconds as a float. Without an argument it uses the system monotonic timer.

## Start-up checklist

Turbine helicopters (`medium_turbine`, `heavy_twin`):

1. `switch("battery", True)` - power. Without it the starter is rejected with the reason `no_electric_power`.
2. `switch("fuel_valve", True)` - the fuel valve.
3. `switch("fuel_pump", True)` - the boost pump, required to start.
4. `button("starter")` - press it and do not release it; the starter cuts out by itself at the right rpm. Keep the throttle at zero during the start.
5. Wait for the `engine_started` event.
6. `switch("generator", True)`.
7. `switch("rotor_brake", False)` - the rotor brake is on when the helicopter is parked.
8. `turn("throttle", 1.0)` and wait until the rotor rpm is about 98 percent.
9. Raise `collective` smoothly. The helicopter lifts off by itself and you get a `liftoff` event.

Piston helicopter (`light_piston`): switch on `ignition` instead of the pump, and switch on `clutch` after releasing the brake.

Shutdown: collective down, throttle to zero, let the engine cool, `button("engine_stop")`, rotor brake on.

## Controls

- Toggle switches, `switch(name, True/False)` or `toggle(name)`: battery, generator, fuel_pump, fuel_valve, ignition (piston only), avionics, nav_lights, landing_light, anti_collision, rotor_brake, clutch (piston only), sas.
- Push buttons, `press`, `release` or `button(name, hold=seconds)`: starter, engine_stop.
- Axes, `turn(name, value)` or `step(name, delta)`:
  collective from 0 to 1, throttle from 0 to 1 (twist grip), cyclic_roll from -1 to 1 (right is positive), cyclic_pitch from -1 to 1 (forward is positive), pedals from -1 to 1 (right is positive).
- `joystick(roll, pitch)` sets the cyclic in one call. `center()` centers the cyclic and the pedals.
- Levers move towards the requested value at a limited speed, like a pilot's hand. The real position is in `get_state()["controls"]`, the requested one in `["targets"]`.
- `controls_info()` returns all controls with readable labels, so you can build menus from it.
- An impossible command returns False and produces a `command_rejected` event with a reason.

## Events

`update()` returns a list of `Event` objects with the fields `t`, `name` and `data`. The main ones:

- Engine and systems: `engine_state`, `engine_started`, `starter_cutout`, `hot_start`, `start_failed`, `engine_flameout`, `engine_failure`, `fuel_exhausted`, `bus_power_changed`, `switch`, `button`.
- Flight: `liftoff`, `touchdown`, `bump`, `crashed`, `moved`, `heading_changed`, `attitude_changed`, `speed_changed`, `vertical_speed_changed`, `vertical_trend_changed`, `altitude_changed`, `rotor_rpm_changed`, `engine_rpm_changed`, `torque_changed`, `fuel_changed`.
- Warnings: `warning` with the fields `id` and `active`. The ids are low_rotor_rpm, high_rotor_rpm, overtorque, overtemp, low_fuel, low_battery, vortex_ring, retreating_blade_stall, overspeed, rotor_brake_on, hard_landing.
- Service: `paused`, `resumed`, `map_edge`, `command_rejected`.

Change events are sent only when a value moves by a threshold from the config, so they do not flood you. A snapshot of the whole state, for sound and speech, is available at any time from `get_state()`.

## What the game does

The game reports collisions. The helicopter decides the outcome from speeds and angles: a touchdown, a light bump, or `crashed=True` (the cause is in `crash_reason`).

- `reached_surface(ground_z)` - touched the ground. Gentle contact gives `touchdown`. A hard, fast or tilted contact gives `crashed`.
- `reached_wall(nx, ny, part)` - a wall. `part` is fuselage, rotor or tail. The normal points from the wall towards the helicopter.
- `reached_ceiling()`, `reached_water()`, `surface_lost()`, `set_ground_level(z)`.
- `fail_engine(reason)` - an engine failure from outside; an autorotation follows.
- `repair()` - restore the helicopter after a crash.

Position: by default the helicopter does not know map coordinates. It sends a `moved` event with the displacement since the previous report:
`dx`, `dy`, `dz`, `distance`, `bearing` (the direction of motion, 0 is north), `relative_bearing` (relative to the nose), `elevation` and `heading`.
The game moves the player itself. The helicopter does keep its own height above the surface, because it needs it to compute the air.

Map edge: `reached_map_edge()`. By default physics does not change and only a `map_edge` event is emitted; the game switches the map and position itself. The `block` mode cancels the speed towards the edge.

Pause: `pause("reason")` and `resume()` freeze the helicopter, and the paused time does not count. Use them when a map reloads or a game event takes over, especially when you use the system timer.

Units: meters inside. `imperial_units=True` switches the API input and output to feet and ft/s. Mass and fuel stay in kilograms. The attributes `heli.x`, `heli.z` and so on are internal and always in meters; the game should read `get_state()` and the events instead.

## Presets and configuration

- `light_piston` - a light piston helicopter, similar to a Robinson R22. Low rotor inertia, a manual clutch.
- `medium_turbine` - a medium turbine helicopter, similar to a Bell 206.
- `medium_turbine_agile` - the same, with a livelier climb.
- `heavy_twin` - a heavy twin-engine helicopter, similar to a Mi-8; the rotor turns clockwise.

Any parameter can be overridden: `HelicopterConfig("medium_turbine", mtow=1600, idle_nr=0.6)`.
A preset of your own is a new entry in the `PRESETS` dictionary that lists only the differences from the defaults (example 4).
Cyclic mode: `cyclic_mode="attitude"` (the stick sets an angle, convenient on a keyboard) or `"rate"` (the stick sets a rotation rate, closer to reality).
After changing parameters by hand, call `config.recalc()`.

## Examples

Run them from the `examples` folder:

- `python3 01_cold_start.py [preset]` - a start-up by the checklist, step by step.
- `python3 02_game_loop.py` - a skeleton of a game: feet, `moved` events, collisions, the map edge, a pause for a map load.
- `python3 03_failures.py [preset]` - an engine failure and running out of fuel, with an autorotation and a flare.
- `python3 04_custom_preset.py` - your own parameters and preset, and how altitude and load change the hover.
- `python3 05_keyboard.py` - a key layout and a key handler.

## Running the checks

- `python3 test_scenarios.py` - failures, collisions, rejected commands.
- `python3 test_features.py` - `moved`, pause, map edge, feet.
- `python3 demo_flight.py light_piston` - a whole flight; pick the preset you like.
- `python3 measure.py` - hover collective and power, speed at full stick, climb, pedals, hover ceiling.

## Porting to NVGT

The code is flat: plain classes, dictionaries, floats, with no dataclasses, generators or lambdas. `Event.data` is a dictionary from string to value, and `update()` returns an array of events. Replace `time.monotonic()` with `ticks() / 1000.0`. Replace the config constructor that takes overrides with a dictionary of overrides.

## Limitations

- The numbers were tuned to resemble the behavior of the real prototypes. They were not checked against flight manuals.
- There is no wind yet, but speeds are already computed relative to the air, and the `wind` field and the `set_wind` method are ready.
- Not modelled: dynamic rollover on the ground, a tail rotor failure, damage to individual parts, air temperature.
- In an autorotation there is no torque, but the tail rotor keeps pushing, so the heading must be held with the pedals. This is intentional.
