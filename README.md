# helisim

A realistic helicopter flight model with no graphics and no sound. It is a Rust
library: the game owns the map, the sounds and the speech, and asks this crate
what the machine is doing.

```rust
use helisim::Helicopter;

let mut heli = Helicopter::new("medium_turbine");
heli.switch_battery();
heli.switch_fuel_valve();
heli.switch_fuel_pump();
heli.start().unwrap();
heli.switch_rotor_brake();

heli.axes.throttle_target = 1.0;
heli.axes.collective_target = 0.6;

loop {
    heli.step(0.02);
    let state = heli.state();
    println!("{}", state.summary());
    let (dx, dy) = heli.displacement(0.02);
    // the game moves the aircraft on its own map by dx, dy
}
```

## What it models

- **Levers with a hand's lag.** `collective`, `throttle`, `pitch`, `roll` and
  `pedals` each have a target and a real position, and the real one travels at a
  limited rate: 0.6 of full travel a second for the collective, 0.35 for the
  throttle, 2.5 for the cyclic and the pedals on the medium turbine.
- **An engine you have to start.** The starter needs the battery, the fuel valve
  and the pump; it winds the compressor to the point where the fuel lights it,
  and from there the throttle governs the speed. Closing the valve, running out
  of fuel or cutting the battery stops it.
- **A rotor with its own inertia.** Rotor speed follows the throttle through the
  transmission and decays on its own when the engine is out; the rotor brake
  stops it on the ground.
- **Thrust from momentum theory.** Collective buys a fraction of the maximum
  thrust, thrust goes with the square of rotor speed, ground effect adds up to
  twelve percent near the surface, induced velocity damps a climb and catches a
  descent, and air density falls with height.
- **A machine that turns by banking.** The stick sets an attitude, the attitude
  follows at a limited rate, a bank turns the nose, and the engine's torque pushes
  it the other way, which the pedals counter.
- **Gravity, always.** Cut the engine in the air and the machine sinks, reaching
  a terminal velocity set by its vertical drag.
- **Damage by contact.** A sink faster than the preset's limit, a tilt beyond 22
  degrees or too much speed over the ground is a crash; anything less is a
  landing.
- **Warnings** a cockpit can turn into sound: low and high rotor speed,
  overtorque, overtemperature, low fuel, low battery, rotor brake on, overspeed,
  retreating blade stall, vortex ring.

## Presets

| Preset | Rotor | Mass | Hover collective | Vne |
| --- | --- | --- | --- | --- |
| `light_piston` | 7.7 m | 622 kg | 0.56 | 50 m/s |
| `medium_turbine` | 10.2 m | 1450 kg | 0.60 | 62 m/s |
| `medium_turbine_agile` | 10.2 m | 1450 kg | 0.53 | 62 m/s |
| `heavy_twin` | 21.3 m | 12000 kg | 0.52 | 69 m/s |

The hover collectives are the ones the Python prototype measured, and the tests
hold the model to them.

## Where this came from

This repository held a Python prototype first: one file, momentum theory, events,
a keyboard example. It was the reference for how the machine should feel, and it
is still in the history (`git show 162386c:helisim.py`).

The port keeps the presets, their numbers, the lever rates, the start-up
sequence, the warning list and the crash rules. What it does not carry yet:

- autorotation entry and the flare: a dead engine gives a controlled descent, not
  a rotor kept alive by being talked down;
- torque as a shaft that can be damaged, rather than a number and a yaw;
- tail rotor failure, dynamic rollover, air temperature;
- the electrics as a bus with a starter load, rather than a percentage;
- the event stream: a game polls `state()` instead.

These are the next things to port, not things dropped by mistake.

## Tests

```bash
cargo test
```

Eleven tests: the starter refusing without power or fuel, the engine catching and
the rotor following, the hover point holding the machine at each preset's
collective, more collective climbing and less descending, an engine cut sinking,
a soft touchdown against a hard one, a bank turning the nose, and three on the
engine module itself.
