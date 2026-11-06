//! A helicopter flight model with no graphics and no sound.
//!
//! The game owns the map, the sounds and the speech. This crate owns the machine:
//! the levers with their actuator lag, the engine and its start-up, the rotor and
//! the thrust it makes, and the numbers a cockpit reads. Ask it for a state
//! snapshot, drive it with one `step` per frame.
//!
//! ```no_run
//! use helisim::{Axes, Helicopter};
//!
//! let mut heli = Helicopter::new("medium_turbine");
//! heli.switch_battery();
//! heli.switch_fuel_valve();
//! heli.switch_fuel_pump();
//! heli.start();
//! heli.step(0.02);
//! ```
//!
//! This is a port of the Python prototype that used to live in this repository.
//! The numbers are the ones that prototype was tuned to; the physics is a
//! momentum-theory model at the level a game needs, not a flight manual.

mod config;
mod engine;
mod state;

pub use config::{Config, Preset};
pub use engine::{Engine, EngineState};
pub use state::State;

/// One step of the model, in seconds. Larger steps are cut into pieces so a
/// frame hitch does not teleport the machine.
pub const MAX_SUBSTEP: f64 = 0.02;

/// The five things a pilot moves, with the lag of a hand.
#[derive(Clone, Copy, Debug, Default)]
pub struct Axes {
    pub collective: f64,
    pub collective_target: f64,
    pub throttle: f64,
    pub throttle_target: f64,
    pub pitch: f64,
    pub pitch_target: f64,
    pub roll: f64,
    pub roll_target: f64,
    pub pedals: f64,
    pub pedals_target: f64,
}

impl Axes {
    fn approach(current: f64, target: f64, rate: f64, dt: f64) -> f64 {
        let step = rate * dt;
        if (target - current).abs() <= step {
            target
        } else if target > current {
            current + step
        } else {
            current - step
        }
    }

    pub fn step(&mut self, cfg: &Config, dt: f64) {
        self.collective = Axes::approach(self.collective, self.collective_target, cfg.collective_rate, dt);
        self.throttle = Axes::approach(self.throttle, self.throttle_target, cfg.throttle_rate, dt);
        self.pitch = Axes::approach(self.pitch, self.pitch_target, cfg.cyclic_rate, dt);
        self.roll = Axes::approach(self.roll, self.roll_target, cfg.cyclic_rate, dt);
        self.pedals = Axes::approach(self.pedals, self.pedals_target, cfg.pedal_rate, dt);
    }
}

/// What is wrong right now. A cockpit turns each of these into a sound.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum Warning {
    LowRotorRpm,
    HighRotorRpm,
    OverTorque,
    OverTemperature,
    LowFuel,
    LowBattery,
    VortexRing,
    RetreatingBladeStall,
    Overspeed,
    RotorBrakeOn,
    HardLanding,
}

/// The machine.
pub struct Helicopter {
    pub cfg: Config,
    pub axes: Axes,
    pub engine: Engine,
    /// Position in world units. The game is free to ignore it and keep its own.
    pub x: f64,
    pub y: f64,
    /// Height above the ground.
    pub agl: f64,
    pub ground: f64,
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    /// Nose direction, degrees, 0 is north.
    pub heading: f64,
    /// Attitude, degrees. Positive pitch is nose down, positive roll is right.
    pub pitch: f64,
    pub roll: f64,
    pub pitch_rate: f64,
    pub roll_rate: f64,
    pub yaw_rate: f64,
    /// Rotor speed as a fraction of nominal.
    pub nr: f64,
    pub fuel_kg: f64,
    pub battery_pct: f64,
    pub on_ground: bool,
    pub crashed: bool,
    pub switches: Switches,
    pub warnings: Vec<Warning>,
    time: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Switches {
    pub battery: bool,
    pub generator: bool,
    pub fuel_valve: bool,
    pub fuel_pump: bool,
    pub rotor_brake: bool,
    pub landing_light: bool,
}

impl Helicopter {
    pub fn new(preset: &str) -> Helicopter {
        Helicopter::with_config(Config::preset(preset))
    }

    pub fn with_config(cfg: Config) -> Helicopter {
        let fuel = cfg.fuel_capacity;
        Helicopter {
            cfg,
            axes: Axes::default(),
            engine: Engine::new(),
            x: 0.0,
            y: 0.0,
            agl: 0.0,
            ground: 0.0,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            heading: 0.0,
            pitch: 0.0,
            roll: 0.0,
            pitch_rate: 0.0,
            roll_rate: 0.0,
            yaw_rate: 0.0,
            nr: 0.0,
            fuel_kg: fuel,
            battery_pct: 100.0,
            on_ground: true,
            crashed: false,
            switches: Switches { rotor_brake: true, ..Switches::default() },
            warnings: Vec::new(),
            time: 0.0,
        }
    }

    // -- switches ------------------------------------------------------------

    pub fn switch_battery(&mut self) -> bool {
        self.switches.battery = !self.switches.battery;
        self.switches.battery
    }

    pub fn switch_fuel_valve(&mut self) -> bool {
        self.switches.fuel_valve = !self.switches.fuel_valve;
        self.switches.fuel_valve
    }

    pub fn switch_fuel_pump(&mut self) -> bool {
        self.switches.fuel_pump = !self.switches.fuel_pump;
        self.switches.fuel_pump
    }

    pub fn switch_generator(&mut self) -> bool {
        self.switches.generator = !self.switches.generator;
        self.switches.generator
    }

    pub fn switch_rotor_brake(&mut self) -> bool {
        self.switches.rotor_brake = !self.switches.rotor_brake;
        self.switches.rotor_brake
    }

    /// Why the starter will not turn, if it will not.
    pub fn can_start(&self) -> Result<(), &'static str> {
        if !self.switches.battery {
            return Err("no power");
        }
        if !self.switches.fuel_valve || !self.switches.fuel_pump {
            return Err("no fuel");
        }
        Ok(())
    }

    pub fn start(&mut self) -> Result<(), &'static str> {
        self.can_start()?;
        self.engine.start();
        Ok(())
    }

    /// The stop button. A running engine winds down over the next seconds.
    pub fn stop(&mut self) {
        self.engine.stop();
    }

    // -- flying --------------------------------------------------------------

    /// One frame. `dt` in seconds; it is cut into `MAX_SUBSTEP` pieces so a
    /// long frame cannot skip a crash.
    pub fn step(&mut self, dt: f64) {
        if self.crashed {
            return;
        }
        let dt = dt.clamp(0.0, 1.0);
        let mut left = dt;
        while left > 0.0 {
            let h = left.min(MAX_SUBSTEP);
            self.substep(h);
            left -= h;
        }
        self.time += dt;
    }

    fn substep(&mut self, dt: f64) {
        self.axes.step(&self.cfg, dt);
        self.engine.step(&self.cfg, &self.switches, self.axes.throttle.max(self.idle_throttle()), self.fuel_kg, dt);
        self.fuel_kg = (self.fuel_kg - self.engine.fuel_flow_kgh * dt / 3600.0).max(0.0);

        // The rotor follows the engine through the transmission. With the engine
        // out it keeps turning on its own inertia and drags down; the brake stops
        // it quickly when it is on the ground.
        let rotor_target = if self.engine.state == EngineState::Running || self.engine.state == EngineState::Cranking {
            self.axes.throttle.max(self.cfg.idle_nr.min(0.4))
        } else {
            0.0
        };
        let tau = if self.switches.rotor_brake && self.on_ground { 1.2 } else { self.cfg.rotor_tau };
        self.nr += (rotor_target - self.nr) * (1.0 - (-dt / tau).exp());
        self.nr = self.nr.clamp(0.0, 1.25);

        self.attitude(dt);
        self.forces(dt);
        self.contacts();
        self.update_warnings();
    }

    fn idle_throttle(&self) -> f64 {
        if self.engine.state == EngineState::Off {
            0.0
        } else {
            self.cfg.idle_n1
        }
    }

    /// The stick sets an attitude and the machine follows it, at a limited rate.
    fn attitude(&mut self, dt: f64) {
        let want_pitch = self.axes.pitch * self.cfg.max_cmd_pitch;
        let want_roll = self.axes.roll * self.cfg.max_cmd_roll;
        let k = 1.0 - (-dt / self.cfg.att_tau).exp();
        let dp = (want_pitch - self.pitch) * k;
        let dr = (want_roll - self.roll) * k;
        let dp = dp.clamp(-self.cfg.max_pitch_rate * dt, self.cfg.max_pitch_rate * dt);
        let dr = dr.clamp(-self.cfg.max_roll_rate * dt, self.cfg.max_roll_rate * dt);
        self.pitch += dp;
        self.roll += dr;
        self.pitch_rate = dp / dt;
        self.roll_rate = dr / dt;

        // A helicopter turns by banking, and the pedals add their own yaw.
        let bank = self.roll.to_radians();
        let turn = self.cfg.turn_from_bank * bank.sin();
        let pedal_yaw = self.axes.pedals * self.cfg.max_yaw_rate;
        // The engine's torque pushes the nose the other way from the rotor.
        let torque_yaw = self.cfg.torque_yaw * (self.engine.torque_pct / 100.0) * self.cfg.rotor_dir as f64;
        let wanted = turn + pedal_yaw + torque_yaw;
        let k_yaw = 1.0 - (-dt / 0.35).exp();
        self.yaw_rate += (wanted - self.yaw_rate) * k_yaw;
        self.heading = (self.heading + self.yaw_rate * dt).rem_euclid(360.0);
    }

    /// Thrust, drag and gravity.
    fn forces(&mut self, dt: f64) {
        let cfg = &self.cfg;
        let rho = 1.225 * (1.0 - 2.2557e-5 * (self.agl + self.ground)).powf(4.2559);
        let radius = cfg.rotor_diameter / 2.0;
        let area = std::f64::consts::PI * radius * radius;
        let weight = cfg.mtow * 9.81;

        // Collective buys a fraction of the maximum thrust, and the rotor makes
        // that fraction only at nominal speed: thrust goes with the square of rpm.
        let lever = cfg.min_collective + (1.0 - cfg.min_collective) * self.axes.collective;
        let max_thrust = cfg.thrust_ratio * weight;
        let ground_effect = if cfg.ground_effect && self.agl < 2.0 * radius {
            1.0 + 0.12 * (1.0 - (self.agl / (2.0 * radius)).clamp(0.0, 1.0))
        } else {
            1.0
        };
        let thrust = lever * max_thrust * (self.nr * self.nr) * (rho / 1.225) * ground_effect;

        // Momentum theory: the induced velocity is what is left of the hover value
        // once the machine is already moving that way, so a climb damps itself and
        // a descent is caught by the air.
        let vh = (thrust / (2.0 * rho * area)).sqrt().max(0.01);
        let climb = self.vz;
        let vi = vh * vh / (climb + (climb * climb + vh * vh).sqrt());
        let effective = thrust * (vi / vh).clamp(0.35, 1.6);

        let tilt = (self.pitch.to_radians().cos()) * (self.roll.to_radians().cos());
        let lift = effective * tilt;
        let drag_v = 0.5 * rho * cfg.cda_v * self.vz * self.vz.abs();
        let accel_v = (lift - weight - drag_v) / cfg.mtow;
        self.vz += accel_v * dt;

        // Forward: the tilted thrust is the only thing that pushes, and the
        // fuselage drags against it.
        let forward = effective * self.pitch.to_radians().sin();
        let speed = (self.vx * self.vx + self.vy * self.vy).sqrt();
        let drag_h = 0.5 * rho * cfg.cda_h * speed * speed;
        let dir = if speed > 0.01 { (self.vx / speed, self.vy / speed) } else { (0.0, 0.0) };
        let accel_h = (forward - drag_h) / cfg.mtow;
        self.vx += accel_h * dir.0 * dt;
        self.vy += accel_h * dir.1 * dt;

        // The bank also slides the machine sideways, which is what a turn feels
        // like from inside.
        let side = effective * self.roll.to_radians().sin() / cfg.mtow;
        let right = (self.heading.to_radians().cos(), -self.heading.to_radians().sin());
        self.vx += side * right.0 * dt;
        self.vy += side * right.1 * dt;

        self.agl += self.vz * dt;
        let north = self.heading.to_radians().cos();
        let east = self.heading.to_radians().sin();
        let fwd = speed * dt;
        if fwd > 0.0 {
            self.x += east * fwd * 0.0; // the game owns motion; see `displacement`
            self.y += north * fwd * 0.0;
        }
        let _ = (east, north);
    }

    /// How far the machine moved this frame, in world units. The game applies it,
    /// because the game owns the map.
    pub fn displacement(&self, dt: f64) -> (f64, f64) {
        (self.vx * dt, self.vy * dt)
    }

    fn contacts(&mut self) {
        if self.agl > 0.0 {
            self.on_ground = false;
            return;
        }
        self.agl = 0.0;
        let sink = -self.vz;
        let tilt = self.pitch.abs().max(self.roll.abs());
        let speed = (self.vx * self.vx + self.vy * self.vy).sqrt();
        if !self.on_ground {
            if sink > self.cfg.crash_sink || tilt > self.cfg.crash_tilt || speed > self.cfg.crash_ground_speed {
                self.crashed = true;
            }
            self.on_ground = true;
        }
        self.vz = self.vz.max(0.0) * 0.2;
        self.vx *= 0.2;
        self.vy *= 0.2;
    }

    fn update_warnings(&mut self) {
        let mut w = Vec::new();
        if self.nr < self.cfg.nr_low && self.engine.state != EngineState::Off {
            w.push(Warning::LowRotorRpm);
        }
        if self.nr > self.cfg.nr_overspeed {
            w.push(Warning::HighRotorRpm);
        }
        if self.engine.torque_pct > 100.0 {
            w.push(Warning::OverTorque);
        }
        if self.engine.egt > self.cfg.egt_limit {
            w.push(Warning::OverTemperature);
        }
        if self.fuel_kg < self.cfg.fuel_capacity * 0.15 {
            w.push(Warning::LowFuel);
        }
        if self.battery_pct < 20.0 {
            w.push(Warning::LowBattery);
        }
        if self.switches.rotor_brake && self.nr > 0.05 {
            w.push(Warning::RotorBrakeOn);
        }
        let speed = (self.vx * self.vx + self.vy * self.vy).sqrt();
        if speed > self.cfg.vne {
            w.push(Warning::Overspeed);
        }
        if self.roll.abs() > 30.0 || self.pitch.abs() > self.cfg.max_cmd_pitch + 8.0 {
            w.push(Warning::RetreatingBladeStall);
        }
        if self.vz < -6.0 && self.nr < 0.9 && self.nr > 0.3 {
            w.push(Warning::VortexRing);
        }
        self.warnings = w;
    }

    pub fn state(&self) -> State {
        State::of(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn running(preset: &str) -> Helicopter {
        let mut h = Helicopter::new(preset);
        h.switch_battery();
        h.switch_fuel_valve();
        h.switch_fuel_pump();
        assert!(h.start().is_ok());
        h.switch_rotor_brake();
        h.axes.throttle_target = 1.0;
        h.axes.throttle = 1.0;
        for _ in 0..600 {
            h.step(0.02);
        }
        h
    }

    #[test]
    fn the_starter_refuses_without_power_or_fuel() {
        let mut h = Helicopter::new("medium_turbine");
        assert_eq!(h.can_start(), Err("no power"));
        h.switch_battery();
        assert_eq!(h.can_start(), Err("no fuel"));
        h.switch_fuel_valve();
        h.switch_fuel_pump();
        assert!(h.can_start().is_ok());
    }

    #[test]
    fn the_engine_runs_up_and_the_rotor_follows_it() {
        let h = running("medium_turbine");
        assert_eq!(h.engine.state, EngineState::Running);
        assert!(h.nr > 0.9, "rotor {}", h.nr);
        assert!(h.engine.n1 > 0.9);
    }

    #[test]
    fn with_the_rotor_up_the_hover_point_holds_the_machine() {
        let mut h = running("medium_turbine");
        h.agl = 50.0;
        h.on_ground = false;
        h.axes.collective = 0.6;
        h.axes.collective_target = 0.6;
        for _ in 0..1200 {
            h.step(0.02);
        }
        assert!(h.vz.abs() < 2.0, "vertical speed should settle near zero, got {}", h.vz);
        assert!(h.agl > 20.0, "must not sink to the ground, height {}", h.agl);
    }

    #[test]
    fn more_collective_climbs_and_less_descends() {
        let mut up = running("medium_turbine");
        up.agl = 200.0;
        up.on_ground = false;
        up.axes.collective = 0.85;
        up.axes.collective_target = 0.85;
        for _ in 0..300 {
            up.step(0.02);
        }
        assert!(up.vz > 1.0, "climb {}", up.vz);

        let mut down = running("medium_turbine");
        down.agl = 200.0;
        down.on_ground = false;
        down.axes.collective = 0.45;
        down.axes.collective_target = 0.45;
        for _ in 0..300 {
            down.step(0.02);
        }
        assert!(down.vz < -1.0, "descent {}", down.vz);
    }

    #[test]
    fn an_engine_cut_in_the_air_sinks() {
        let mut h = running("medium_turbine");
        h.agl = 400.0;
        h.on_ground = false;
        h.axes.collective = 0.6;
        h.axes.collective_target = 0.6;
        h.stop();
        // Watch the first seconds, before the machine reaches the ground and the
        // contact code has any say: the question is whether gravity wins.
        for _ in 0..200 {
            h.step(0.02);
        }
        assert!(h.vz < -3.0, "vertical speed {}", h.vz);
        assert!(h.agl < 400.0, "height {}", h.agl);
        assert!(h.nr < 0.95, "the rotor must decay, {}", h.nr);
    }

    #[test]
    fn a_hard_touchdown_is_a_crash_and_a_soft_one_is_not() {
        let mut soft = running("medium_turbine");
        soft.agl = 0.4;
        soft.on_ground = false;
        soft.vz = -0.5;
        for _ in 0..60 {
            soft.step(0.02);
        }
        assert!(soft.on_ground, "it has to be on the ground");
        assert!(!soft.crashed, "half a metre a second is a landing");

        let mut hard = running("medium_turbine");
        hard.agl = 0.4;
        hard.on_ground = false;
        hard.vz = -9.0;
        for _ in 0..60 {
            hard.step(0.02);
        }
        assert!(hard.crashed, "nine metres a second is a crash");
    }

    #[test]
    fn a_bank_turns_the_machine() {
        let mut h = running("medium_turbine");
        h.agl = 200.0;
        h.on_ground = false;
        h.axes.roll = 0.6;
        h.axes.roll_target = 0.6;
        for _ in 0..200 {
            h.step(0.02);
        }
        assert!(h.heading > 5.0, "heading {}", h.heading);
    }

    #[test]
    fn every_preset_lifts_at_its_own_collective() {
        for (preset, hover) in [
            ("light_piston", 0.56),
            ("medium_turbine", 0.60),
            ("medium_turbine_agile", 0.53),
            ("heavy_twin", 0.52),
        ] {
            let mut h = running(preset);
            h.agl = 50.0;
            h.on_ground = false;
            h.axes.collective = hover;
            h.axes.collective_target = hover;
            for _ in 0..600 {
                h.step(0.02);
            }
            assert!(
                h.vz.abs() < 3.0,
                "{preset} should hover near collective {hover}, vertical speed {}",
                h.vz
            );
        }
    }
}
