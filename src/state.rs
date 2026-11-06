//! One snapshot of everything a cockpit reads, in the game's units.

use crate::engine::EngineState;
use crate::{Helicopter, Warning};

#[derive(Clone, Debug)]
pub struct State {
    /// Height above the ground, metres.
    pub agl: f64,
    /// Vertical speed, metres a second, positive up.
    pub vz: f64,
    /// Ground speed, metres a second.
    pub speed: f64,
    /// Compass heading, degrees.
    pub heading: f64,
    /// Attitude, degrees: positive pitch is nose down, positive roll is right.
    pub pitch: f64,
    pub roll: f64,
    pub pitch_rate: f64,
    pub roll_rate: f64,
    pub yaw_rate: f64,
    /// Rotor and compressor speed as percentages.
    pub rotor_pct: f64,
    pub compressor_pct: f64,
    pub torque_pct: f64,
    pub egt: f64,
    pub fuel_kg: f64,
    pub fuel_flow_kgh: f64,
    pub battery_pct: f64,
    pub engine: EngineState,
    pub collective: f64,
    pub throttle: f64,
    pub on_ground: bool,
    pub crashed: bool,
    pub warnings: Vec<Warning>,
}

impl State {
    pub fn of(h: &Helicopter) -> State {
        State {
            agl: h.agl,
            vz: h.vz,
            speed: (h.vx * h.vx + h.vy * h.vy).sqrt(),
            heading: h.heading,
            pitch: h.pitch,
            roll: h.roll,
            pitch_rate: h.pitch_rate,
            roll_rate: h.roll_rate,
            yaw_rate: h.yaw_rate,
            rotor_pct: h.nr * 100.0,
            compressor_pct: h.engine.n1 * 100.0,
            torque_pct: h.engine.torque_pct,
            egt: h.engine.egt,
            fuel_kg: h.fuel_kg,
            fuel_flow_kgh: h.engine.fuel_flow_kgh,
            battery_pct: h.battery_pct,
            engine: h.engine.state,
            collective: h.axes.collective,
            throttle: h.axes.throttle,
            on_ground: h.on_ground,
            crashed: h.crashed,
            warnings: h.warnings.clone(),
        }
    }

    /// A one-line summary, the sort a screen reader can read out.
    pub fn summary(&self) -> String {
        format!(
            "height {:.0}, vertical speed {:+.1}, speed {:.0}, heading {:.0}, rotor {:.0} percent",
            self.agl, self.vz, self.speed, self.heading, self.rotor_pct
        )
    }
}
