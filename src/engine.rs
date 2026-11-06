//! The engine behind the rotor: a compressor that has to be wound up, fuel that
//! makes it self-sustaining, and a temperature that follows.

use crate::config::Config;
use crate::Switches;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum EngineState {
    Off,
    Cranking,
    Running,
    Stopping,
}

pub struct Engine {
    pub state: EngineState,
    /// Compressor speed, 0..1 of nominal.
    pub n1: f64,
    /// Exhaust gas temperature, degrees Celsius.
    pub egt: f64,
    /// Shaft torque as a percentage of the maximum continuous.
    pub torque_pct: f64,
    /// Fuel burn, kg per hour.
    pub fuel_flow_kgh: f64,
    /// 0..1, drops when the engine is abused.
    pub health: f64,
}

impl Engine {
    pub fn new() -> Engine {
        Engine {
            state: EngineState::Off,
            n1: 0.0,
            egt: 20.0,
            torque_pct: 0.0,
            fuel_flow_kgh: 0.0,
            health: 1.0,
        }
    }

    pub fn start(&mut self) {
        if self.state != EngineState::Running {
            self.state = EngineState::Cranking;
        }
    }

    pub fn stop(&mut self) {
        if self.state == EngineState::Running || self.state == EngineState::Cranking {
            self.state = EngineState::Stopping;
        }
    }

    /// One step. `throttle` is where the twist grip is, and `fuel_kg` what is
    /// left in the tanks: an engine with no fuel stops, whoever left the valves
    /// open.
    pub fn step(&mut self, cfg: &Config, sw: &Switches, throttle: f64, fuel_kg: f64, dt: f64) {
        if fuel_kg <= 0.0 && self.state != EngineState::Off {
            self.state = EngineState::Stopping;
        }

        match self.state {
            EngineState::Cranking => {
                if !sw.battery || !sw.fuel_valve || !sw.fuel_pump {
                    self.state = EngineState::Stopping;
                } else {
                    // The starter winds the compressor up to the point where the
                    // fuel lights it. Below that point it would just whine.
                    self.n1 = (self.n1 + cfg.starter_rate * dt * 10.0).min(1.0);
                    if self.n1 >= cfg.self_sustain_n1.max(0.3) {
                        self.state = EngineState::Running;
                    }
                }
            }
            EngineState::Running => {
                if !sw.fuel_valve || !sw.fuel_pump {
                    self.state = EngineState::Stopping;
                }
                // The governor holds the compressor at the grip: the pilot asks
                // for a speed, not for a fuel flow.
                let want = throttle.clamp(cfg.idle_n1, 1.0);
                let k = 1.0 - (-dt / cfg.n1_tau).exp();
                self.n1 += (want - self.n1) * k;
            }
            EngineState::Stopping | EngineState::Off => {
                self.n1 = (self.n1 - cfg.n1_decay * dt * 5.0).max(0.0);
                if self.n1 <= 0.001 {
                    self.state = EngineState::Off;
                    self.n1 = 0.0;
                }
            }
        }

        // Torque and fuel follow the compressor: a turbine at idle makes a
        // fraction of its rated power.
        let running = self.state == EngineState::Running;
        let load = if running { (self.n1 / cfg.idle_n1.max(0.2)).clamp(0.0, 1.4) } else { 0.0 };
        self.torque_pct = (load * 85.0).min(140.0);
        self.fuel_flow_kgh = if running {
            cfg.idle_fuel_kgh + (cfg.max_power_kw * cfg.sfc - cfg.idle_fuel_kgh) * load.min(1.0)
        } else {
            0.0
        };

        // EGT rises with compressor speed and lags behind it.
        let want_egt = if self.n1 > 0.02 {
            20.0 + 480.0 * self.n1 + 380.0 * self.n1 * self.n1
        } else {
            20.0
        };
        self.egt += (want_egt - self.egt) * (1.0 - (-dt / 2.0).exp());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Config;

    fn electrical() -> Switches {
        Switches { battery: true, fuel_valve: true, fuel_pump: true, ..Switches::default() }
    }

    #[test]
    fn the_engine_catches_and_runs() {
        let cfg = Config::preset("medium_turbine");
        let mut e = Engine::new();
        e.start();
        for _ in 0..400 {
            e.step(&cfg, &electrical(), 1.0, 100.0, 0.02);
        }
        assert_eq!(e.state, EngineState::Running);
        assert!(e.n1 > 0.9, "compressor {}", e.n1);
        assert!(e.egt > 400.0, "temperature {}", e.egt);
    }

    #[test]
    fn closing_the_valve_stops_it() {
        let cfg = Config::preset("medium_turbine");
        let mut e = Engine::new();
        e.start();
        for _ in 0..400 {
            e.step(&cfg, &electrical(), 1.0, 100.0, 0.02);
        }
        let mut sw = electrical();
        sw.fuel_valve = false;
        for _ in 0..600 {
            e.step(&cfg, &sw, 1.0, 100.0, 0.02);
        }
        assert_eq!(e.state, EngineState::Off);
        assert_eq!(e.torque_pct, 0.0);
    }

    #[test]
    fn no_fuel_means_no_engine() {
        let cfg = Config::preset("medium_turbine");
        let mut e = Engine::new();
        e.start();
        for _ in 0..200 {
            e.step(&cfg, &electrical(), 1.0, 0.0, 0.02);
        }
        assert_ne!(e.state, EngineState::Running);
    }
}
