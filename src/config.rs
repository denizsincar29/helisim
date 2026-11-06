//! Every number the model needs, and the four presets the prototype was tuned to.
//!
//! The names and the values come from the Python prototype this crate replaces:
//! they are what a game felt right with, not what a flight manual says.

/// Which machine. The names are the ones the old prototype used.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Preset {
    LightPiston,
    MediumTurbine,
    MediumTurbineAgile,
    HeavyTwin,
}

impl Preset {
    pub fn from_name(name: &str) -> Preset {
        match name {
            "light_piston" => Preset::LightPiston,
            "medium_turbine_agile" => Preset::MediumTurbineAgile,
            "heavy_twin" => Preset::HeavyTwin,
            _ => Preset::MediumTurbine,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Preset::LightPiston => "light_piston",
            Preset::MediumTurbine => "medium_turbine",
            Preset::MediumTurbineAgile => "medium_turbine_agile",
            Preset::HeavyTwin => "heavy_twin",
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub preset: Preset,

    // -- rotor and airframe
    pub rotor_diameter: f64,
    /// Nominal rotor speed, rpm. Only used to report real rpm to a cockpit.
    pub rotor_rpm: f64,
    /// +1 for a rotor that turns counter-clockwise seen from above.
    pub rotor_dir: f64,
    pub empty_mass: f64,
    /// Maximum take-off mass, kg. The weight the model balances against.
    pub mtow: f64,
    pub fuel_capacity: f64,
    /// Maximum thrust over weight at sea level and nominal rpm.
    pub thrust_ratio: f64,
    /// Thrust at zero collective, as a fraction of the maximum.
    pub min_collective: f64,
    pub ground_effect: bool,
    /// Flat-plate drag, square metres, forward and vertical.
    pub cda_h: f64,
    pub cda_v: f64,
    pub vne: f64,

    // -- engine
    pub max_power_kw: f64,
    pub idle_n1: f64,
    pub idle_nr: f64,
    pub starter_rate: f64,
    /// Compressor speed at which the engine runs without the starter.
    pub self_sustain_n1: f64,
    pub n1_tau: f64,
    pub n1_decay: f64,
    pub sfc: f64,
    pub idle_fuel_kgh: f64,
    pub egt_limit: f64,
    /// How fast the rotor follows the throttle, seconds.
    pub rotor_tau: f64,

    // -- controls and attitude
    pub collective_rate: f64,
    pub throttle_rate: f64,
    pub cyclic_rate: f64,
    pub pedal_rate: f64,
    pub max_cmd_pitch: f64,
    pub max_cmd_roll: f64,
    pub max_pitch_rate: f64,
    pub max_roll_rate: f64,
    pub att_tau: f64,
    /// Degrees a second of turn for each radian of bank, at full sine.
    pub turn_from_bank: f64,
    pub max_yaw_rate: f64,
    /// Yaw from engine torque at full torque, degrees a second.
    pub torque_yaw: f64,

    // -- limits and damage
    pub nr_low: f64,
    pub nr_overspeed: f64,
    pub crash_sink: f64,
    pub crash_tilt: f64,
    pub crash_ground_speed: f64,
}

impl Config {
    pub fn preset(name: &str) -> Config {
        Config::of(Preset::from_name(name))
    }

    pub fn of(preset: Preset) -> Config {
        match preset {
            Preset::MediumTurbine => Config {
                preset,
                rotor_diameter: 10.2,
                rotor_rpm: 394.0,
                rotor_dir: 1.0,
                empty_mass: 840.0,
                mtow: 1450.0,
                fuel_capacity: 280.0,
                thrust_ratio: 1.4,
                min_collective: 0.04,
                ground_effect: true,
                cda_h: 1.25,
                cda_v: 1.6,
                vne: 62.0,
                max_power_kw: 313.0,
                idle_n1: 0.62,
                idle_nr: 0.55,
                starter_rate: 0.035,
                self_sustain_n1: 0.35,
                n1_tau: 2.5,
                n1_decay: 0.04,
                sfc: 0.33,
                idle_fuel_kgh: 35.0,
                egt_limit: 810.0,
                rotor_tau: 3.0,
                collective_rate: 0.6,
                throttle_rate: 0.35,
                cyclic_rate: 2.5,
                pedal_rate: 2.5,
                max_cmd_pitch: 25.0,
                max_cmd_roll: 30.0,
                max_pitch_rate: 45.0,
                max_roll_rate: 60.0,
                att_tau: 0.30,
                turn_from_bank: 55.0,
                max_yaw_rate: 50.0,
                torque_yaw: 18.0,
                nr_low: 0.94,
                nr_overspeed: 1.22,
                crash_sink: 5.5,
                crash_tilt: 22.0,
                crash_ground_speed: 10.0,
            },
            Preset::MediumTurbineAgile => Config {
                thrust_ratio: 1.55,
                att_tau: 0.24,
                turn_from_bank: 62.0,
                ..Config::of(Preset::MediumTurbine)
            },
            Preset::LightPiston => Config {
                preset,
                rotor_diameter: 7.7,
                rotor_rpm: 530.0,
                rotor_dir: 1.0,
                empty_mass: 400.0,
                mtow: 622.0,
                fuel_capacity: 60.0,
                thrust_ratio: 1.55,
                min_collective: 0.04,
                ground_effect: true,
                cda_h: 0.8,
                cda_v: 1.0,
                vne: 50.0,
                max_power_kw: 98.0,
                idle_n1: 0.40,
                idle_nr: 0.45,
                starter_rate: 0.08,
                self_sustain_n1: 0.0,
                n1_tau: 1.2,
                n1_decay: 0.1,
                sfc: 0.30,
                idle_fuel_kgh: 6.0,
                egt_limit: 700.0,
                rotor_tau: 2.0,
                collective_rate: 0.8,
                throttle_rate: 0.5,
                cyclic_rate: 2.5,
                pedal_rate: 2.5,
                max_cmd_pitch: 25.0,
                max_cmd_roll: 30.0,
                max_pitch_rate: 70.0,
                max_roll_rate: 90.0,
                att_tau: 0.18,
                turn_from_bank: 60.0,
                max_yaw_rate: 45.0,
                torque_yaw: 14.0,
                nr_low: 0.94,
                nr_overspeed: 1.22,
                crash_sink: 4.5,
                crash_tilt: 22.0,
                crash_ground_speed: 10.0,
            },
            Preset::HeavyTwin => Config {
                preset,
                rotor_diameter: 21.3,
                rotor_rpm: 192.0,
                rotor_dir: -1.0,
                empty_mass: 7200.0,
                mtow: 12000.0,
                fuel_capacity: 1870.0,
                thrust_ratio: 1.5,
                min_collective: 0.04,
                ground_effect: true,
                cda_h: 3.2,
                cda_v: 8.0,
                vne: 69.0,
                max_power_kw: 2600.0,
                idle_n1: 0.65,
                idle_nr: 0.55,
                starter_rate: 0.02,
                self_sustain_n1: 0.35,
                n1_tau: 4.0,
                n1_decay: 0.04,
                sfc: 0.28,
                idle_fuel_kgh: 220.0,
                egt_limit: 880.0,
                rotor_tau: 5.0,
                collective_rate: 0.4,
                throttle_rate: 0.25,
                cyclic_rate: 1.5,
                pedal_rate: 1.8,
                max_cmd_pitch: 18.0,
                max_cmd_roll: 25.0,
                max_pitch_rate: 20.0,
                max_roll_rate: 25.0,
                att_tau: 0.6,
                turn_from_bank: 40.0,
                max_yaw_rate: 45.0,
                torque_yaw: 20.0,
                nr_low: 0.94,
                nr_overspeed: 1.22,
                crash_sink: 5.0,
                crash_tilt: 22.0,
                crash_ground_speed: 10.0,
            },
        }
    }
}
