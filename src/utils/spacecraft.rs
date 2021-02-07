use crate::conf::Scenario;
use crate::utils::math::Vec2;


/// changing properties
#[derive(Debug)]
#[derive(Clone, Copy)]
pub struct SpacecraftDynamic {
    pub t: f64,                         // unit: sec - time since beginning of simulation
    pub dt: f64,                        // unit: sec

    pub eng_throttle: f64,              // unit: 0-1
    pub eng_gimbal: f64,                // unit: rad

    pub fuel_mass: f64,                 // unit: kg

    pub pos: Vec2,                      // unit: m
    pub vel: Vec2,                      // unit: m/s
    pub acc: Vec2,                      // unit: m/s**2

    pub ang_pos: f64,                   // unit: rad
    pub ang_vel: f64,                   // unit: rad/s
    pub ang_acc: f64,                   // unit: rad/s**2
}


impl SpacecraftDynamic {
    pub fn new(conf: &Scenario) -> SpacecraftDynamic {
        SpacecraftDynamic {
            t: 0.0,
            dt: 1.0,

            eng_gimbal: 0.0,
            eng_throttle: 0.0,

            fuel_mass: conf.initial_sc_fuel_mass,

            pos: conf.initial_sc_pos,
            vel: conf.initial_sc_vel,
            acc: Vec2 {x: 0.0, y: 0.0},

            ang_pos: conf.initial_sc_ang_pos,
            ang_vel: 0.0,
            ang_acc: 0.0,
        }
    }
}
