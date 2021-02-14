use serde::{Serialize, Deserialize};

use crate::conf::Scenario;
use crate::utils::math::Vec2;


/// changing properties
#[derive(Debug)]
#[derive(Clone, Copy)]
#[derive(Serialize, Deserialize)]
pub struct SpacecraftDynamic {

    // nav miscs

    pub t: f64,                         // unit: sec - time since beginning of simulation
    pub dt: f64,                        // unit: sec

    pub fuel_mass: f64,                 // unit: kg

    // nav acc

    pub acc_thrust: f64,
    pub acc_atm: f64,
    pub acc_gravity: f64,
    pub acc_centrifugal: f64,

    // nav trans

    pub pos: Vec2,                      // unit: m
    pub vel: Vec2,                      // unit: m/s
    pub acc: Vec2,                      // unit: m/s**2

    pub dv: f64,                        // unit: m/s - dv expended until now

    // nav ang

    pub ang_pos: f64,                   // unit: rad
    pub ang_vel: f64,                   // unit: rad/s
    pub ang_acc: f64,                   // unit: rad/s**2

    // gui

    pub gui: Vec2,                      // unit: m/s**2

    // ctr

    pub eng_throttle: f64,              // unit: 0-1
    pub eng_gimbal: f64,                // unit: 0-1
}


impl SpacecraftDynamic {
    pub fn new(conf: &Scenario) -> SpacecraftDynamic {
        SpacecraftDynamic {
            t: 0.0,
            dt: 1.0,

            eng_throttle: 0.0,
            fuel_mass: conf.initial_sc_fuel_mass,

            eng_gimbal: 0.0,

            acc_thrust: 0.0,
            acc_gravity: 0.0,
            acc_centrifugal: 0.0,
            acc_atm: 0.0,

            pos: conf.initial_sc_pos,
            vel: conf.initial_sc_vel,
            acc: Vec2 {x: 0.0, y: 0.0},

            dv: 0.0,

            ang_pos: conf.initial_sc_ang_pos,
            ang_vel: 0.0,
            ang_acc: 0.0,

            gui: Vec2 {x: 0.0, y: 0.0},
        }
    }
}
