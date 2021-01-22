use std::f64::consts::PI;

use crate::utils::math::Vec2;


/// fixed properties
#[derive(Debug)]
#[derive(Clone, Copy)]
pub struct SpacecraftStatic {
    pub nominal_thrust: f64,            // unit: N
    pub nominal_mass_flow: f64,         // unit: kg/s

    pub dry_mass: f64,                  // unit: kg
}

/// changing properties
#[derive(Debug)]
#[derive(Clone, Copy)]
pub struct SpacecraftDynamic {
    pub t: f64,                         // unit: sec - time since beginning of simulation

    pub eng_throttle: f64,              // unit: 0-1
    pub eng_gimbal: f64,                // unit: rad

    pub fuel_mass: f64,                 // unit: kg

    pub pos: Vec2,                      // unit: m
    pub vel: Vec2,                      // unit: m/s
    pub acc: Vec2,                      // unit: m/s**2

    pub ang_pos: f64,                   // unit: rad
    pub ang_vel: f64,                   // unit: rad/s
}


impl SpacecraftStatic {
    pub fn new() -> SpacecraftStatic {
        SpacecraftStatic {
            nominal_thrust: 45_000.0,  // TODO from conf
            nominal_mass_flow: 14.75,  // TODO from conf - thrust/(g0*isp)
            dry_mass: 6_800.0,  // TODO from conf
        }
    }
}


impl SpacecraftDynamic {
    pub fn new() -> SpacecraftDynamic {
        SpacecraftDynamic {
            t: 0.0,

            eng_gimbal: 0.0,
            eng_throttle: 0.0,

            fuel_mass: 8400.0,  // TODO from conf

            pos: Vec2 {x: 0.0, y: 15_000.0},  // TODO from conf
            vel: Vec2 {x: 1673.0, y: 0.0},  // TODO from conf - (MOON_MU/(MOON_RADIUS+15_000))**.5
            acc: Vec2 {x: 0.0, y: 0.0},

            ang_pos: 180.0*PI/180.0,
            ang_vel: 0.0,
        }
    }
}
