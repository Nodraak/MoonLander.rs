use std::f64::consts::PI;

use crate::utils::space::G0;
use crate::utils::math::Vec2;


pub struct Conf {
    pub sc_width: f64,              // unit: m
    pub sc_height: f64,             // unit: m
    pub sc_dry_mass: f64,           // unit: kg
    pub sc_fuel_mass: f64,          // unit: kg

    pub sc_nominal_thrust: f64,     // unit: N
    pub sc_nominal_isp: f64,        // unit: s
    pub sc_nominal_mass_flow: f64,  // unit: kg/s

    pub sc_pos: Vec2,               // unit: m
    pub sc_vel: Vec2,               // unit: m/s

    pub gui_af_x: f64,              // unit: m/s**2
    pub gui_vf_x: f64,              // unit: m/s
    pub gui_pf_x: f64,              // unit: m

    pub gui_af_y: f64,              // unit: m/s**2
    pub gui_vf_y: f64,              // unit: m/s
    pub gui_pf_y: f64,              // unit: m

    // compute_tgo() assumes a 100% thrust, but on average you want the engine to run at X %
    pub gui_thrust_mul: f64,        // unit: -

    pub control_eng_gimbal_pos_max: f64,  // unit: deg
    pub control_eng_gimbal_vel_max: f64,  // unit: deg/s
}

impl Conf {
    pub fn new() -> Conf {
        Conf {
            sc_width: 4.0,
            sc_height: 8.0,
            sc_dry_mass: 6_800.0,
            sc_fuel_mass: 8400.0,

            sc_nominal_thrust: 45_000.0,
            sc_nominal_isp: 311.0,
            sc_nominal_mass_flow: 45_000.0/(311.0*G0),

            sc_pos: Vec2 {x: 0.0, y: 15_000.0},
            sc_vel: Vec2 {x: 1673.0, y: 0.0},

            gui_af_x: 0.0,
            gui_vf_x: 0.0,
            gui_pf_x: 430_000.0,

            gui_af_y: 0.0,
            gui_vf_y: 0.0,
            gui_pf_y: 0.0,

            gui_thrust_mul: 0.80,

            control_eng_gimbal_pos_max: 4.0*PI/180.0,
            // Apollo dps: 0.2 deg / sec (https://www.ibiblio.org/apollo/Documents/SGA_Memo04_660120.pdf)
            control_eng_gimbal_vel_max: 1.0*PI/180.0,
        }
    }
}
