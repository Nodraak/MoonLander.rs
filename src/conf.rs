use serde::{Serialize, Deserialize};

use crate::utils::math::Vec2;
use crate::utils::bodies::Body;


#[derive(Clone, Copy)]
#[derive(Serialize, Deserialize)]
pub struct Scenario {
    pub body: Body,                     // orbited body, containing info about gravity, atmospheric, etc

    pub initial_sc_pos: Vec2,       // unit: m
    pub initial_sc_vel: Vec2,       // unit: m/s
    pub initial_sc_ang_pos: f64,    // unit: rad
    pub initial_sc_fuel_mass: f64,  // unit: kg

    pub sc_width: f64,              // unit: m
    pub sc_height: f64,             // unit: m
    pub sc_dry_mass: f64,           // unit: kg
    pub sc_cd: f64,                 // unit: - - drag coefficient (https://en.wikipedia.org/wiki/Drag_coefficient)

    pub sc_nominal_thrust: f64,     // unit: N
    pub sc_nominal_isp: f64,        // unit: s
    pub sc_nominal_mass_flow: f64,  // unit: kg/s

    pub gui_af_x: f64,              // unit: m/s**2
    pub gui_vf_x: f64,              // unit: m/s
    pub gui_pf_x: f64,              // unit: m

    pub gui_af_y: f64,              // unit: m/s**2
    pub gui_vf_y: f64,              // unit: m/s
    pub gui_pf_y: f64,              // unit: m

    // compute_tgo() assumes a 100% thrust, but on average you want the engine to run at X %
    pub gui_thrust_mul: f64,        // unit: -

    pub ctr_eng_gimbal_pos_max: f64,    // unit: rad
    pub ctr_eng_gimbal_vel_max: f64,    // unit: rad/s
}


#[derive(Clone, Copy)]
pub struct Conf {
    pub dt_step: f64,               // unit: sec
    pub dt_sleep: f64,              // unit: sec

    pub s: Scenario,
}


impl Conf {
    pub fn new(dt_step: f64, dt_sleep: f64, scenario: Scenario) -> Conf {
        Conf {
            dt_step: dt_step,
            dt_sleep: dt_sleep,
            s: scenario,
        }
    }
}
