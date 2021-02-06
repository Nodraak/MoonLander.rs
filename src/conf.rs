use serde::{Serialize, Deserialize};

use crate::utils::math::Vec2;
use crate::utils::bodies::Body;


#[derive(Clone, Copy)]
#[derive(PartialEq)]
#[derive(Serialize, Deserialize)]
pub enum SubCommand {
    Sim,
    Ksp,
}


#[derive(Clone, Copy)]
#[derive(PartialEq)]
#[derive(Serialize, Deserialize)]
pub enum TgoEstimate {
    TgoGivenFixed,
    TgoEstimateFixed,
    TgoEstimateUpdating,
}

#[derive(Clone, Copy)]
#[derive(PartialEq)]
#[derive(Serialize, Deserialize)]
pub enum GuiSpacecraft {
    GuiDescent,
    GuiAscent,
}

#[derive(Clone, Copy)]
#[derive(PartialEq)]
#[derive(Serialize, Deserialize)]
pub enum CtrSpacecraft {
    CtrSpacecraftDescent,
    CtrSpacecraftAscent,
}


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

    pub ctr_eng_gimbal_pos_max: f64,    // unit: rad
    pub ctr_eng_gimbal_vel_max: f64,    // unit: rad/s

    // GNC functions (descent/ascent/...)

    pub tgo_method: TgoEstimate,
    pub tgo_init: f64,                  // unit: s - used only with TgoEstimate::TgoGivenFixed
    pub tgo_thrust_mul: f64,            // unit: - - used only with TgoEstimate::TgoEstimateFixed
                                        //    | TgoEstimate::TgoEstimateUpdating, estimate_tgo() assumes a 100% thrust,
                                        //    but on average you want the  engine to run at X %
    pub tgo_stop: f64,                  // unit: s - stop the loop a few seconds before touchdown, to prevent guidance
                                        //    from diverging to +/- inf
    pub gui_spacecraft: GuiSpacecraft,
    pub ctr_spacecraft: CtrSpacecraft,
}


#[derive(Clone, Copy)]
pub struct Conf {
    pub subcommand: SubCommand,     // sim | ksp
    pub dt_step: f64,               // unit: sec
    pub dt_sleep: f64,              // unit: sec

    pub s: Scenario,
}


impl Conf {
    pub fn new(subcommand: SubCommand, dt_step: f64, dt_sleep: f64, scenario: Scenario) -> Conf {
        Conf {
            subcommand: subcommand,
            dt_step: dt_step,
            dt_sleep: dt_sleep,
            s: scenario,
        }
    }
}
