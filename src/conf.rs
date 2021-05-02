use serde::{Serialize, Deserialize};
use uom::si::f64::*;
use uom::si::ratio::ratio;
use uom::si::time::second;

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
    GuiAscentToOrbit,
    GuiAscentToHover,
}

#[derive(Clone, Copy)]
#[derive(PartialEq)]
#[derive(Serialize, Deserialize)]
pub enum CtrSpacecraft {
    CtrSpacecraftDescent,
    CtrSpacecraftAscentToOrbit,
    CtrSpacecraftAscentToHover,
}


#[derive(Clone, Copy)]
#[derive(Serialize, Deserialize)]
pub struct Scenario {
    pub body: Body,                     // orbited body, containing info about gravity, atmospheric, etc

    pub initial_sc_pos: Vec2<Length>,
    pub initial_sc_vel: Vec2<Velocity>,
    pub initial_sc_ang_pos: Angle,
    pub initial_sc_fuel_mass: Mass,

    pub sc_width: Length,
    pub sc_height: Length,
    pub sc_dry_mass: Mass,
    pub sc_cd: Ratio,                       // drag coefficient (https://en.wikipedia.org/wiki/Drag_coefficient)

    pub sc_nominal_thrust: Force,
    pub sc_nominal_isp: Time,
    pub sc_nominal_mass_flow: MassRate,

    pub gui_af_x: Acceleration,
    pub gui_vf_x: Velocity,
    pub gui_pf_x: Length,

    pub gui_af_y: Acceleration,
    pub gui_vf_y: Velocity,
    pub gui_pf_y: Length,

    pub ctr_eng_gimbal_tau: Time,
    pub ctr_eng_gimbal_kp: Option<Ratio>,
    pub ctr_eng_gimbal_kd: Option<Time>,

    pub ctr_eng_gimbal_pos_max: Angle,
    pub ctr_eng_gimbal_vel_max: AngularVelocity,

    // GNC functions (descent/ascent/...)

    pub tgo_method: TgoEstimate,
    pub tgo_init: Time,                 // used only with TgoEstimate::TgoGivenFixed
    pub tgo_thrust_mul: Ratio,          // used only with TgoEstimate::TgoEstimateFixed
                                        //    | TgoEstimate::TgoEstimateUpdating, estimate_tgo() assumes a 100% thrust,
                                        //    but on average you want the  engine to run at X %
    pub tgo_stop: Time,                 // stop the loop a few seconds before touchdown, to prevent guidance
                                        //    from diverging to +/- inf
    pub gui_spacecraft: GuiSpacecraft,
    pub ctr_spacecraft: CtrSpacecraft,
}

impl Scenario {
    pub fn load(filepath: &str) -> Self {
        // parameters estimated with a power regression from simulation data
        let kp_scale = 4.224639;
        let kp_exponent = -1.524106;
        let kd_scale = -2.203941;
        let kd_exponent = -0.759514;

        let f = std::fs::File::open(filepath).unwrap();
        let mut scenario: Scenario = serde_yaml::from_reader(f).unwrap();

        scenario.ctr_eng_gimbal_kp = Some(Ratio::new::<ratio>(
            kp_scale*scenario.ctr_eng_gimbal_tau.get::<second>().powf(kp_exponent)
        ));
        scenario.ctr_eng_gimbal_kd = Some(Time::new::<second>(
            kd_scale*scenario.ctr_eng_gimbal_tau.get::<second>().powf(kd_exponent)
        ));

        scenario
    }
}


#[derive(Clone, Copy)]
pub struct Conf {
    pub subcommand: SubCommand,     // sim | ksp
    pub dt_step: Time,
    pub dt_sleep: Time,

    pub s: Scenario,
}


impl Conf {
    pub fn new(subcommand: SubCommand, dt_step: Time, dt_sleep: Time, scenario: Scenario) -> Conf {
        Conf {
            subcommand: subcommand,
            dt_step: dt_step,
            dt_sleep: dt_sleep,
            s: scenario,
        }
    }
}
