use std::f64::consts::E;

use lazy_static::lazy_static;
use uom::si::f64::*;
use uom::si::acceleration::meter_per_second_squared;
use uom::si::angle::degree;
use uom::si::length::meter;
use uom::si::ratio::{percent, ratio};
use uom::si::time::second;
use uom::si::velocity::meter_per_second;

use crate::gnc::common::Spacecraft;
use crate::squared;


lazy_static! {
    // https://en.wikipedia.org/wiki/Gravity_of_Earth
    pub static ref G0: Acceleration = Acceleration::new::<meter_per_second_squared>(9.806);
}


/// Estimate TGO for current spacecraft properties and pos and vel goals
///
/// dv_x = diff vel_x
/// dv_y_vel = diff vel_y
/// dv_y_gravity = integral from 0 to tgo of body_gravity()-body_centrifugal()
///     Note: vx = vx0 - t*vx0/tgo = vx0/tgo * (tgo-t)
pub fn tgo_estimate(craft: &Spacecraft, final_vel_x_goal: Velocity, final_vel_y_goal: Velocity, thrust_mul: Ratio) -> Time {
    let conf = craft.conf.s;

    let mass = conf.sc_dry_mass + craft.cur.fuel_mass;

    let dv_x = craft.cur.vel.x-final_vel_x_goal;
    let dv_y_vel = craft.cur.vel.y-final_vel_y_goal;

    let mut tgo = Time::new::<second>(0.0);  // initial estimate
    for _ in 0..5 {  // 5 loops are more than enough
        let dv_y_gravity = tgo * (
            conf.body.gravity(craft.cur.pos.y)
            - (squared!(craft.cur.vel.x)) / (3.0 * conf.body.radius)
        );
        // x**2+y**2 underestimate ; x+y overestimate ; it is better to over estimate
        let dv = dv_x.abs() + (dv_y_gravity-dv_y_vel).abs();
        let epow: Ratio = dv/(conf.sc_nominal_isp*(*G0));
        tgo = mass * (
            1.0 - 1.0 / E.powf(epow.get::<ratio>())
        ) / (conf.sc_nominal_mass_flow*thrust_mul);
    }

    tgo
}


pub fn has_softly_landed(craft: &Spacecraft) -> bool {
    if (
        (craft.cur.pos.y < Length::new::<meter>(1.0))
        && (craft.cur.vel.y < Velocity::new::<meter_per_second>(0.2)) && (craft.cur.vel.x < Velocity::new::<meter_per_second>(1.0))
        && (craft.cur.eng_throttle < Ratio::new::<percent>(30.0)) && (craft.cur.ang_pos < Angle::new::<degree>(100.0))
    ) {
        true
    } else {
        false
    }
}
