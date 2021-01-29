use std::f64::consts::{E, PI};

use crate::gnc::common::Spacecraft;


pub const G0: f64 = 9.806;

pub const MOON_RADIUS: f64 = 1737.4 * 1e3;      // m
pub const MOON_MASS: f64 = 7.342 * 1e22;        // kg
pub const MOON_MU: f64 = 4.9048695 * 1e12;      // m**3/s**2

pub const MOON_ORBITAL_15000: f64 = 1673.004;   // m/s - (MOON_MU/(MOON_RADIUS+15_000.0)).powf(0.5)

// TODO

// def dv(isp, m_dry, m_wet):
//     return isp*G0*log(m_wet/m_dry)

// def orbital_speed(mu, a, r):
//     return sqrt(mu*(2/r - 1/a))


// def moon_orbital_speed(a, alt):
//     return orbital_speed(MOON_MU, a, MOON_RADIUS+alt)

pub fn moon_gravity(altitude: f64) -> f64 {
    MOON_MU/(MOON_RADIUS+altitude).powi(2)
}

pub fn moon_centrifugal(vel: f64, altitude: f64) -> f64 {
    vel.powi(2)/(MOON_RADIUS+altitude)
}


/// Estimate TGO for current spacecraft properties and pos and vel goals
///
/// dv_x = diff vel_x
/// dv_y_vel = diff vel_y
/// dv_y_gravity = integral from 0 to tgo of moon_gravity()-moon_centrifugal()
///     Note: vx = vx0 - t*vx0/tgo = vx0/tgo * (tgo-t)
pub fn tgo_estimate(craft: &Spacecraft, final_vel_x_goal: f64, final_vel_y_goal: f64, thrust_mul: f64) -> f64 {
    let mass = craft.spec.dry_mass + craft.cur.fuel_mass;

    let dv_x = craft.cur.vel.x-final_vel_x_goal;
    let dv_y_vel = craft.cur.vel.y-final_vel_y_goal;

    let mut tgo = 0.0;  // initial estimate
    for _ in 0..5 {  // 5 loops are more than enough
        let dv_y_gravity = tgo * (moon_gravity(craft.cur.pos.y) - (craft.cur.vel.x.powi(2)) / (3.0 * MOON_RADIUS));
        // x**2+y**2 underestimate ; x+y overestimate ; it is better to over estimate
        let dv = dv_x.abs() + (dv_y_gravity-dv_y_vel).abs();
        tgo = mass * (1.0 - 1.0 / E.powf(dv/(craft.spec.nominal_isp*G0))) / (craft.spec.nominal_mass_flow*thrust_mul);
    }

    tgo
}


pub fn has_softly_landed(craft: &Spacecraft) -> bool {
    if (
        (craft.cur.pos.y < 1.0) && (craft.cur.vel.y < 0.2) && (craft.cur.vel.x < 1.0)
        && (craft.cur.eng_throttle < 0.30) && (craft.cur.ang_pos < 100.0*PI/180.0)
    ) {
        true
    } else {
        false
    }
}
