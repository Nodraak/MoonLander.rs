use std::f64::consts::PI;

use crate::adapters::common::ActuatorsValues;
use crate::gnc::common::Spacecraft;
use crate::utils::math::{Vec2, sign, saturate};


/// Main control function
pub fn ctr(spacecraft: &mut Spacecraft, goal_acc: Vec2) -> ActuatorsValues {
    let (ctr_thrust, ctr_angle) = spacecraft_controler(spacecraft, goal_acc);
    let ctr_eng_gimbal = engine_controler(spacecraft, (ctr_thrust, ctr_angle));

    spacecraft.cur.eng_throttle = ctr_thrust;
    spacecraft.cur.eng_gimbal = ctr_eng_gimbal;

    ActuatorsValues {
        engine_throttle: ctr_thrust,
        engine_gimbal: ctr_eng_gimbal,
    }
}

/// Spacecraft control function
///
/// Input:
///     goal_acc_x, goal_acc_y
///     spacecraft mass
///     engine max_thrust
/// Output:
///     commanded thrust (respecting engine constraints)
///     commanded (ideal) (spacecraft) attitude angle
fn spacecraft_controler(spacecraft: &Spacecraft, goal_acc: Vec2) -> (f64, f64) {
    // instead of wasting propelant, let gravity work
    if goal_acc.y < 0.0 {
        println!("WARN: gravity");
        return (1.0, PI);
    }

    let mass = spacecraft.spec.dry_mass + spacecraft.cur.fuel_mass;

    let mut ctr_thrust = (goal_acc.x.powi(2) + goal_acc.y.powi(2)).powf(0.5) * mass;

    let mut ctr_angle;

    // best case, control is possible
    if ctr_thrust < spacecraft.spec.nominal_thrust {
        ctr_angle = goal_acc.y.atan2(goal_acc.x);
    }
    // else, try to save what can be saved (fulfill y, best effort x)
    else {
        println!("WARN: thrust norm {}", ctr_thrust/spacecraft.spec.nominal_thrust);
        ctr_thrust = spacecraft.spec.nominal_thrust;

        ctr_angle = (goal_acc.y*mass/spacecraft.spec.nominal_thrust).asin();
        if goal_acc.x < 0.0 {
            ctr_angle = PI-ctr_angle;
        }
    }

    (ctr_thrust, ctr_angle)
}

/// Engine controller function
///
/// Input:
///     sc_attitude_desired, sc_attitude_current
///     sc_mass or sc_moment_of_inertia
///     sc_eng_thrust, sc_eng_gimbal_current, sc_eng_gimbal_max
/// Output:
///     commanded (engine) gimbal_angle (respecting engine constraints)
fn engine_controler(spacecraft: &Spacecraft, (ctr_thrust, ctr_angle): (f64, f64)) -> f64 {

    let max_eng_gimbal_pos = 4.0*3.14/180.0;  // 4 deg
    let max_eng_gimbal_vel = 1.0*3.14/180.0;  // 1 deg/sec - apollo dps: 0.2 deg / sec (https://www.ibiblio.org/apollo/Documents/SGA_Memo04_660120.pdf)

    // compute att error

    let sc_attitude_desired = ctr_angle;
    let sc_attitude_current = spacecraft.cur.ang_pos;

    let sc_attitude_error = sc_attitude_desired - sc_attitude_current;

    // compute torque for correction

    let sc_mass = spacecraft.spec.dry_mass + spacecraft.cur.fuel_mass;
    let sc_moment_of_inertia = 0.5 * sc_mass * 2.0_f64.powi(2);  // 1/2*m*r**2 = kg.m**2

    // TODO check sc angular vel: max 5 deg/sec -> How? Or dont check at all?

    let ctr_sc_att_acc = sc_attitude_error; // TODO PID. For now, assume a correction with T = 1 everywhere

    let ctr_torque = sc_moment_of_inertia * ctr_sc_att_acc;  // N*m = kg*m**2 * rad/sec**2

    // compute engine gimbal

    let eng_gimbal_current = spacecraft.cur.eng_gimbal;

    let mut ctr_eng_gimbal = (ctr_torque/(4.0*spacecraft.spec.nominal_thrust)).asin();  // Torque = L*F*sin(alpha)
    ctr_eng_gimbal = saturate(ctr_eng_gimbal, -max_eng_gimbal_pos, max_eng_gimbal_pos);

    let eng_gimbal_err = ctr_eng_gimbal - eng_gimbal_current;

    if eng_gimbal_err.abs() > max_eng_gimbal_vel {
        ctr_eng_gimbal = eng_gimbal_current + sign(eng_gimbal_err)*max_eng_gimbal_vel;
    }

    // return

    ctr_eng_gimbal
}
