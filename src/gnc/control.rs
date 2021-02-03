use std::f64::consts::PI;

use crate::adapters::common::ActuatorsValues;
use crate::conf::Conf;
use crate::gnc::common::Spacecraft;
use crate::utils::math::{Vec2, sign, saturate};


/// Main control function
pub fn ctr(spacecraft: &mut Spacecraft, goal_acc: Vec2) -> ActuatorsValues {
    let sc_mass = spacecraft.conf.sc_dry_mass + spacecraft.cur.fuel_mass;
    let sc_thrust = spacecraft.conf.sc_nominal_thrust;
    let sc_ang_pos = spacecraft.cur.ang_pos;
    let sc_ang_vel = spacecraft.cur.ang_vel;
    let eng_gimbal_cur = spacecraft.cur.eng_gimbal*spacecraft.conf.ctr_eng_gimbal_pos_max;

    let (ctr_sc_thrust, ctr_ang_pos) = spacecraft_controler(goal_acc, sc_mass, sc_thrust);
    let ctr_eng_gimbal = engine_controler(
        &spacecraft.conf, ctr_ang_pos, sc_mass, sc_ang_pos, sc_ang_vel, eng_gimbal_cur,
    );

    spacecraft.cur.eng_throttle = ctr_sc_thrust;
    spacecraft.cur.eng_gimbal = ctr_eng_gimbal / spacecraft.conf.ctr_eng_gimbal_pos_max;

    ActuatorsValues {
        engine_throttle: spacecraft.cur.eng_throttle,
        engine_gimbal: spacecraft.cur.eng_gimbal,
    }
}


/// Spacecraft control function (high level control)
///
/// Input:
///     goal_acc_x, goal_acc_y
///     spacecraft mass
///     engine max_thrust
/// Output:
///     commanded thrust (respecting engine constraints) as [0; 1] of max (nominal) thrust
///     commanded (ideal) (spacecraft) attitude angle
fn spacecraft_controler(goal_acc: Vec2, sc_mass: f64, sc_thrust: f64) -> (f64, f64) {
    // instead of wasting propelant, let gravity work
    if goal_acc.y < 0.0 {
        println!("WARN: gravity");
        (1.0, PI)
    } else {
        let mut ctr_thrust = (goal_acc.x.powi(2) + goal_acc.y.powi(2)).powf(0.5) * sc_mass;

        let mut ctr_angle;

        // best case, control is possible
        if ctr_thrust < sc_thrust {
            ctr_angle = goal_acc.y.atan2(goal_acc.x);
        }
        // else, try to save what can be saved (fulfill y, best effort x)
        else {
            println!("WARN: thrust norm {} times the available thrust", ctr_thrust/sc_thrust);
            ctr_thrust = sc_thrust;

            ctr_angle = (goal_acc.y*sc_mass/sc_thrust).asin();
            if goal_acc.x < 0.0 {
                ctr_angle = PI-ctr_angle;
            }
        }

        (ctr_thrust/sc_thrust, ctr_angle)
    }
}


/// Engine controller function (low level control)
///
/// Controller implemented as a double (cascade) PID to control the spacecraft
/// ang_pos via its ang_acc (engine gimbal).
///
/// Input:
///     sc_attitude_desired, sc_attitude_current
///     sc_mass or sc_moment_of_inertia
///     sc_eng_thrust, sc_eng_gimbal_current, sc_eng_gimbal_max
/// Output:
///     commanded (engine) gimbal_angle (respecting engine constraints) as [-1; 1] of max gimbal
///
/// Double PID block diagram:
///
/// ctr_ang_pos ----\
///                 |
/// sc_ang_pos -----O
///                 | ang_pos_err
///                 V
///               PID 1
///                 |
///                 | ctr_ang_vel
///                 |
/// sc_ang_vel -----O
///                 | ang_vel_err
///                 V
///               PID 2
///                 |
///                 | ctr_ang_acc
///
fn engine_controler(conf: &Conf, ctr_ang_pos: f64, sc_mass: f64, sc_ang_pos: f64, sc_ang_vel: f64, eng_gimbal_cur: f64) -> f64 {

    // ang_pos_err and ctr_ang_vel - PID 1

    let ang_pos_err = ctr_ang_pos - sc_ang_pos;
    let ctr_ang_vel = ang_pos_err * 0.5; // TODO PD. For now, assume a correction with T = 1 everywhere
    // TODO check ctr_ang_vel: max 5 deg/sec -> How? Or dont check at all?

    // ang_vel_err and ctr_ang_acc - PID 2

    let ang_vel_err = ctr_ang_vel - sc_ang_vel;
    let ctr_ang_acc = ang_vel_err; // TODO PD. For now, assume a correction with T = 1 everywhere

    // compute torque for correction

    let sc_moment_of_inertia = 0.5 * sc_mass * (conf.sc_width/2.0).powi(2);  // 1/2*m*r**2 = kg.m**2
    let ctr_torque = ctr_ang_acc * sc_moment_of_inertia;  // N*m = kg*m**2 * rad/sec**2

    // compute engine gimbal

    let max_eng_gimbal_pos = conf.ctr_eng_gimbal_pos_max;
    let max_eng_gimbal_vel = conf.ctr_eng_gimbal_vel_max;

    let mut ctr_eng_gimbal = (ctr_torque/(conf.sc_height/2.0*conf.sc_nominal_thrust)).asin();  // Torque = L*F*sin(alpha)
    ctr_eng_gimbal = saturate(ctr_eng_gimbal, -max_eng_gimbal_pos, max_eng_gimbal_pos);

    let eng_gimbal_err = ctr_eng_gimbal - eng_gimbal_cur;

    if eng_gimbal_err.abs() > max_eng_gimbal_vel {
        ctr_eng_gimbal = eng_gimbal_cur + sign(eng_gimbal_err)*max_eng_gimbal_vel;
    }

    // return

    ctr_eng_gimbal
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::assert_approx_eq;

    #[test]
    fn test_gui_1_spacecraft_angle() {
        let mass = 1_000.0;
        let thrust = 20_000.0;

        let (ctr_thrust, ctr_angle) = spacecraft_controler(Vec2 {x: 10.0, y: 0.0}, mass, thrust);
        assert_eq!(ctr_angle, 0.0);

        let (ctr_thrust, ctr_angle) = spacecraft_controler(Vec2 {x: 7.071067811865475, y: 7.071067811865475}, mass, thrust);
        assert_eq!(ctr_angle, PI/4.0);

        let (ctr_thrust, ctr_angle) = spacecraft_controler(Vec2 {x: 0.0, y: 10.0}, mass, thrust);
        assert_eq!(ctr_angle, PI/2.0);

        let (ctr_thrust, ctr_angle) = spacecraft_controler(Vec2 {x: -7.071067811865475, y: 7.071067811865475}, mass, thrust);
        assert_eq!(ctr_angle, 3.0*PI/4.0);

        let (ctr_thrust, ctr_angle) = spacecraft_controler(Vec2 {x: -10.0, y: 0.0}, mass, thrust);
        assert_eq!(ctr_angle, PI);
    }

    #[test]
    fn test_gui_2_spacecraft_thrust() {
        let mass = 1_000.0;
        let thrust = 20_000.0;

        // +x

        let (ctr_thrust, ctr_angle) = spacecraft_controler(Vec2 {x: 10.0, y: 0.0}, mass, thrust);
        assert_eq!(ctr_thrust, 10_000.0);

        let (ctr_thrust, ctr_angle) = spacecraft_controler(Vec2 {x: 20.0, y: 0.0}, mass, thrust);
        assert_eq!(ctr_thrust, 20_000.0);

        let (ctr_thrust, ctr_angle) = spacecraft_controler(Vec2 {x: 30.0, y: 0.0}, mass, thrust);
        assert_eq!(ctr_thrust, 20_000.0);

        // -x

        let (ctr_thrust, ctr_angle) = spacecraft_controler(Vec2 {x: -10.0, y: 0.0}, mass, thrust);
        assert_eq!(ctr_thrust, 10_000.0);

        let (ctr_thrust, ctr_angle) = spacecraft_controler(Vec2 {x: -20.0, y: 0.0}, mass, thrust);
        assert_eq!(ctr_thrust, 20_000.0);

        let (ctr_thrust, ctr_angle) = spacecraft_controler(Vec2 {x: -30.0, y: 0.0}, mass, thrust);
        assert_eq!(ctr_thrust, 20_000.0);

        // +y

        let (ctr_thrust, ctr_angle) = spacecraft_controler(Vec2 {x: 0.0, y: 10.0}, mass, thrust);
        assert_eq!(ctr_thrust, 10_000.0);

        let (ctr_thrust, ctr_angle) = spacecraft_controler(Vec2 {x: 0.0, y: 20.0}, mass, thrust);
        assert_eq!(ctr_thrust, 20_000.0);

        let (ctr_thrust, ctr_angle) = spacecraft_controler(Vec2 {x: 0.0, y: 30.0}, mass, thrust);
        assert_eq!(ctr_thrust, 20_000.0);

        // -y -> gravity

        let (ctr_thrust, ctr_angle) = spacecraft_controler(Vec2 {x: 0.0, y: -10.0}, mass, thrust);
        assert_eq!(ctr_thrust, thrust);
        assert_eq!(ctr_angle, PI);

        let (ctr_thrust, ctr_angle) = spacecraft_controler(Vec2 {x: 0.0, y: -20.0}, mass, thrust);
        assert_eq!(ctr_thrust, thrust);
        assert_eq!(ctr_angle, PI);

        let (ctr_thrust, ctr_angle) = spacecraft_controler(Vec2 {x: 0.0, y: -30.0}, mass, thrust);
        assert_eq!(ctr_thrust, thrust);
        assert_eq!(ctr_angle, PI);
    }

    #[test]
    fn test_gui_3_engine() {
        {
            let sc_mass = 1_000.0;  // Kg
            let sc_thrust = 20_000.0;  // N = Kg*m/s**2
            let sc_att_cur = 0.0*PI/180.0;  // rad
            let eng_gimbal_cur = 0.0*PI/180.0;  // rad

            let ctr_angle = 10.0*PI/180.0;  // rad
            let eng_gimbal_ref = engine_controler(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
            assert_approx_eq!(eng_gimbal_ref, 0.25000079328132707*PI/180.0, 1e-6);

            // check linear (PID)

            {
                let ctr_angle = 2.0*10.0*PI/180.0;  // rad
                let eng_gimbal = engine_controler(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
                assert_approx_eq!(eng_gimbal, 2.0*eng_gimbal_ref, 1e-6);
            }

            {
                let ctr_angle = 3.0*10.0*PI/180.0;  // rad
                let eng_gimbal = engine_controler(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
                assert_approx_eq!(eng_gimbal, 3.0*eng_gimbal_ref, 1e-6);
            }

            // check ang vel max

            let max_eng_gimbal_vel = 1.0*PI/180.0;  // TODO conf

            {
                let ctr_angle = 3.0*10.0*PI/180.0;  // rad
                let eng_gimbal = engine_controler(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
                assert_approx_eq!(eng_gimbal, 3.0*eng_gimbal_ref, 1e-6);
            }
            {
                let ctr_angle = 4.0*10.0*PI/180.0;  // rad
                let eng_gimbal = engine_controler(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
                assert_approx_eq!(eng_gimbal, max_eng_gimbal_vel, 1e-6);
            }
            {
                let ctr_angle = 5.0*10.0*PI/180.0;  // rad
                let eng_gimbal = engine_controler(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
                assert_approx_eq!(eng_gimbal, max_eng_gimbal_vel, 1e-6);
            }
        }

        // check ang pos max

        {
            let sc_mass = 1_000.0;  // Kg
            let sc_thrust = 10_000.0;  // N = Kg*m/s**2
            let sc_att_cur = 0.0*PI/180.0;  // rad

            // let max_eng_gimbal_pos = 4.0*PI/180.0;  // TODO conf

            let eng_gimbal_cur = 3.5*PI/180.0;  // rad

            // ref

            let ctr_angle = 70.0*PI/180.0;  // rad
            let eng_gimbal_ref = engine_controler(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
            assert_approx_eq!(eng_gimbal_ref, 3.5*PI/180.0, 1e-4);

            // check linear (PID)

            let ctr_angle = 60.0*PI/180.0;  // rad
            let eng_gimbal = engine_controler(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
            assert_approx_eq!(eng_gimbal, eng_gimbal_ref - 2.0*0.25*PI/180.0, 1e-4);

            let ctr_angle = 65.0*PI/180.0;  // rad
            let eng_gimbal = engine_controler(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
            assert_approx_eq!(eng_gimbal, eng_gimbal_ref - 1.0*0.25*PI/180.0, 1e-4);

            let ctr_angle = 70.0*PI/180.0;  // rad
            let eng_gimbal = engine_controler(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
            assert_approx_eq!(eng_gimbal, eng_gimbal_ref, 1e-6);

            let ctr_angle = 75.0*PI/180.0;  // rad
            let eng_gimbal = engine_controler(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
            assert_approx_eq!(eng_gimbal, eng_gimbal_ref + 1.0*0.25*PI/180.0, 1e-4);

            let ctr_angle = 80.0*PI/180.0;  // rad
            let eng_gimbal = engine_controler(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
            assert_approx_eq!(eng_gimbal, eng_gimbal_ref + 2.0*0.25*PI/180.0, 1e-4);

            // max

            let ctr_angle = 85.0*PI/180.0;  // rad
            let eng_gimbal = engine_controler(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
            assert_approx_eq!(eng_gimbal, eng_gimbal_ref + 2.0*0.25*PI/180.0, 1e-4);

            let ctr_angle = 90.0*PI/180.0;  // rad
            let eng_gimbal = engine_controler(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
            assert_approx_eq!(eng_gimbal, eng_gimbal_ref + 2.0*0.25*PI/180.0, 1e-4);
        }
    }

    #[test]
    fn test_gui_4_ctr() {
        // TODO test ctr()
    }
}
