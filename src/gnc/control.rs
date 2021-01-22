use std::f64::consts::PI;

use crate::adapters::common::ActuatorsValues;
use crate::gnc::common::Spacecraft;
use crate::utils::math::{Vec2, sign, saturate};


/// Main control function
pub fn ctr(spacecraft: &mut Spacecraft, goal_acc: Vec2) -> ActuatorsValues {
    let sc_mass = spacecraft.spec.dry_mass + spacecraft.cur.fuel_mass;
    let sc_thrust = spacecraft.spec.nominal_thrust;
    let sc_att_cur = spacecraft.cur.ang_pos;
    let eng_gimbal_cur = spacecraft.cur.eng_gimbal;

    let (ctr_sc_thrust, ctr_sc_angle) = spacecraft_controler(goal_acc, sc_mass, sc_thrust);
    let ctr_eng_gimbal = engine_controler(ctr_sc_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);

    spacecraft.cur.eng_throttle = ctr_sc_thrust/sc_thrust;
    spacecraft.cur.eng_gimbal = ctr_eng_gimbal;

    ActuatorsValues {
        engine_throttle: spacecraft.cur.eng_throttle,
        engine_gimbal: spacecraft.cur.eng_gimbal,
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
fn spacecraft_controler(goal_acc: Vec2, sc_mass: f64, sc_thrust: f64) -> (f64, f64) {
    // instead of wasting propelant, let gravity work
    if goal_acc.y < 0.0 {
        println!("WARN: gravity");
        return (sc_thrust, PI);
    }

    let mut ctr_thrust = (goal_acc.x.powi(2) + goal_acc.y.powi(2)).powf(0.5) * sc_mass;

    let mut ctr_angle;

    // best case, control is possible
    if ctr_thrust < sc_thrust {
        ctr_angle = goal_acc.y.atan2(goal_acc.x);
    }
    // else, try to save what can be saved (fulfill y, best effort x)
    else {
        println!("WARN: thrust norm {}", ctr_thrust/sc_thrust);
        ctr_thrust = sc_thrust;

        ctr_angle = (goal_acc.y*sc_mass/sc_thrust).asin();
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
fn engine_controler(ctr_angle: f64, sc_mass: f64, sc_thrust: f64, sc_att_cur: f64, eng_gimbal_cur: f64) -> f64 {

    // TODO conf
    let max_eng_gimbal_pos = 4.0*PI/180.0;  // 4 deg
    let max_eng_gimbal_vel = 1.0*PI/180.0;  // 1 deg/sec - apollo dps: 0.2 deg / sec (https://www.ibiblio.org/apollo/Documents/SGA_Memo04_660120.pdf)
    let sc_width: f64 = 4.0;  // m
    let sc_height = 8.0;  // m

    // compute att error

    let sc_attitude_error = ctr_angle - sc_att_cur;

    // compute torque for correction

    let sc_moment_of_inertia = 0.5 * sc_mass * (sc_width/2.0).powi(2);  // 1/2*m*r**2 = kg.m**2

    // TODO check sc angular vel: max 5 deg/sec -> How? Or dont check at all?

    let ctr_sc_att_acc = sc_attitude_error; // TODO PID. For now, assume a correction with T = 1 everywhere

    let ctr_torque = sc_moment_of_inertia * ctr_sc_att_acc;  // N*m = kg*m**2 * rad/sec**2

    // compute engine gimbal

    let mut ctr_eng_gimbal = (ctr_torque/(sc_height/2.0*sc_thrust)).asin();  // Torque = L*F*sin(alpha)
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
