use std::f64::consts::PI;

use uom::si::f64::*;
use uom::si::acceleration::meter_per_second_squared;
use uom::si::angle::{degree, radian};
use uom::si::ratio::ratio;
use uom::si::time::second;

use crate::{sqrt, squared};
use crate::adapters::common::ActuatorsValues;
use crate::conf::{Scenario, CtrSpacecraft};
use crate::gnc::common::Spacecraft;
use crate::utils::math::{Vec2, sign, saturate};


/// Main control function
pub fn ctr(spacecraft: &mut Spacecraft) -> ActuatorsValues {
    let conf = spacecraft.conf.s;
    let sc_mass = conf.sc_dry_mass + spacecraft.cur.fuel_mass;
    let sc_thrust = conf.sc_nominal_thrust;
    let sc_ang_pos = spacecraft.cur.ang_pos;
    let sc_ang_vel = spacecraft.cur.ang_vel;
    let eng_gimbal_cur: Angle = (spacecraft.cur.eng_gimbal*conf.ctr_eng_gimbal_pos_max).into();
    let goal_acc = spacecraft.cur.gui;

    let (ctr_sc_thrust, ctr_ang_pos): (Force, Angle) = match conf.ctr_spacecraft {
        CtrSpacecraft::CtrSpacecraftDescent | CtrSpacecraft::CtrSpacecraftAscentToHover => {
            control_translation(goal_acc, sc_mass, sc_thrust)
        },
        CtrSpacecraft::CtrSpacecraftAscentToOrbit => {
            let (_thrust, ctr_ang_pos_optim): (Force, Angle) = control_translation(goal_acc, sc_mass, sc_thrust);

            // to avoid a dangerously big angular command, perform a nice constant pitch rate
            let tf = Time::new::<second>(50.0);

            if spacecraft.cur.t > tf {
                (sc_thrust, ctr_ang_pos_optim)
            } else {
                let af = ctr_ang_pos_optim;

                let na: Angle = (spacecraft.cur.t/tf*(Angle::new::<degree>(90.0)-af)).into();

                let ctr_ang_pos: Angle = Angle::new::<degree>(90.0) - na;
                (sc_thrust, ctr_ang_pos)
            }
        },
    };

    let ctr_eng_gimbal: Angle = control_angular(
        &conf, spacecraft.cur.dt, ctr_ang_pos, sc_mass, sc_ang_pos, sc_ang_vel, eng_gimbal_cur,
    );

    spacecraft.cur.eng_throttle = ctr_sc_thrust / sc_thrust;
    spacecraft.cur.eng_gimbal = ctr_eng_gimbal / conf.ctr_eng_gimbal_pos_max;

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
///     commanded thrust (respecting engine constraints)
///     commanded (ideal) (spacecraft) attitude angle
fn control_translation(goal_acc: Vec2<Acceleration>, sc_mass: Mass, sc_thrust: Force) -> (Force, Angle) {
    // instead of wasting propelant, let gravity work
    if goal_acc.y < Acceleration::new::<meter_per_second_squared>(0.0) {
        println!("WARN: gravity");
        (sc_thrust, Angle::new::<radian>(PI))
    } else {
        let mut ctr_thrust: Force = Acceleration::new::<meter_per_second_squared>(
            sqrt!(squared!(goal_acc.x) + squared!(goal_acc.y))
        ) * sc_mass;

        let mut ctr_angle;

        // best case, control is possible
        if ctr_thrust < sc_thrust {
            ctr_angle = goal_acc.y.atan2(goal_acc.x);
        }
        // else, try to save what can be saved (fulfill y, best effort x)
        else {
            println!("WARN: thrust norm {} times the available thrust", (ctr_thrust/sc_thrust).get::<ratio>());
            ctr_thrust = sc_thrust;

            ctr_angle = (goal_acc.y*sc_mass/sc_thrust).asin();
            if goal_acc.x < Acceleration::new::<meter_per_second_squared>(0.0) {
                ctr_angle = Angle::new::<radian>(PI)-ctr_angle;
            }
        }

        (ctr_thrust, ctr_angle)
    }
}


/// Engine controller function (low level control)
///
/// Controller implemented as a PID to control the spacecraft ang_pos via its
/// ang_acc (engine gimbal).
///
/// Input:
///     sc_attitude_desired, sc_attitude_current
///     sc_mass or sc_moment_of_inertia
///     sc_eng_thrust, sc_eng_gimbal_current, sc_eng_gimbal_max
/// Output:
///     commanded (engine) gimbal_angle (respecting engine constraints)
///
fn control_angular(conf: &Scenario, dt: Time, ctr_ang_pos: Angle, sc_mass: Mass, sc_ang_pos: Angle, sc_ang_vel: AngularVelocity, eng_gimbal_cur: Angle) -> Angle {
    // some sanity checks
    assert!(Time::new::<second>(1e-6) < dt);
    assert!(dt <= Time::new::<second>(1.0));

    // some variable aliases

    let kp: Ratio = conf.ctr_eng_gimbal_kp.unwrap();
    let kd: Time = conf.ctr_eng_gimbal_kd.unwrap();

    // small hack to respect Dimensional analysis
    // convert Angle into AngularAcceleration
    let control_transfer_function = 1.0 / squared!(Time::new::<second>(1.0));

    // ang acc PID

    let err: Angle = ctr_ang_pos - sc_ang_pos;
    let derr: AngularVelocity = sc_ang_vel;  // TODO try derive err
    let control: Angle = (kp*err + kd*derr).into();
    let ctr_ang_acc: AngularAcceleration = (control * control_transfer_function).into();

    // compute torque for correction

    let sc_moment_of_inertia = 0.5 * sc_mass * squared!(conf.sc_width/2.0);  // 1/2*m*r**2 = kg.m**2
    let ctr_torque: Torque = (ctr_ang_acc * sc_moment_of_inertia).into();  // N*m = kg*m**2 * rad/sec**2

    // compute engine gimbal

    let sin_gimbal: Ratio = ctr_torque/(conf.sc_height/2.0*conf.sc_nominal_thrust);  // Torque = L*F*sin(alpha)
    assert!(sin_gimbal.abs() <= Ratio::new::<ratio>(1.0));

    let mut ctr_eng_gimbal: Angle = sin_gimbal.asin();

    // apply engine gimbal limits (vel and pos)

    let eng_gimbal_vel: AngularVelocity = ((ctr_eng_gimbal - eng_gimbal_cur)/dt).into();
    if eng_gimbal_vel.abs() > conf.ctr_eng_gimbal_vel_max {
        let inc: Angle = (sign(eng_gimbal_vel.value)*conf.ctr_eng_gimbal_vel_max*dt).into();
        ctr_eng_gimbal = eng_gimbal_cur + inc;
    }

    ctr_eng_gimbal = saturate(ctr_eng_gimbal, -conf.ctr_eng_gimbal_pos_max, conf.ctr_eng_gimbal_pos_max);

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

        let (ctr_thrust, ctr_angle) = control_translation(Vec2 {x: 10.0, y: 0.0}, mass, thrust);
        assert_eq!(ctr_angle, 0.0);

        let (ctr_thrust, ctr_angle) = control_translation(Vec2 {x: 7.071067811865475, y: 7.071067811865475}, mass, thrust);
        assert_eq!(ctr_angle, PI/4.0);

        let (ctr_thrust, ctr_angle) = control_translation(Vec2 {x: 0.0, y: 10.0}, mass, thrust);
        assert_eq!(ctr_angle, PI/2.0);

        let (ctr_thrust, ctr_angle) = control_translation(Vec2 {x: -7.071067811865475, y: 7.071067811865475}, mass, thrust);
        assert_eq!(ctr_angle, 3.0*PI/4.0);

        let (ctr_thrust, ctr_angle) = control_translation(Vec2 {x: -10.0, y: 0.0}, mass, thrust);
        assert_eq!(ctr_angle, PI);
    }

    #[test]
    fn test_gui_2_spacecraft_thrust() {
        let mass = 1_000.0;
        let thrust = 20_000.0;

        // +x

        let (ctr_thrust, ctr_angle) = control_translation(Vec2 {x: 10.0, y: 0.0}, mass, thrust);
        assert_eq!(ctr_thrust, 10_000.0);

        let (ctr_thrust, ctr_angle) = control_translation(Vec2 {x: 20.0, y: 0.0}, mass, thrust);
        assert_eq!(ctr_thrust, 20_000.0);

        let (ctr_thrust, ctr_angle) = control_translation(Vec2 {x: 30.0, y: 0.0}, mass, thrust);
        assert_eq!(ctr_thrust, 20_000.0);

        // -x

        let (ctr_thrust, ctr_angle) = control_translation(Vec2 {x: -10.0, y: 0.0}, mass, thrust);
        assert_eq!(ctr_thrust, 10_000.0);

        let (ctr_thrust, ctr_angle) = control_translation(Vec2 {x: -20.0, y: 0.0}, mass, thrust);
        assert_eq!(ctr_thrust, 20_000.0);

        let (ctr_thrust, ctr_angle) = control_translation(Vec2 {x: -30.0, y: 0.0}, mass, thrust);
        assert_eq!(ctr_thrust, 20_000.0);

        // +y

        let (ctr_thrust, ctr_angle) = control_translation(Vec2 {x: 0.0, y: 10.0}, mass, thrust);
        assert_eq!(ctr_thrust, 10_000.0);

        let (ctr_thrust, ctr_angle) = control_translation(Vec2 {x: 0.0, y: 20.0}, mass, thrust);
        assert_eq!(ctr_thrust, 20_000.0);

        let (ctr_thrust, ctr_angle) = control_translation(Vec2 {x: 0.0, y: 30.0}, mass, thrust);
        assert_eq!(ctr_thrust, 20_000.0);

        // -y -> gravity

        let (ctr_thrust, ctr_angle) = control_translation(Vec2 {x: 0.0, y: -10.0}, mass, thrust);
        assert_eq!(ctr_thrust, thrust);
        assert_eq!(ctr_angle, PI);

        let (ctr_thrust, ctr_angle) = control_translation(Vec2 {x: 0.0, y: -20.0}, mass, thrust);
        assert_eq!(ctr_thrust, thrust);
        assert_eq!(ctr_angle, PI);

        let (ctr_thrust, ctr_angle) = control_translation(Vec2 {x: 0.0, y: -30.0}, mass, thrust);
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
            let eng_gimbal_ref = control_angular(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
            assert_approx_eq!(eng_gimbal_ref, 0.25000079328132707*PI/180.0, 1e-6);

            // check linear (PID)

            {
                let ctr_angle = 2.0*10.0*PI/180.0;  // rad
                let eng_gimbal = control_angular(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
                assert_approx_eq!(eng_gimbal, 2.0*eng_gimbal_ref, 1e-6);
            }

            {
                let ctr_angle = 3.0*10.0*PI/180.0;  // rad
                let eng_gimbal = control_angular(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
                assert_approx_eq!(eng_gimbal, 3.0*eng_gimbal_ref, 1e-6);
            }

            // check ang vel max

            let max_eng_gimbal_vel = 1.0*PI/180.0;  // TODO conf

            {
                let ctr_angle = 3.0*10.0*PI/180.0;  // rad
                let eng_gimbal = control_angular(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
                assert_approx_eq!(eng_gimbal, 3.0*eng_gimbal_ref, 1e-6);
            }
            {
                let ctr_angle = 4.0*10.0*PI/180.0;  // rad
                let eng_gimbal = control_angular(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
                assert_approx_eq!(eng_gimbal, max_eng_gimbal_vel, 1e-6);
            }
            {
                let ctr_angle = 5.0*10.0*PI/180.0;  // rad
                let eng_gimbal = control_angular(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
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
            let eng_gimbal_ref = control_angular(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
            assert_approx_eq!(eng_gimbal_ref, 3.5*PI/180.0, 1e-4);

            // check linear (PID)

            let ctr_angle = 60.0*PI/180.0;  // rad
            let eng_gimbal = control_angular(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
            assert_approx_eq!(eng_gimbal, eng_gimbal_ref - 2.0*0.25*PI/180.0, 1e-4);

            let ctr_angle = 65.0*PI/180.0;  // rad
            let eng_gimbal = control_angular(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
            assert_approx_eq!(eng_gimbal, eng_gimbal_ref - 1.0*0.25*PI/180.0, 1e-4);

            let ctr_angle = 70.0*PI/180.0;  // rad
            let eng_gimbal = control_angular(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
            assert_approx_eq!(eng_gimbal, eng_gimbal_ref, 1e-6);

            let ctr_angle = 75.0*PI/180.0;  // rad
            let eng_gimbal = control_angular(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
            assert_approx_eq!(eng_gimbal, eng_gimbal_ref + 1.0*0.25*PI/180.0, 1e-4);

            let ctr_angle = 80.0*PI/180.0;  // rad
            let eng_gimbal = control_angular(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
            assert_approx_eq!(eng_gimbal, eng_gimbal_ref + 2.0*0.25*PI/180.0, 1e-4);

            // max

            let ctr_angle = 85.0*PI/180.0;  // rad
            let eng_gimbal = control_angular(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
            assert_approx_eq!(eng_gimbal, eng_gimbal_ref + 2.0*0.25*PI/180.0, 1e-4);

            let ctr_angle = 90.0*PI/180.0;  // rad
            let eng_gimbal = control_angular(ctr_angle, sc_mass, sc_thrust, sc_att_cur, eng_gimbal_cur);
            assert_approx_eq!(eng_gimbal, eng_gimbal_ref + 2.0*0.25*PI/180.0, 1e-4);
        }
    }

    #[test]
    fn test_gui_4_ctr() {
        // TODO test ctr()
    }
}
