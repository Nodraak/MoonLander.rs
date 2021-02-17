use serde::{Serialize, Deserialize};
use uom::si::f64::*;
use uom::si::acceleration::meter_per_second_squared;
use uom::si::angular_acceleration::radian_per_second_squared;
use uom::si::angular_velocity::radian_per_second;
use uom::si::ratio::ratio;
use uom::si::time::second;
use uom::si::velocity::meter_per_second;

use crate::conf::Scenario;
use crate::utils::math::Vec2;


/// changing properties
#[derive(Debug)]
#[derive(Clone, Copy)]
#[derive(Serialize, Deserialize)]
pub struct SpacecraftDynamic {

    // nav miscs

    pub t: Time,                        // time since beginning of simulation
    pub dt: Time,

    pub fuel_mass: Mass,

    // nav acc

    pub acc_thrust: Acceleration,
    pub acc_atm: Acceleration,
    pub acc_gravity: Acceleration,
    pub acc_centrifugal: Acceleration,

    // nav trans

    pub pos: Vec2<Length>,
    pub vel: Vec2<Velocity>,
    pub acc: Vec2<Acceleration>,

    pub dv: Velocity,                   // dv expended until now

    // nav ang

    pub ang_pos: Angle,
    pub ang_vel: AngularVelocity,
    pub ang_acc: AngularAcceleration,

    // gui

    pub gui: Vec2<Acceleration>,

    // ctr

    pub eng_throttle: Ratio,            // range: [0; 1]
    pub eng_gimbal: Ratio,              // range: [-1; +1]
}


impl SpacecraftDynamic {
    pub fn new(conf: &Scenario) -> SpacecraftDynamic {
        SpacecraftDynamic {
            t: Time::new::<second>(0.0),
            dt: Time::new::<second>(1.0),

            eng_throttle: Ratio::new::<ratio>(0.0),
            fuel_mass: conf.initial_sc_fuel_mass,

            eng_gimbal: Ratio::new::<ratio>(0.0),

            acc_thrust: Acceleration::new::<meter_per_second_squared>(0.0),
            acc_gravity: Acceleration::new::<meter_per_second_squared>(0.0),
            acc_centrifugal: Acceleration::new::<meter_per_second_squared>(0.0),
            acc_atm: Acceleration::new::<meter_per_second_squared>(0.0),

            pos: conf.initial_sc_pos,
            vel: conf.initial_sc_vel,
            acc: Vec2 {
                x: Acceleration::new::<meter_per_second_squared>(0.0),
                y: Acceleration::new::<meter_per_second_squared>(0.0),
            },

            dv: Velocity::new::<meter_per_second>(0.0),

            ang_pos: conf.initial_sc_ang_pos,
            ang_vel: AngularVelocity::new::<radian_per_second>(0.0),
            ang_acc: AngularAcceleration::new::<radian_per_second_squared>(0.0),

            gui: Vec2 {
                x: Acceleration::new::<meter_per_second_squared>(0.0),
                y: Acceleration::new::<meter_per_second_squared>(0.0),
            },
        }
    }
}
