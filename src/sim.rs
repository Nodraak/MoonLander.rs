use crate::adapters::common::{SensorsValues, ActuatorsValues};
use crate::utils::math::Vec2;
use crate::utils::space::{moon_centrifugal, moon_gravity};


#[derive(Clone, Copy)]
pub struct SimValues {
    t: f64,                         // unit: sec - time since beginning of simulation

    // changing properties
    sc_fuel_mass: f64,              // unit: kg
    sc_pos: Vec2,                   // unit: m
    sc_vel: Vec2,                   // unit: m/s
    sc_acc: Vec2,                   // unit: m/s**2
    sc_ang_pos: f64,                // unit: rad
    sc_ang_vel: f64,                // unit: rad/s

    // control
    engine_gimbal: f64,             // unit: rad
    engine_throttle: f64,           // unit: 0-1
}


pub struct Sim {
    // fixed properties
    engine_nominal_thrust: f64,         // unit: N
    engine_nominal_mass_flow: f64,      // unit: kg/s
    sc_dry_mass: f64,                   // unit: kg

    // latest changing properties
    cur: SimValues,

    // all changing properties
    all: Vec<SimValues>,
}


impl Sim {
    pub fn new() -> Sim {
        Sim {
            engine_nominal_thrust: 0.0,
            engine_nominal_mass_flow: 1.0, // TODO from conf
            sc_dry_mass: 0.0,  // TODO from conf

            cur: SimValues {
                t: 0.0,

                sc_fuel_mass: 0.0,  // TODO from conf
                sc_pos: Vec2 {x: 0.0, y: 0.0},
                sc_vel: Vec2 {x: 0.0, y: 0.0},
                sc_acc: Vec2 {x: 0.0, y: 0.0},
                sc_ang_pos: 0.0,
                sc_ang_vel: 0.0,

                engine_gimbal: 0.0,
                engine_throttle: 0.0,
            },

            all: vec![],
        }
    }

    pub fn read_sensors(&self) -> Result<SensorsValues, &'static str> {
        Ok(SensorsValues {
            spacecraft_acc: self.cur.sc_acc,
            spacecraft_altitude: Some(self.cur.sc_pos.y),
        })
    }

    pub fn write_actuators(&mut self, control: ActuatorsValues) -> Result<(), &'static str> {
        let dt = 1.0;

        // compute everything

        // TODO first compute torque and update sc_angle, then compute sc_acc

        let sc_mass = self.sc_dry_mass + self.cur.sc_fuel_mass;

        let engine_acc_norm = control.engine_throttle * self.engine_nominal_thrust/sc_mass;
        let engine_acc = Vec2::new_polar(engine_acc_norm, self.cur.sc_ang_pos);
        let gravity_acc = Vec2 {
            x: 0.0,
            y:
                - moon_gravity(self.cur.sc_pos.y)
                + moon_centrifugal(self.cur.sc_vel.x, self.cur.sc_pos.y),
        };
        let sc_acc = engine_acc + gravity_acc;
        // self.g = self.acc_y/G0

        let sc_vel = self.cur.sc_vel + sc_acc*dt;
        let sc_pos = self.cur.sc_pos + sc_vel*dt;

        let sc_fuel_mass = self.cur.sc_fuel_mass - self.engine_nominal_mass_flow*control.engine_throttle*dt;

        // self.dv_flight += acc*dt

        let t = self.cur.t + dt;

        // save everything

        self.cur.t = t;

        self.cur.sc_fuel_mass = sc_fuel_mass;
        self.cur.sc_pos = sc_pos;
        self.cur.sc_vel = sc_vel;
        self.cur.sc_acc = sc_acc;
        // self.cur.sc_ang_pos = sc_ang_pos; TODO
        // self.cur.sc_ang_vel = sc_ang_vel; TODO

        self.cur.engine_gimbal = control.engine_gimbal;
        self.cur.engine_throttle = control.engine_throttle;

        self.all.push(self.cur);

        Ok(())
    }
}
