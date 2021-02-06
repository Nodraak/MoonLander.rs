use std::f64::consts::PI;

use crate::adapters::common::{SensorsValues, ActuatorsValues};
use crate::conf::{Conf, Scenario};
use crate::utils::math::Vec2;
use crate::utils::spacecraft::SpacecraftDynamic;


pub struct Sim {
    dt: f64,
    conf: Scenario,                 // spacecraft configuration / static properties
    cur: SpacecraftDynamic,         // latest changing properties
    all: Vec<SpacecraftDynamic>,    // all changing properties
}


impl Sim {
    pub fn new(conf: Conf) -> Sim {
        Sim {
            dt: conf.dt_step,
            conf: conf.s,
            cur: SpacecraftDynamic::new(&conf.s),
            all: vec![],
        }
    }

    pub fn read_sensors(&self) -> Result<SensorsValues, &'static str> {
        Ok(SensorsValues {
            dt_step: self.dt,
            spacecraft_acc: self.cur.acc,
            spacecraft_ang_acc: self.cur.ang_acc,
            spacecraft_altitude: Some(self.cur.pos.y),
        })
    }

    pub fn write_actuators(&mut self, control: ActuatorsValues) -> Result<(), &'static str> {
        let dt = self.dt;
        let sc_mass = self.conf.sc_dry_mass + self.cur.fuel_mass;

        // compute t, mass

        let t = self.cur.t + dt;

        let sc_fuel_mass = self.cur.fuel_mass - self.conf.sc_nominal_mass_flow*control.engine_throttle*dt;

        // self.dv_flight += acc*dt

        // compute torque and angular vel/pos

        let sc_moment_of_inertia = 0.5 * sc_mass * (self.conf.sc_height/2.0).powi(2);  // 1/2*m*r**2 = kg.m**2
        let torque = (
            self.conf.sc_height/2.0
            * control.engine_throttle*self.conf.sc_nominal_thrust
            * (control.engine_gimbal*self.conf.ctr_eng_gimbal_pos_max).sin()
        );
        let sc_ang_acc = torque/sc_moment_of_inertia;

        let sc_ang_vel = self.cur.ang_vel + sc_ang_acc*dt;
        let sc_ang_pos = self.cur.ang_pos + sc_ang_vel*dt;

        // compute thrust and aerodynamic drags

        let engine_acc_norm = control.engine_throttle * self.conf.sc_nominal_thrust/sc_mass;
        let engine_acc = Vec2::new_polar(engine_acc_norm, self.cur.ang_pos);

        // dynamic pressure q: Pa = Kg/(m*s**2)
        // dynamic pressure n: N = Kg/(m*s**2) * m**2 = Kg*m/(s**2)
        let dp_q = 0.5 * self.conf.body.atmosphere_density(self.cur.pos.y) * self.cur.vel.norm().powi(2);
        let dp_n = dp_q * (PI*(self.conf.sc_width/2.0).powi(2)) * self.conf.sc_cd;
        let dp_drag = Vec2::new_polar(-dp_n/sc_mass, self.cur.ang_pos);

        // compute gravity/centrifugal

        let gravity_acc = Vec2::new_polar(-self.conf.body.gravity(self.cur.pos.y), PI/2.0);
        let centrifugal_acc = Vec2::new_polar(self.conf.body.centrifugal(self.cur.vel.x, self.cur.pos.y), PI/2.0);

        // compute acc/vel/pos

        let sc_acc = engine_acc + dp_drag + gravity_acc + centrifugal_acc;
        // self.g = self.acc_y/G0

        let sc_vel = self.cur.vel + sc_acc*dt;
        let sc_pos = self.cur.pos + sc_vel*dt;

        // save everything

        self.cur.t = t;

        self.cur.eng_throttle = control.engine_throttle;
        self.cur.fuel_mass = sc_fuel_mass;
        self.cur.eng_gimbal = control.engine_gimbal;

        self.cur.acc_thrust = engine_acc.norm();
        self.cur.acc_atm = -dp_drag.norm();
        self.cur.acc_gravity = -gravity_acc.norm();
        self.cur.acc_centrifugal = centrifugal_acc.norm();

        self.cur.pos = sc_pos;
        self.cur.vel = sc_vel;
        self.cur.acc = sc_acc;

        self.cur.ang_pos = sc_ang_pos;
        self.cur.ang_vel = sc_ang_vel;
        self.cur.ang_acc = sc_ang_acc;

        self.all.push(self.cur);

        println!("{:?}", self.cur);

        Ok(())
    }

    pub fn export_to_csv(&self, tgo: f64) {
        let mass = self.conf.sc_dry_mass + self.cur.fuel_mass;
        println!(
            "CSV SIM;{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};{:};",
            tgo,
            self.cur.eng_throttle,
            mass,
            self.cur.eng_gimbal*self.conf.ctr_eng_gimbal_pos_max,
            self.cur.acc_thrust,
            self.cur.acc_atm,
            self.cur.acc_gravity,
            self.cur.acc_centrifugal,
            self.cur.acc.x, self.cur.acc.y,
            self.cur.vel.x, self.cur.vel.y,
            self.cur.pos.x, self.cur.pos.y,
            self.cur.ang_acc,
            self.cur.ang_vel,
            self.cur.ang_pos,
        );
    }
}
