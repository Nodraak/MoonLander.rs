use std::f64::consts::PI;

use uom::si::f64::*;
use uom::si::acceleration::meter_per_second_squared;
use uom::si::ratio::ratio;
use uom::si::velocity::meter_per_second;

use crate::{mul, norm, sqrt, squared};
use crate::adapters::common::{SensorsValues, ActuatorsValues};
use crate::conf::{Conf, Scenario};
use crate::utils::math::Vec2;
use crate::utils::spacecraft::SpacecraftDynamic;


pub struct Sim {
    dt: Time,
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

        let sc_moment_of_inertia = 0.5 * sc_mass * squared!(self.conf.sc_height/2.0);  // 1/2*m*r**2 = kg.m**2
        let alpha: Angle = (control.engine_gimbal*self.conf.ctr_eng_gimbal_pos_max).into();
        let torque = (
            self.conf.sc_height/2.0
            * control.engine_throttle*self.conf.sc_nominal_thrust
            * alpha.sin()
        );
        let sc_ang_acc: AngularAcceleration = (torque/sc_moment_of_inertia).into();

        let dav: AngularVelocity = (sc_ang_acc*dt).into();
        let sc_ang_vel = self.cur.ang_vel + dav;
        let dap: Angle = (sc_ang_vel*dt).into();
        let sc_ang_pos = self.cur.ang_pos + dap;

        // compute thrust and aerodynamic drags

        let ang_vunit = Vec2 {
            x: Ratio::new::<ratio>(sc_ang_pos.cos().value),
            y: Ratio::new::<ratio>(sc_ang_pos.sin().value),
        };

        let engine_acc_norm = control.engine_throttle * self.conf.sc_nominal_thrust/sc_mass;
        let engine_acc = mul!(ang_vunit, engine_acc_norm);

        // dynamic pressure q: Pa = Kg/(m*s**2)
        // dynamic pressure n: N = Kg/(m*s**2) * m**2 = Kg*m/(s**2)
        let vel: Velocity = Velocity::new::<meter_per_second>(norm!(self.cur.vel));
        let dp_q: Pressure = 0.5 * self.conf.body.atmosphere_density(self.cur.pos.y) * squared!(vel);
        let dp_n: Force = dp_q * (PI*squared!(self.conf.sc_width/2.0)) * self.conf.sc_cd;
        let dp_drag = mul!(ang_vunit, -dp_n/sc_mass);

        // compute gravity/centrifugal

        let vertical_vunit = Vec2 {
            x: Ratio::new::<ratio>(0.0),
            y: Ratio::new::<ratio>(1.0),
        };

        let gravity_acc = mul!(vertical_vunit, -self.conf.body.gravity(self.cur.pos.y));
        let centrifugal_acc = mul!(vertical_vunit, self.conf.body.centrifugal(self.cur.vel.x, self.cur.pos.y));

        // compute acc/vel/pos

        let sc_acc: Vec2<Acceleration> = engine_acc + dp_drag + gravity_acc + centrifugal_acc;
        // self.g = self.acc_y/G0

        let sc_vel = self.cur.vel + mul!(sc_acc, dt);
        let sc_pos = self.cur.pos + mul!(sc_vel, dt);

        // save everything

        self.cur.t = t;
        self.cur.dt = dt;

        self.cur.eng_throttle = control.engine_throttle;
        self.cur.fuel_mass = sc_fuel_mass;
        self.cur.eng_gimbal = control.engine_gimbal;

        self.cur.acc_thrust = Acceleration::new::<meter_per_second_squared>(norm!(engine_acc));
        self.cur.acc_atm = -Acceleration::new::<meter_per_second_squared>(norm!(dp_drag));
        self.cur.acc_gravity = -Acceleration::new::<meter_per_second_squared>(norm!(gravity_acc));
        self.cur.acc_centrifugal = Acceleration::new::<meter_per_second_squared>(norm!(centrifugal_acc));

        self.cur.pos = sc_pos;
        self.cur.vel = sc_vel;
        self.cur.acc = sc_acc;

        self.cur.ang_pos = sc_ang_pos;
        self.cur.ang_vel = sc_ang_vel;
        self.cur.ang_acc = sc_ang_acc;

        self.all.push(self.cur);

        Ok(())
    }

    pub fn export_to_csv_conf(&self) {
        println!("[LOGD:Sim::export_to_csv_conf] CSV={}", serde_json::to_string(&self.conf).unwrap());
    }

    pub fn export_to_csv_cur(&self) {
        println!("[LOGD:Sim::export_to_csv_cur] CSV={}", serde_json::to_string(&self.cur).unwrap());
    }
}
