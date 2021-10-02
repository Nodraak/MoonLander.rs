use pyo3::prelude::*;
use serde::{Serialize, Deserialize};
use uom::si::f64::*;
use uom::si::acceleration::meter_per_second_squared;
use uom::si::angle::{degree, radian};
use uom::si::angular_velocity::radian_per_second;
use uom::si::angular_acceleration::radian_per_second_squared;
use uom::si::ratio::ratio;
use uom::si::time::second;
use uom::si::velocity::meter_per_second;
use crate::adapters::common::{Adapter, SensorsValues, ActuatorsValues};
use crate::squared;
use crate::utils::math::Vec2;


pub struct AdapterKSP<'py> {
    conn: &'py PyAny,       // krpc connection object
    vessel: &'py PyAny,
    body_ref_frame: &'py PyAny,
    surface_ref_frame: &'py PyAny,

    last_met: Time,
    last_vel_vert: Velocity,
    last_vel_horiz: Velocity,
    last_ang_pos: Angle,                        // spacecraft's pitch, not velocity's
    last_ang_vel: AngularVelocity,
}


#[derive(Serialize, Deserialize)]
struct RawSensorsValues {
    met: Time,
    vel_vert: Velocity,
    vel_north: Velocity,
    vel_east: Velocity,
    ang_pos: Angle,
}


impl RawSensorsValues {
    fn export_to_csv(self) {
        println!("[LOGD:ksp::export_to_csv] CSV={}", serde_json::to_string(&self).unwrap());
    }
}


pub fn init<'py>(py: &'py Python) -> Result<AdapterKSP<'py>, &'static str> {
    match init_(py) {
        Err(e) => {
            // TODO: handle errors better
            println!("Python error:");
            e.print_and_set_sys_last_vars(*py);
            Err("Python error")
        },
        Ok(ksp) => {
            Ok(ksp)
        },
    }
}

fn init_<'py>(py: &'py Python) -> PyResult<AdapterKSP<'py>> {
    let krpc = py.import("krpc")?;
    let conn = krpc.call_method0("connect")?;

    let vessel = conn.getattr("space_center")?.getattr("active_vessel")?;
    let body_ref_frame = vessel.getattr("orbit")?.getattr("body")?.getattr("reference_frame")?;
    let surface_ref_frame = vessel.getattr("surface_reference_frame")?;

    Ok(AdapterKSP {
        conn: conn,
        vessel: vessel,
        body_ref_frame: body_ref_frame,
        surface_ref_frame: surface_ref_frame,

        last_met: Time::new::<second>(0.0),
        last_vel_vert: Velocity::new::<meter_per_second>(0.0),
        last_vel_horiz: Velocity::new::<meter_per_second>(0.0),
        last_ang_pos: Angle::new::<radian>(0.0),
        last_ang_vel: AngularVelocity::new::<radian_per_second>(0.0),
    })
}

impl Adapter for AdapterKSP<'_> {
    /// ## kRPC API and conventions:
    ///
    /// flight.pitch: The pitch of the vessel relative to the horizon, in
    ///     degrees. A value between -90° and +90°.
    ///
    /// flight.heading: The heading of the vessel (its angle relative to north),
    ///     in degrees. A value between 0° and 360°.
    ///
    /// Source: https://krpc.github.io/krpc/python/api/space-center/flight.html
    ///
    /// ## kRPC velocity bug
    ///
    /// Note canonical method shown below has a bug, using workaround.
    /// Ref: https://github.com/krpc/krpc/issues/454
    ///
    /// ex1_ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
    ///     position=vessel.orbit.body.reference_frame,
    ///     rotation=vessel.surface_reference_frame,
    /// )
    /// velocity = vessel.flight(ex1_ref_frame).velocity
    fn read_sensors(&mut self) -> SensorsValues {

        // Get raw data

        let met = Time::new::<second>(self.vessel
            .getattr("met").unwrap()
            .extract().unwrap());

        /*
            vel.heading
            vel.pitch
            flight.heading
            flight.pitch
        */

        let vel_ = self.vessel
            .call_method1("flight", (self.body_ref_frame, )).unwrap()
            .getattr("velocity").unwrap();
        let vels: (f64, f64, f64) = self.conn
            .getattr("space_center").unwrap()
            .call_method1("transform_direction", (vel_, self.body_ref_frame, self.surface_ref_frame)).unwrap()
            .extract().unwrap();
        let vel_vert = Velocity::new::<meter_per_second>(vels.0);
        let vel_north = Velocity::new::<meter_per_second>(vels.1);
        let vel_east = Velocity::new::<meter_per_second>(vels.2);

        let vel_horiz = (squared!(vel_north) + squared!(vel_east)).sqrt();

        let ang_pos = Angle::new::<degree>(self.vessel
            .call_method1("flight", (self.surface_ref_frame, )).unwrap()
            .getattr("pitch").unwrap()
            .extract().unwrap());

        let rsv = RawSensorsValues {
            met: met,
            vel_vert: vel_vert,
            vel_north: vel_north,
            vel_east: vel_east,
            ang_pos: ang_pos,
        };

        rsv.export_to_csv();

        // Compute sensor data

        let mut dt: Time = met - self.last_met;
        let mut acc_x: Acceleration = (vel_horiz-self.last_vel_horiz)/dt;
        let mut acc_y: Acceleration = (vel_vert-self.last_vel_vert)/dt;
        let mut ang_vel: AngularVelocity = ((ang_pos-self.last_ang_pos)/dt).into();
        let mut ang_acc: AngularAcceleration = ((ang_vel-self.last_ang_vel)/dt).into();

        if (dt.abs() < Time::new::<second>(0.001)) || (dt.abs() > Time::new::<second>(1.000)) {
            dt = Time::new::<second>(1.0);
            acc_x = Acceleration::new::<meter_per_second_squared>(0.0);
            acc_y = Acceleration::new::<meter_per_second_squared>(0.0);
            ang_vel = AngularVelocity::new::<radian_per_second>(0.0);
            ang_acc = AngularAcceleration::new::<radian_per_second_squared>(0.0);
        }

        // Update internal state and return

        self.last_met = met;
        self.last_vel_vert = vel_vert;
        self.last_vel_horiz = vel_horiz;
        self.last_ang_pos = ang_pos;
        self.last_ang_vel = ang_vel;

        SensorsValues {
            dt_step: dt,
            spacecraft_acc: Vec2 {x: acc_x, y: acc_y},
            spacecraft_ang_acc: ang_acc,
            spacecraft_altitude: None,  // TODO
        }
    }

    /// ## kRPC API and conventions:
    ///
    /// control.throttle: The state of the throttle. A value between 0 and 1.
    ///
    /// control.pitch: The state of the pitch control. A value between -1 and 1.
    ///     Equivalent to the w and s keys.
    ///
    /// control.yaw: The state of the yaw control. A value between -1 and 1.
    ///     Equivalent to the a and d keys.
    ///
    /// Source: https://krpc.github.io/krpc/python/api/space-center/control.html
    fn write_actuators(&mut self, control: ActuatorsValues) {
        let ves_control = self.vessel.getattr("control").unwrap();

        ves_control.setattr("throttle", control.engine_throttle.get::<ratio>()).unwrap();
        ves_control.setattr("pitch", -control.engine_gimbal.get::<ratio>()).unwrap();

        /*
            pitch
            yaw
            (roll)
        */
    }

    fn export_to_csv_conf(&self) {
        // Not implemented
    }

    fn export_to_csv_cur(&self) {
        // Not implemented
    }
}
