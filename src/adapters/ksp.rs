use std::f64::consts::PI;
use pyo3::prelude::*;
use crate::adapters::common::{Adapter, SensorsValues, ActuatorsValues};
use crate::utils::math::Vec2;


pub struct AdapterKSP<'py> {
    conn: &'py PyAny,       // krpc connection object
    vessel: &'py PyAny,
    body_ref_frame: &'py PyAny,
    surface_ref_frame: &'py PyAny,

    last_met: f64,          // seconds
    last_vel_vert: f64,     // m/s
    last_vel_horiz: f64,    // m/s
    last_pitch: f64,        // rad - spacecraft's pitch, not velocity's
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

        last_met: 0.0,
        last_vel_vert: 0.0,
        last_vel_horiz: 0.0,
        last_pitch: 0.0,
    })
}

impl Adapter for AdapterKSP<'_> {
    fn read_sensors(&mut self) -> Result<SensorsValues, &'static str> {
        /*
            Note canonical method shown below has a bug, using workaround.
            Ref: https://github.com/krpc/krpc/issues/454

            ex1_ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
                position=vessel.orbit.body.reference_frame,
                rotation=vessel.surface_reference_frame,
            )
            velocity = vessel.flight(ex1_ref_frame).velocity
        */

        // Get raw data

        let met = self.vessel
            .getattr("met").unwrap()
            .extract().unwrap();

        let dt = met - self.last_met;

        // TODO if dt == 0 => game is paused, return None

        let vel_ = self.vessel
            .call_method1("flight", (self.body_ref_frame, )).unwrap()
            .getattr("velocity").unwrap();
        let (vel_vert, vel_north, vel_east): (f64, f64, f64) = self.conn
            .getattr("space_center").unwrap()
            .call_method1("transform_direction", (vel_, self.body_ref_frame, self.surface_ref_frame)).unwrap()
            .extract().unwrap();
        let vel_horiz = (vel_north.powi(2) + vel_east.powi(2)).sqrt();

        let pitch = self.vessel
            .call_method1("flight", (self.surface_ref_frame, )).unwrap()
            .getattr("pitch").unwrap()
            .extract().unwrap();

        // Compute sensor data

        let acc_x = (vel_horiz-self.last_vel_horiz)/dt;
        let acc_y = (vel_vert-self.last_vel_vert)/dt;
        let ang_acc = (pitch-self.last_pitch)/dt * PI/180.0;  // KSP pitch is in deg, convert to rad

        // Update internal state and return

        self.last_met = met;
        self.last_vel_vert = vel_vert;
        self.last_vel_horiz = vel_horiz;
        self.last_pitch = pitch;

        Ok(SensorsValues {
            dt_step: dt,
            spacecraft_acc: Vec2 {x: acc_x, y: acc_y},
            spacecraft_ang_acc: ang_acc,
            spacecraft_altitude: None,  // TODO
        })
    }

    fn write_actuators(&mut self, control: ActuatorsValues) -> Result<(), &'static str> {
        let ves_control = self.vessel.getattr("control").unwrap();

        ves_control.setattr("throttle", control.engine_throttle).unwrap();
        ves_control.setattr("pitch", control.engine_gimbal).unwrap();

        Ok(())
    }

    fn export_to_csv_header(&self) {
        // Not implemented
    }

    fn export_to_csv(&self, _tgo: f64) {
        // Not implemented
    }
}
