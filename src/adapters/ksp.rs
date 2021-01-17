use pyo3::prelude::*;
use crate::adapters::common::{Adapter, SensorsValues, ActuatorsValues};


pub struct AdapterKSP<'py> {
    py: &'py Python<'py>,
    krpc: &'py PyModule,    // krpc module
    conn: &'py PyAny,       // krpc connection object
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

    let v = conn.getattr("krpc")?.call_method0("get_status")?.getattr("version")?;

    println!("krpc v: {}", v);

    /*
    vessel = conn.space_center.active_vessel
    refframe = vessel.orbit.body.reference_frame

    stream_func = vessel.position
    stream_func_args = (refframe, )

    read_position = conn.add_stream(stream_func, *stream_func_args)
    */

    Ok(AdapterKSP {
        py: py,
        krpc: krpc,
        conn: conn,
    })
}

impl Adapter for AdapterKSP<'_> {
    fn read_sensors(&self) -> Result<SensorsValues, &'static str> {
        /*
        vessel
            .mass -> integration from mass_flow (thrust ratio and isp)
            .moment_of_inertia: 3-tuple
            -> v1: get exact value ; maybe v2 estimate/compute

        # https://krpc.github.io/krpc/python/api/space-center/flight.html#SpaceCenter.Flight
        vessel.flight(reference_frame=XX)
        -> altitude

        vessel.surface_reference_frame
        vessel.orbital_reference_frame
        vessel.surface_velocity_reference_frame

        vessel
            .position(reference_frame)
            .direction(reference_frame)
            .velocity(reference_frame)

        conn.space_center.transform_position

        */
        Ok(SensorsValues {
            spacecraft_acc: (1.0, 1.0),
            spacecraft_altitude: Some(1.0),
        })
    }

    fn write_actuators(&self, values: ActuatorsValues) -> Result<(), &'static str> {
        /*
        # https://krpc.github.io/krpc/python/api/space-center/control.html#SpaceCenter.Control
        vessel.control
            .throttle
            .pitch
            .yaw
            .roll
        */

        Ok(())
    }

    fn tick(&self) {
        // TODO: sleep X ms
    }
}
