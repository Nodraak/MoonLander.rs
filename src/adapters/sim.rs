use std::net::TcpStream;
use crate::adapters::common::{Adapter, SensorsValues, ActuatorsValues};


pub struct AdapterSim {
    stream: TcpStream,
}


pub fn init() -> Result<AdapterSim, &'static str> {
    match init_() {
        Err(_) => {
            println!("Error adapter::sim::init()");
            Err("Error adapter::sim::init()")
        },
        Ok(sim) => {
            Ok(sim)
        }
    }
}

fn init_() -> std::io::Result<AdapterSim> {
    let stream = TcpStream::connect("127.0.0.1:34254")?;

    Ok(AdapterSim {
        stream: stream,
    })
}

impl Adapter for AdapterSim {
    fn read_sensors(&self) -> Result<SensorsValues, &'static str> {
        Ok(SensorsValues {
            spacecraft_acc: (1.0, 1.0),
            spacecraft_altitude: Some(1.0),
        })
    }

    fn write_actuators(&self, values: ActuatorsValues) -> Result<(), &'static str> {
        Ok(())
    }
}
