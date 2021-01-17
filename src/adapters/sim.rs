use crate::adapters::common::{Adapter, SensorsValues, ActuatorsValues};
use crate::sim::Sim;


pub struct AdapterSim {
    sim: Sim,
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

fn init_() -> Result<AdapterSim, &'static str> {
    let sim = Sim::new();

    Ok(AdapterSim {
        sim: sim,
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

    fn tick(&self) {
        self.sim.tick();
        // TODO: if real time: sleep X ms
    }
}
