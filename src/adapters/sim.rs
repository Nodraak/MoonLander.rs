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
        self.sim.read_sensors()
    }

    fn write_actuators(&mut self, control: ActuatorsValues) -> Result<(), &'static str> {
        self.sim.write_actuators(control)
    }
}
