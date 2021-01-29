use crate::adapters::common::{Adapter, SensorsValues, ActuatorsValues};
use crate::sim::Sim;


pub struct AdapterSim {
    sim: Sim,
}


pub fn init(dt_step: f64) -> Result<AdapterSim, &'static str> {
    match init_(dt_step) {
        Err(_) => {
            println!("Error adapter::sim::init()");
            Err("Error adapter::sim::init()")
        },
        Ok(sim) => {
            Ok(sim)
        }
    }
}

fn init_(dt_step: f64) -> Result<AdapterSim, &'static str> {
    let sim = Sim::new(dt_step);

    Ok(AdapterSim {
        sim: sim,
    })
}

impl Adapter for AdapterSim {
    fn read_sensors(&mut self) -> Result<SensorsValues, &'static str> {
        self.sim.read_sensors()
    }

    fn write_actuators(&mut self, control: ActuatorsValues) -> Result<(), &'static str> {
        self.sim.write_actuators(control)
    }

    fn export_to_csv_header(&self) {
        self.sim.export_to_csv_header();
    }

    fn export_to_csv(&self, tgo: f64) {
        self.sim.export_to_csv(tgo);
    }
}
