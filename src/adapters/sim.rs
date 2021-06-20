use crate::adapters::common::{Adapter, SensorsValues, ActuatorsValues};
use crate::conf::Conf;
use crate::sim::Sim;


pub struct AdapterSim {
    sim: Sim,
}


pub fn init(conf: Conf) -> Result<AdapterSim, &'static str> {
    match init_(conf) {
        Err(_) => {
            println!("Error adapter::sim::init()");
            Err("Error adapter::sim::init()")
        },
        Ok(sim) => {
            Ok(sim)
        }
    }
}

fn init_(conf: Conf) -> Result<AdapterSim, &'static str> {
    let sim = Sim::new(conf);

    Ok(AdapterSim {
        sim: sim,
    })
}

impl Adapter for AdapterSim {
    fn read_sensors(&mut self) -> SensorsValues {
        let mut ret = self.sim.read_sensors();

        // inject noise and bias

        ret
    }

    fn write_actuators(&mut self, control: ActuatorsValues) {
        let mut c = control;

        // inject noise and bias
        // c.engine_throttle *= 0.98;

        self.sim.write_actuators(c)
    }

    fn export_to_csv_conf(&self) {
        self.sim.export_to_csv_conf();
    }

    fn export_to_csv_cur(&self) {
        self.sim.export_to_csv_cur();
    }
}
