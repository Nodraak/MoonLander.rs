mod adapters;
mod gnc;
mod sim;
mod spacecraft;
mod utils;

use std::process::exit;
use clap;
use pyo3::prelude::*;
use crate::gnc::common::Spacecraft;


// TODO check rust enum
#[derive(PartialEq)]
enum Mode {
    SimFastest,
    SimRealTime,
    Ksp,
}


fn land(mode: Mode, adapter: &mut dyn adapters::common::Adapter) {
    let mut sc = Spacecraft::new();

    for tgo_ in 0..500 {
        let tgo = 500-tgo_;

        let sensors_vals = adapter.read_sensors().unwrap();

        gnc::navigation::nav(&mut sc, sensors_vals);
        let gui_out = gnc::guidance::gui(&sc, tgo as f64);
        let actuators_vals = gnc::control::ctr(&mut sc, gui_out);

        adapter.write_actuators(actuators_vals);

        // TODO break condition

        if mode == Mode::SimRealTime || mode == Mode::Ksp {
            // TODO sleep X ms
        }
    }
}


fn main() {
    println!("Hello, world!");

    // configure CLI args

    let matches = clap::App::new("Moon Lander")
        .arg(
            clap::Arg::with_name("config")
            .short("c")
            .long("config")
            .help("Specify a config file")
            .takes_value(true)
        )
        .subcommand(
            clap::SubCommand::with_name("sim")
            .about("runs a simulation")
        )
        .subcommand(
            clap::SubCommand::with_name("ksp")
            .about("connects to ksp")
        )
        .get_matches();

    // handle cli args

    let config = matches.value_of("config").unwrap_or("conf.json");
    println!("Config file: {}", config);

    match matches.subcommand() {
        ("sim", submatches) => {
            println!("Subcommand: sim");

            let mut adapter = adapters::sim::init().unwrap();  // TODO handle error

            land(Mode::SimFastest, &mut adapter);
        },
        ("ksp", submatches) => {
            println!("Subcommand: ksp");

            let gil = Python::acquire_gil();
            let py = gil.python();

            let mut adapter = adapters::ksp::init(&py).unwrap();  // TODO handle error

            land(Mode::Ksp, &mut adapter);
        },
        _ => {
            println!("Error: expected SubCommand: sim|ksp");
            exit(1);
        }
    }
}
