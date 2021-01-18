mod adapters;
mod gnc;
mod sim;
mod utils;

use std::process::exit;
use clap;
use pyo3::prelude::*;
use crate::adapters::common::ActuatorsValues;


fn land(adapter: &mut dyn adapters::common::Adapter) {
    loop {
        // TODO
        let _ = adapter.read_sensors();
        // gnc::nav();
        gnc::guidance::gui();
        gnc::control::ctr();
        let _ = adapter.write_actuators(ActuatorsValues {engine_gimbal: 0.0, engine_throttle: 0.0});

        // TODO break condition

        // TODO: sleep X ms if needed
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

            land(&mut adapter);
        },
        ("ksp", submatches) => {
            println!("Subcommand: ksp");

            let gil = Python::acquire_gil();
            let py = gil.python();

            let mut adapter = adapters::ksp::init(&py).unwrap();  // TODO handle error

            land(&mut adapter);
        },
        _ => {
            println!("Error: expected SubCommand: sim|ksp");
            exit(1);
        }
    }
}
