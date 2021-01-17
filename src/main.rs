mod adapters;
mod sim;

use std::process::exit;
use clap;
use pyo3::prelude::*;


fn land(adapter: &dyn adapters::common::Adapter) {
    loop {
        // TODO
        // adapter.read_sensors();
        // gnc::nav();
        // gnc::gui();
        // gnc::ctr();
        // adapter.write_sensors();
        // adapter.tick();

        // TODO break condition
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

            let adapter = adapters::sim::init().unwrap();  // TODO handle error

            land(&adapter);
        },
        ("ksp", submatches) => {
            println!("Subcommand: ksp");

            let gil = Python::acquire_gil();
            let py = gil.python();

            let adapter = adapters::ksp::init(&py).unwrap();  // TODO handle error

            land(&adapter);
        },
        _ => {
            println!("Error: expected SubCommand: sim|ksp");
            exit(1);
        }
    }
}
