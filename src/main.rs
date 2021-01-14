use std::process::exit;
use clap;
use pyo3::prelude::*;

pub mod adapters;


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
            .about("starts a sim server")
        )
        .subcommand(
            clap::SubCommand::with_name("gnc-sim")
            .about("connects to a sim server")
        )
        .subcommand(
            clap::SubCommand::with_name("gnc-ksp")
            .about("connects to ksp")
        )
        .get_matches();

    // handle cli args

    let config = matches.value_of("config").unwrap_or("conf.json");
    println!("Config file: {}", config);

    match matches.subcommand() {
        ("sim", submatches) => {
            println!("Subcommand: sim");

            // TODO start sim server
        },
        ("gnc-sim", submatches) => {
            println!("Subcommand: gnc-sim");

            // TODO
            let _sim = adapters::sim::init();
            // TODO run gnc
        },
        ("gnc-ksp", submatches) => {
            println!("Subcommand: gnc-ksp");

            let gil = Python::acquire_gil();
            let py = gil.python();

            let _ksp = adapters::ksp::init(&py);
            // TOOD run gnc
        },
        _ => {
            println!("Error: expected SubCommand: sim|gnc-sim|gnc-ksp");
            exit(1);
        }
    }
}
