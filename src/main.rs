// When they are multi-lines, I like to enclose my if conditions to differentiate with the body
#![allow(unused_parens)]

mod adapters;
mod gnc;
mod sim;
mod spacecraft;
mod utils;

use std::process::exit;
use std::{thread, time};
use clap;
use pyo3::prelude::*;
use crate::gnc::common::Spacecraft;
use crate::utils::space::{tgo_estimate, has_softly_landed};


// TODO check rust enum
#[derive(PartialEq)]
enum Mode {
    SimFastest,
    SimRealTime,
    Ksp,
}


fn land(mode: Mode, adapter: &mut dyn adapters::common::Adapter) {
    let final_vel_x_goal = 0.0;  // TODO conf
    let final_vel_y_goal = 0.0;
    let thrust_mul = 0.90;  // compute_tgo() assumes a 100% thrust, but on average you want the engine to run at X %

    let mut sc = Spacecraft::new();

    let tgo_est = tgo_estimate(&sc, final_vel_x_goal, final_vel_y_goal, thrust_mul);

    // Note: stop the loop a few seconds before touchdown, to prevent guidance from diverging to +/- inf
    for tgo_ in 0..(tgo_est-3) {
        let tgo = tgo_est-tgo_;

        let sensors_vals = adapter.read_sensors().unwrap();

        gnc::navigation::nav(&mut sc, sensors_vals);
        let gui_out = gnc::guidance::gui(&sc, tgo as f64);
        let actuators_vals = gnc::control::ctr(&mut sc, gui_out);

        adapter.write_actuators(actuators_vals);

        if (tgo % 50) == 0 {
            sc.export_to_csv_header();

            if mode == Mode::SimFastest || mode == Mode::SimRealTime {
                adapter.export_to_csv_header();
            }
        }
        sc.export_to_csv(tgo);
        if mode == Mode::SimFastest || mode == Mode::SimRealTime {
            adapter.export_to_csv(tgo);
        }

        // TODO break condition

        if mode == Mode::SimRealTime || mode == Mode::Ksp {
            thread::sleep(time::Duration::from_millis(500));
        }
    }

    if has_softly_landed(&sc) {
        println!("Landing is SUCCESSFUL");
    } else {
        println!("Landing is FAILED");
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
