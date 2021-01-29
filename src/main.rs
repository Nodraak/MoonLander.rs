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


fn land(adapter: &mut dyn adapters::common::Adapter, dt_step: f64, dt_sleep: f64) {
    let final_vel_x_goal = 0.0;  // TODO conf
    let final_vel_y_goal = 0.0;
    let thrust_mul = 0.80;  // compute_tgo() assumes a 100% thrust, but on average you want the engine to run at X %

    let mut sc = Spacecraft::new();

    let mut tgo = tgo_estimate(&sc, final_vel_x_goal, final_vel_y_goal, thrust_mul);

    // Note: stop the loop a few seconds before touchdown, to prevent guidance from diverging to +/- inf
    while tgo > 2.0 {

        // inputs, gnc, outputs

        let sensors_vals = adapter.read_sensors().unwrap();

        gnc::navigation::nav(&mut sc, sensors_vals);
        let gui_out = gnc::guidance::gui(&sc, tgo);
        let actuators_vals = gnc::control::ctr(&mut sc, gui_out);

        adapter.write_actuators(actuators_vals);

        // export

        if ((tgo as u64) % 50) == 0 {
            sc.export_to_csv_header();
            adapter.export_to_csv_header();
        }
        sc.export_to_csv(tgo);
        adapter.export_to_csv(tgo);

        // time

        tgo -= dt_step;
        thread::sleep(time::Duration::from_millis((dt_sleep*1000.0) as u64));
    }

    if has_softly_landed(&sc) {
        println!("Landing is SUCCESSFUL");
    } else {
        println!("Landing is FAILED");
    }
}


fn main() {
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

            let dt_step = 0.100;
            let dt_sleep = 0.0;

            let mut adapter = adapters::sim::init(dt_step).unwrap();  // TODO handle error

            land(&mut adapter, dt_step, dt_sleep);
        },
        ("ksp", submatches) => {
            println!("Subcommand: ksp");

            let dt_step = 0.100;
            let dt_sleep = 0.500;

            let gil = Python::acquire_gil();
            let py = gil.python();

            let mut adapter = adapters::ksp::init(&py).unwrap();  // TODO handle error

            land(&mut adapter, dt_step, dt_sleep);
        },
        _ => {
            println!("Error: expected SubCommand: sim|ksp");
            exit(1);
        }
    }
}
