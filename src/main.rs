// When they are multi-lines, I like to enclose my if conditions to differentiate with the body
#![allow(unused_parens)]

mod adapters;
mod conf;
mod gnc;
mod sim;
mod utils;

use std::process::exit;
use std::{thread, time};

#[macro_use]
extern crate assert_float_eq;

use clap;
use pyo3::prelude::*;
use serde_json;
use uom::si::f64::*;
use uom::si::time::second;
use uom::si::velocity::meter_per_second;

use crate::conf::{Scenario, Conf, TgoEstimate, SubCommand};
use crate::gnc::common::Spacecraft;
use crate::utils::space::{tgo_estimate, has_softly_landed};


fn land(adapter: &mut dyn adapters::common::Adapter, conf: Conf) {
    let mut sc = Spacecraft::new(conf);

    let mut tgo: Time = match conf.s.tgo_method {
        TgoEstimate::TgoGivenFixed => {
            conf.s.tgo_init
        },
        TgoEstimate::TgoEstimateFixed | TgoEstimate::TgoEstimateUpdating => {
            tgo_estimate(&sc, sc.conf.s.gui_vf_x, sc.conf.s.gui_vf_y, sc.conf.s.tgo_thrust_mul)
        }
    };

    sc.export_to_csv_conf();
    adapter.export_to_csv_conf();

    loop {
        if conf.s.tgo_method == TgoEstimate::TgoEstimateUpdating {
            tgo = tgo_estimate(&sc, sc.conf.s.gui_vf_x, sc.conf.s.gui_vf_y, sc.conf.s.tgo_thrust_mul);
        }

        println!("[LOGD:land] tgo={:.3}", tgo.get::<second>());

        // inputs, gnc, outputs

        let sensors_vals = adapter.read_sensors();
        println!("[LOGD:land] SensorsValues={}", serde_json::to_string(&sensors_vals).unwrap());

        gnc::navigation::nav(&mut sc, &sensors_vals);
        gnc::guidance::gui(&mut sc, tgo);
        let actuators_vals = gnc::control::ctr(&mut sc);

        println!("[LOGD:land] ActuatorsValues={}", serde_json::to_string(&actuators_vals).unwrap());
        adapter.write_actuators(actuators_vals);

        // quick check internal state

        sc.check_state();

        // export

        sc.export_to_csv_cur();
        adapter.export_to_csv_cur();

        // Stop the loop a few seconds before touchdown, to prevent guidance from diverging to +/- inf

        if tgo < conf.s.tgo_stop {
            break;
        }

        // time

        tgo -= sensors_vals.dt_step;
        thread::sleep(time::Duration::from_secs_f64(conf.dt_sleep.get::<second>()));
    }

    if has_softly_landed(&sc) {
        println!("Landing is SUCCESSFUL");
    } else {
        println!("Landing is FAILED");
    }

    println!("spacecraft.dv: {:.1} m/s", sc.cur.dv.get::<meter_per_second>());
}


fn main() {
    // configure CLI args

    let matches = clap::App::new("Moon Lander")
        .arg(
            clap::Arg::with_name("config")
            .short("c")
            .long("config")
            .help("Specify a config file")
            .default_value("conf/default.yaml")
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

    let scenario = Scenario::load(matches.value_of("config").unwrap());

    match matches.subcommand() {
        ("sim", _submatches) => {
            println!("Subcommand: sim");

            let dt_step = Time::new::<second>(0.100);

            let conf = Conf::new(SubCommand::Sim, dt_step, Time::new::<second>(0.0), scenario);

            let mut adapter = adapters::sim::init(conf).unwrap();  // TODO handle error

            land(&mut adapter, conf);
        },
        ("ksp", _submatches) => {
            println!("Subcommand: ksp");

            let dt_sleep = Time::new::<second>(0.100);

            let gil = Python::acquire_gil();
            let py = gil.python();

            let conf = Conf::new(SubCommand::Ksp, Time::new::<second>(0.0), dt_sleep, scenario);

            let mut adapter = adapters::ksp::init(&py).unwrap();  // TODO handle error

            land(&mut adapter, conf);
        },
        _ => {
            println!("Error: expected SubCommand: sim|ksp");
            exit(1);
        }
    }
}
