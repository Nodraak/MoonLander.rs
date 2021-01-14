
use pyo3::prelude::*;

pub mod adapters;


fn main() {
    println!("Hello, world!");

    let gil = Python::acquire_gil();
    let py = gil.python();
    let _ksp_adapter = adapters::ksp::init(&py);
}
