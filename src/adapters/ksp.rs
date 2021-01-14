use std::io;
use pyo3::prelude::*;
use super::Adapter;


pub struct AdapterKSP<'py> {
    py: &'py Python<'py>,
    krpc: &'py PyModule,    // krpc module
    conn: &'py PyAny,       // krpc connection object
}


pub fn init<'py>(py: &'py Python) -> Result<AdapterKSP<'py>, &'static str> {
    match init_(py) {
        Err(e) => {
            // TODO: handle errors better
            println!("Python error:");
            e.print_and_set_sys_last_vars(*py);
            Err("Python error")
        },
        Ok(ksp) => {
            Ok(ksp)
        },
    }
}

fn init_<'py>(py: &'py Python) -> PyResult<AdapterKSP<'py>> {
    let krpc = py.import("krpc")?;
    let conn = krpc.call_method0("connect")?;

    let v = conn.getattr("krpc")?.call_method0("get_status")?.getattr("version")?;

    println!("krpc v: {}", v);

    Ok(AdapterKSP {
        py: py,
        krpc: krpc,
        conn: conn,
    })
}

impl Adapter for AdapterKSP<'_> {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        Ok(0)
    }

    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        Ok(0)
    }
}
