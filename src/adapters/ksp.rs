use pyo3::prelude::*;

pub struct KSP<'py> {
    py: &'py Python<'py>,
    krpc: &'py PyModule,    // krpc module
    conn: &'py PyAny,       // krpc connection object
}


pub fn init<'py>(py: &'py Python) -> Result<KSP<'py>, &'static str> {
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

fn init_<'py>(py: &'py Python) -> PyResult<KSP<'py>> {
    let krpc = py.import("krpc")?;
    let conn = krpc.call_method0("connect")?;

    let v = conn.getattr("krpc")?.call_method0("get_status")?.getattr("version")?;

    println!("krpc v: {}", v);

    Ok(KSP {
        py: py,
        krpc: krpc,
        conn: conn,
    })
}
