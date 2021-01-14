use std::io::prelude::*;
use std::io;
use std::net::TcpStream;
use super::Adapter;


pub struct AdapterSim {
    stream: TcpStream,
}


pub fn init() -> Result<AdapterSim, &'static str> {
    match init_() {
        Err(_) => {
            println!("Error adapter::sim::init()");
            Err("Error adapter::sim::init()")
        },
        Ok(sim) => {
            Ok(sim)
        }
    }
}

fn init_() -> std::io::Result<AdapterSim> {
    let stream = TcpStream::connect("127.0.0.1:34254")?;

    Ok(AdapterSim {
        stream: stream,
    })
}

impl Adapter for AdapterSim {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        Ok(0)
    }

    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        Ok(0)
    }
}
