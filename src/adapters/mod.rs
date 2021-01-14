use std::io;

pub trait Adapter {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize>;
    fn write(&mut self, buf: &[u8]) -> io::Result<usize>;
}

pub mod ksp;
pub mod sim;
