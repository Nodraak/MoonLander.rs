*Note: work in progress. Refer to [TODO.md](TODO.md) for the roadmap.*

# README.md

(Moon) lander guidance software.

The goal is to land on [KSP's Mun](https://wiki.kerbalspaceprogram.com/wiki/Mun).
This repo comes with a simulator to verify the guidance before attempting a landing.

## Architecture

**Overview**

The plan (From https://blog.nodraak.fr/2020/08/aerospace-sim-1-preliminary-study/):

![](docs/images/Architecture.svg)

**Implementation**

GNC is implemented in Rust.

I/O will be implemented via adapters (to be able to swap between the simulator and KSP):

* Simulator: direct (Rust) function calls
* KSP: calling Python from Rust (https://github.com/PyO3/pyo3), itself calling Krpc lib (protobuf to KSP) (https://krpc.github.io/krpc/)

## Building and running

**Dependencies**

* Rust
* For KSP subcommand
    * KSP game
    * Krpc addon: `sudo pip3 install krpc`

**Building**

`cargo build`

**Running**

Sim:

1. `cargo run -- sim`

KSP:

1. Start KSP and Krpc
2. `cargo run -- ksp`

**Plotting**

You can pipe moon_lander's stdout to plotter.py: `cargo run -- sim | python3 plotter.py`.
The graphs will also be saved as `output.png`.
