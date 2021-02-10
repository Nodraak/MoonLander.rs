*Note: work in progress. Refer to [TODO.md](TODO.md) for the roadmap.*

# README.md

(Moon) lander guidance software.

The goal is to land on [KSP's Mun](https://wiki.kerbalspaceprogram.com/wiki/Mun).
This repo comes with a simulator to verify the guidance before attempting a landing.

## Architecture

**Overview**

1. The plan (From https://blog.nodraak.fr/2020/08/aerospace-sim-1-preliminary-study/): ![](docs/images/Architecture.svg)
2. The first prototype: <https://blog.nodraak.fr/2020/12/aerospace-sim-2-guidance-law/>
3. The final implementation: this repo

**Implementation**

GNC is implemented in Rust.

I/O will be implemented via adapters (to be able to swap between the simulator and KSP):

* Simulator: direct (Rust) function calls
* KSP: calling Python from Rust (https://github.com/PyO3/pyo3), itself calling Krpc lib (protobuf to KSP) (https://krpc.github.io/krpc/)

## Building and running

**Dependencies**

* Rust (and a few crates)
* For the `ksp` subcommand:
    * KSP game
    * Krpc addon: `sudo pip3 install krpc`

**Building**

`cargo build`

**Running**

Sim:

1. `cargo run -- -c conf/Apollo-descent.yaml sim`

KSP:

1. Start KSP and Krpc
2. `cargo run -- -c conf/Apollo-descent.yaml ksp`

**Plotting**

You can pipe moon_lander's stdout to plotter.py: `cargo run -- -c conf/Apollo-descent.yaml sim | py plotter.py `.
It will run MoonLander, then show some curves.
The graphs will also be saved as `output.png`.

Example:

![](docs/images/plot_ctr_ang.png)
![](docs/images/plot_nav.png)

## Key concepts needed for landing on the Moon

* GNC
* TGO-based guidance (PID with a predesigned trajectory is flaky)
* Take into account moon_gravity and moon_centrifugal (guidance)
* Double cascade PID for angular control of the spacecraft with the engine
