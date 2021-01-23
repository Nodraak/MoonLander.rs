# TODO.md

0. Basic
    * [ok] Rust Hello world
1. Sim
    * [ok] Reimplement python prototype
    * [ok] Add spacecraft angle (torque, inertia, ...)
    * [ok] Write some tests to verify the algorithms
    * Add fancy nav
        * [ok] Vel and pos from acc integration
        * [wip] Mass from acc or from engine throttle
        * Vel and pos from radar altimeter meas integration
    * Implement sim analysis loop: determine TGO, check fuel, ...
        * [wip] Estimate tgo
        * [wip] At the end: is landed, and compare sim vs spacecraft
        * During sim loop: check fuel and other constraints
    * [wip] Implement export to csv and Python plotter
    * Implement conf settings
2. KSP
    * [ok] Rust I/O adapters prototypes (rust for Sim, Python for Krcp) -> Adapter trait
    * [ok?] Setup KSP addons
    * Implement Krpc interfaces (read & write)
    * Test whole I/O chain (read & write)
3. Update lander spec
4. Start Nodraak's space program on KSP
    * Test launch (orbitial or suborbital, with or without landing)
    * Land on the Mun!
5. Write some blog articles
