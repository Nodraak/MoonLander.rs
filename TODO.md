# TODO.md

0. Basic
    * [ok] Rust Hello world
1. Sim
    * [ok] Reimplement python prototype
    * [ok] Add spacecraft angle (torque, inertia, ...)
    * [ok] Write some tests to verify the algorithms
    * Add fancy nav
        * [ok] Vel and pos from acc integration
        * [ok] Mass from acc or from engine throttle
        * Add noise in sensors meas
        * Vel and pos from radar altimeter meas integration
    * Implement sim analysis loop: determine TGO, check fuel, ...
        * [ok] Estimate tgo
        * [wip] At the end: is landed, and compare sim vs spacecraft
        * During sim loop: check fuel and other constraints
    * [ok] Implement export to csv and Python plotter
    * [wip] Implement dt
    * [wip] Implement conf settings
2. KSP
    * [ok] Rust I/O adapters prototypes (rust for Sim, Python for Krcp) -> Adapter trait
    * [ok] Setup KSP addons
    * [ok] Implement Krpc interfaces (read & write)
    * [wip] Test whole I/O chain (read & write)
3. Start Nodraak's space program on KSP
    * Update lander spec
    * Test launch (orbitial or suborbital, with or without landing)
    * Land on the Mun!
4. Write some blog articles
    * TODO blog:
        * Add disclaimer about Exomars and PTS at top of every Aerospace article
        * Rehost pdfs to prevent 404
    * Double PID for rotation control from the engine gimbal
    * KSP lander
    * Rust gnc and/or Performances for real-life implementation
