# TODO.md

0. Basic
    * [ok] Rust Hello world
1. Sim
    * [ok] Reimplement python prototype
    * [ok] Add spacecraft angle (torque, inertia, ...)
    * [wip] Write some tests to verify the algorithms
    * Add fancy nav (vel and pos)
        * From acc integration
        * From radar altimeter meas integration
        * Mass estimation (from acc, from engine throttle)
    * Implement sim analysis loop: determine TGO, check fuel, ...
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
