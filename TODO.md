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
        * Add noise in sensors meas - sensor spec:
            * https://www.nasa.gov/sites/default/files/atoms/files/soa2018_final_doc.pdf
            * https://www.colorado.edu/event/ippw2018/sites/default/files/attached-files/dltech_2_hormigo_presid501_presslides_docid1146.pdf
            * https://ntrs.nasa.gov/api/citations/20080033125/downloads/20080033125.pdf
        * Vel and pos from radar altimeter meas integration
    * Implement sim analysis loop: determine TGO, check fuel, ...
        * [ok] Estimate tgo
        * [wip] At the end: is landed, and compare sim vs spacecraft
        * [ok] During sim loop: check fuel and other constraints (angles)
    * [ok] Implement export to csv and Python plotter
    * [ok] Implement dt
    * [ok] Implement conf settings + load from yaml
    * [ok] Refacto gui to allow several guidance methods (tgo landing, tgo ascent, fixed ascent)
    * [ok] Spacecraft angle / Engine gimbal PID
    * [wip?] Rework sim to use 3D coordinates (simple PID to nullify the 3rd dimension?)
2. KSP
    * [ok] Rust I/O adapters prototypes (rust for Sim, Python for Krcp) -> Adapter trait
    * [ok] Setup KSP addons
    * [ok] Implement Krpc interfaces (read & write)
    * [ok] Error handling (Nan, div by zero, etc)
    * [wip] Test whole I/O chain (read & write)
3. Start Nodraak's space program on KSP
    * [ok] Update lander spec
    * Test launch (orbitial or suborbital, with or without landing)
    * Land on the Mun!
4. Write some blog articles
    * TODO blog:
        * Add disclaimer about Exomars and PTS at top of every Aerospace article
        * Rehost pdfs to prevent 404
    * [ok] Guidance: tgo based guidance
    * Control: double PID for rotation control from the engine gimbal
    * uom
    * KSP lander: how to design a rocket
    * Navigation: radar altimeter: Kalman / custom fusion / Apollo fusion
    * Rust gnc and/or Performances for real-life implementation
