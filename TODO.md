# TODO.md


print ksp pos, vel, acc
print sim pos, vel, acc

try project ksp to compute sensors acc, check if both prints match



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
            ```
                mass: 2%
                thurst 1%
                isp 0.5%
                thrust
                    noise 0.33 %
                    misalign 0 deg
                acc
                    noise 35 μg/√Hz
                    bias 0 ug
            ```
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
    * [ok] Guidance: tgo based guidance
    * [ok] uom
    * [ok] Control: PID for rotation control of the engine gimbal
    * KSP lander: how to design a rocket
    * Navigation: radar altimeter: Kalman / custom fusion / Apollo fusion
    * Rust gnc and/or Performances for real-life implementation


Pitch mismatch:
    1. Check sensor raw values equal sc nav values
    2. Feed ksp-like values from sim to sc nav to test


doc (https://github.com/iliekturtles/uom/issues/177)
add moment of inertia unit
