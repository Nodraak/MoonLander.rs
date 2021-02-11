# README.md

*Nodraak's Space Program - Test Mission 1 - 150 m*

## Trajectory

a. Take off: reach x = 100 m, y = 150 m, in 50 seconds
-. Hover for 10 seconds
b. Land: reach x = 200 m, y = 0 m, in 50 seconds

## Spacecraft

Spec:

* G = 9.81 m/s**2
* 100 seconds burn (with some margin)
* First estimation: 10 t spacecraft, 10 t * 10 m/s**2 * 1.5 = 150 kN thrust

Detailed design, after simulation:

* Remote control unit - mass=0.100 t
* Landing legs LT-1 Landing Struts - mass=4 * 0.050 t
* Tank Rockomax X200-16 - 8+1 t (1600 units)
* Engine LV-T45 Swivel - mass=1.500 t - thrust=168 kN
    * Mass vs thrust: 11 t * 9.81 * 1.30 = 140 kN ; ok, less than 168
    * Fuel consumption: 1600/14 = 114 sec ; ok, more than 100

GNC configuration:

* Dry mass: 2 800 Kg
* Fuel mass: 8 000 Kg
