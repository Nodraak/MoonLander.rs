
/// Main control function
pub fn control() {
    spacecraft_controler();
    engine_controler();
}

/// Spacecraft control function
///
/// Input:
///     goal_acc_x, goal_acc_y
///     spacecraft mass
///     engine max_thrust
/// Output:
///     commanded thrust (respecting engine constraints)
///     commanded (ideal) (spacecraft) attitude angle
fn spacecraft_controler() {
    /*
    TODO

    # instead of wasting propelant, let gravity work
    if goal_acc_y < 0:
        print('WARN: gravity')
        return 1.0, pi

    ctr_thrust = sqrt(goal_acc_x**2 + goal_acc_y**2) * craft.mass

    # best case, control is possible
    if ctr_thrust < craft.thrust:
        ctr_angle = atan2(goal_acc_y, goal_acc_x)
    # else, try to save what can be saved (fulfill y, best effort x)
    else:
        print('WARN: thrust norm', ctr_thrust/craft.thrust)
        ctr_thrust = craft.thrust

        ctr_angle = asin(goal_acc_y*craft.mass/craft.thrust)
        if goal_acc_x < 0:
            ctr_angle = pi-ctr_angle

    return ctr_thrust, ctr_angle
    */
}

/// Engine controller function
///
/// Input:
///     sc_attitude_desired, sc_attitude_current
///     sc_mass or sc_moment_of_inertia
///     sc_eng_thrust, sc_eng_gimbal_current, sc_eng_gimbal_max
/// Output:
///     commanded (engine) gimbal_angle (respecting engine constraints)
fn engine_controler() {
    /*
    TODO

    sc_moment_of_inertia = 1/2 * sc_mass * 2**2;  // 1/2*m*r**2 = kg.m**2

    sc_attitude_error = sc_attitude_desired - sc_attitude_current;

    // TODO check sc angular vel: max 5 deg/sec

    ctr_sc_att_acc = sc_attitude_error; // TODO PID. For now, assume a correction with T = 1 everywhere

    ctr_torque = sc_moment_of_inertia * ctr_sc_att_acc;  // N*m = kg*m**2 * rad/sec**2

    ctr_sc_eng_gimbal = asin(ctr_torque/(4*sc_eng_thrust));  // Torque = L*F*sin(alpha)

    // TODO limit eng max gimbal

    eng_gimbal_err = ctr_sc_eng_gimbal - sc_eng_gimbal_current;

    // check eng gimbal rate: max 1 deg / sec
    // apollo dps: 0.2 deg / sec (https://www.ibiblio.org/apollo/Documents/SGA_Memo04_660120.pdf)
    if eng_gimbal_err > 1 deg/sec:
        ctr_sc_eng_gimbal = sc_eng_gimbal_current + sign(eng_gimbal_err) * 1 deg/sec;

    */
}
