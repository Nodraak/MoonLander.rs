use crate::gnc::common::Spacecraft;

pub fn gui(spacecraft: &Spacecraft) { // (craft, acc_cmd_x, acc_cmd_y, s, TGO):
    /*
    TODO

    moon_acc_y = -moon_gravity(craft.pos_y) +moon_centrifugal(craft.vel_x, craft.pos_y)

    a_x = acc_cmd_x.subs(s['v0'], craft.vel_x).subs(s['p0'], craft.pos_x).subs(s['tgo'], TGO)
    a_y = acc_cmd_y.subs(s['v0'], craft.vel_y).subs(s['p0'], craft.pos_y).subs(s['tgo'], TGO)

    return a_x, a_y-moon_acc_y
    */
}
