use crate::gnc::common::Spacecraft;
use crate::utils::math::Vec2;
use crate::utils::space::{moon_centrifugal, moon_gravity};


/// Quadratic law: acc = af -6/tgo*(v0+vf) + 12/tgo.powi(2)*(pf-p0)
/// For more info, cf. https://blog.nodraak.fr/2020/12/aerospace-sim-2-guidance-law/
pub fn gui(spacecraft: &Spacecraft, tgo: f64) -> Vec2 {

    let moon_acc_y = -moon_gravity(spacecraft.cur.pos.y) +moon_centrifugal(spacecraft.cur.vel.x, spacecraft.cur.pos.y);

    // x

    let v0_x = spacecraft.cur.vel.x;
    let p0_x = spacecraft.cur.pos.x;

    let acc_x = (
        spacecraft.conf.gui_af_x
        -6.0/tgo*(v0_x+spacecraft.conf.gui_vf_x)
        + 12.0/tgo.powi(2)*(spacecraft.conf.gui_pf_x-p0_x)
    );

    // y

    let v0_y = spacecraft.cur.vel.y;
    let p0_y = spacecraft.cur.pos.y;

    let acc_y = (
        spacecraft.conf.gui_af_y
        -6.0/tgo*(v0_y+spacecraft.conf.gui_vf_y)
        + 12.0/tgo.powi(2)*(spacecraft.conf.gui_pf_y-p0_y)
    );

    // return

    Vec2 {
        x: acc_x,
        y: acc_y-moon_acc_y,
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gui_1() {
        // TODO
    }
}
