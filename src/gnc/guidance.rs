use crate::gnc::common::Spacecraft;
use crate::utils::math::Vec2;
use crate::utils::space::{moon_centrifugal, moon_gravity};


/// Quadratic law: acc = af -6/tgo*(v0+vf) + 12/tgo.powi(2)*(pf-p0)
pub fn gui(spacecraft: &Spacecraft, tgo: f64) -> Vec2 {

    let moon_acc_y = -moon_gravity(spacecraft.cur.pos.y) +moon_centrifugal(spacecraft.cur.vel.x, spacecraft.cur.pos.y);

    // x

    let af_x = 0.0;
    let vf_x = 0.0;
    let pf_x = 405_000.0;  // TODO from conf

    let v0_x = spacecraft.cur.vel.x;
    let p0_x = spacecraft.cur.pos.x;

    let acc_x = af_x -6.0/tgo*(v0_x+vf_x) + 12.0/tgo.powi(2)*(pf_x-p0_x);

    // y

    let af_y = 0.0;
    let vf_y = 0.0;
    let pf_y = 0.0;

    let v0_y = spacecraft.cur.vel.y;
    let p0_y = spacecraft.cur.pos.y;

    let acc_y = af_y -6.0/tgo*(v0_y+vf_y) + 12.0/tgo.powi(2)*(pf_y-p0_y);

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
