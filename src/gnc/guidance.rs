use crate::conf::GuiSpacecraft;
use crate::gnc::common::Spacecraft;
use crate::utils::math::Vec2;


pub fn gui(spacecraft: &mut Spacecraft, tgo: f64) {
    let body_acc_y = (
        -spacecraft.conf.s.body.gravity(spacecraft.cur.pos.y)
        +spacecraft.conf.s.body.centrifugal(spacecraft.cur.vel.x, spacecraft.cur.pos.y)
    );

    let acc = match spacecraft.conf.s.gui_spacecraft {
        GuiSpacecraft::GuiDescent => gui_descent(spacecraft, tgo),
        GuiSpacecraft::GuiAscent => gui_ascent(spacecraft, tgo),
    };

    spacecraft.cur.gui = acc + Vec2 {
        x: 0.0,
        y: -body_acc_y,
    }
}


/// Descent guidance
///
/// x: Quadratic law
///     acc = k1*t**2+k2*t+k3
///     acc = af -6/tgo*(v0+vf) + 12/tgo.powi(2)*(pf-p0)
/// y: Quadratic law
///     acc = k1*t**2+k2*t+k3
///     acc = af -6/tgo*(v0+vf) + 12/tgo.powi(2)*(pf-p0)
/// For more info, cf. https://blog.nodraak.fr/2020/12/aerospace-sim-2-guidance-law/
pub fn gui_descent(spacecraft: &Spacecraft, tgo: f64) -> Vec2 {
    let conf = spacecraft.conf.s;

    // x

    let v0_x = spacecraft.cur.vel.x;
    let p0_x = spacecraft.cur.pos.x;

    let acc_x = (
        conf.gui_af_x
        -6.0/tgo*(v0_x+conf.gui_vf_x)
        + 12.0/tgo.powi(2)*(conf.gui_pf_x-p0_x)
    );

    // y

    let v0_y = spacecraft.cur.vel.y;
    let p0_y = spacecraft.cur.pos.y;

    let acc_y = (
        conf.gui_af_y
        -6.0/tgo*(v0_y+conf.gui_vf_y)
        + 12.0/tgo.powi(2)*(conf.gui_pf_y-p0_y)
    );

    // return

    Vec2 {
        x: acc_x,
        y: acc_y,
    }
}


/// Ascent guidance
///
/// x: Linear law:
///     acc = k1*t+k2
///     acc = -af + 2/tgo*(vf-v0)
/// y: Quadratic law:
///     acc = k1*t**2+k2*t+k3
///     acc = af -6/tgo*(v0+vf) + 12/tgo.powi(2)*(pf-p0)
/// For more info, cf. https://blog.nodraak.fr/2020/12/aerospace-sim-2-guidance-law/
pub fn gui_ascent(spacecraft: &Spacecraft, tgo: f64) -> Vec2 {
    let conf = spacecraft.conf.s;

    // x

    let v0_x = spacecraft.cur.vel.x;

    let acc_x = (
        -conf.gui_af_x
        + 2.0/tgo*(conf.gui_vf_x-v0_x)
    );

    // y

    let v0_y = spacecraft.cur.vel.y;
    let p0_y = spacecraft.cur.pos.y;

    let acc_y = (
        conf.gui_af_y
        -6.0/tgo*(v0_y+conf.gui_vf_y)
        + 12.0/tgo.powi(2)*(conf.gui_pf_y-p0_y)
    );

    // return

    Vec2 {
        x: acc_x,
        y: acc_y,
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
