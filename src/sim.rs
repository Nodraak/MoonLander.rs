
pub struct Sim {

}

impl Sim {
    pub fn new() -> Sim {
        Sim {}
    }

    pub fn tick(&self) {
        /*
        TODO

        acc = self.thrust*thrust_ratio/self.mass
        self.acc_x = acc*cos(angle)
        self.acc_y = acc*sin(angle) - moon_gravity(self.pos_y) + moon_centrifugal(self.vel_x, self.pos_y)
        self.g = self.acc_y/G0

        self.vel_x += self.acc_x*dt
        self.vel_y += self.acc_y*dt

        self.pos_x += self.vel_x*dt
        self.pos_y += self.vel_y*dt

        self.mass -= self.mass_flow_direction*self.mass_flow*thrust_ratio*dt
        self.dv_flight += acc*dt
        self.t += dt

        if self.p:
            self.p.tick(self.t, self.pos_x, self.pos_y, self.vel_x, self.vel_y, self.acc_x, self.acc_y, angle, thrust_ratio)
        */
    }
}
