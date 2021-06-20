#[macro_export]
macro_rules! squared {
    ($quantity:expr) => {{
        $quantity.powi(uom::typenum::P2::new())
    }};
}

#[macro_export]
macro_rules! sqrt {
    ($quantity:expr) => {{
        $quantity.value.powf(0.5)
    }};
}

#[macro_export]
macro_rules! mul {
    ($a:expr, $b:expr) => {{
        Vec2 {
            x: $a.x * $b,
            y: $a.y * $b,
        }
    }};
}

#[macro_export]
macro_rules! norm {
    ($quantity:expr) => {{
        sqrt!(squared!($quantity.x) + squared!($quantity.y))
    }};
}


#[macro_export]
macro_rules! modulo {
    ($val:expr, $modval:expr) => {{
        let mut ret = $val;
        while ret > ($modval/2.0).into() {
            ret -= $modval;
        }
        while ret < (-$modval/2.0).into() {
            ret += $modval;
        }
        ret
    }};
}
