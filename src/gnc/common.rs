use crate::spacecraft::{SpacecraftStatic, SpacecraftDynamic};


pub struct Spacecraft {
    pub spec: SpacecraftStatic,         // fixed properties
    pub cur: SpacecraftDynamic,         // latest changing properties
    pub all: Vec<SpacecraftDynamic>,    // all changing properties
}


impl Spacecraft {
    pub fn new() -> Spacecraft {
        Spacecraft {
            spec: SpacecraftStatic::new(),
            cur: SpacecraftDynamic::new(),
            all: vec![],
        }
    }
}
