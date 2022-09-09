use crate::grid_map::BooleanGridMap;
use init_with::InitWith;
use crate::{RobotPosition, SensorMap};

pub struct Sonar {
    range_meters: f64,
    orientation: f64,
    cone_width: f64
}

pub struct SonarMap<const N: usize> {
    map: BooleanGridMap,
    sonars: [Sonar; N]
}

impl <const N: usize> SonarMap<N> {
    pub fn new(orientations: &[f64; N], cone_width: f64, range_meters: f64, cells_per_meter: u64, meters_per_side: u64) -> Self {
        SonarMap {
            map: BooleanGridMap::new(cells_per_meter, meters_per_side),
            sonars: <[Sonar; N]>::init_with_indices(|i| Sonar { range_meters, orientation: orientations[i], cone_width})
        }
    }
}

impl <const N: usize> SensorMap for SonarMap<N> {
    type SensorReading = [f64; N];

    fn fit(&self, position: &RobotPosition, reading: &Self::SensorReading) -> f64 {
        todo!()
    }

    fn update_from(&mut self, reading: &Self::SensorReading) {
        todo!()
    }
}