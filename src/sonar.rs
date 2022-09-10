use crate::grid_map::BooleanGridMap;
use crate::{RobotPosition, SensorMap};
use array_init::array_init;
use crate::position_types::Heading;

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Sonar {
    range_meters: f64,
    orientation: Heading,
    cone_width: Heading
}

impl Sonar {
    pub fn new(range_meters: f64, orientation: Heading, cone_width: Heading) -> Self {
        Sonar {range_meters, orientation, cone_width}
    }

    pub fn contact_points(&self, robot: RobotPosition, distance_reading: f64, num_points: usize) -> Vec<(f64, f64)> {
        Vec::new()
    }
}

#[derive(Clone, Debug)]
pub struct SonarMap<const N: usize> {
    map: BooleanGridMap,
    sonars: [Sonar; N]
}

impl <const N: usize> SonarMap<N> {
    pub fn new(orientations: &[Heading; N], cone_width: Heading, range_meters: f64, cells_per_meter: u64, meters_per_side: u64) -> Self {
        SonarMap {
            map: BooleanGridMap::new(cells_per_meter, meters_per_side),
            sonars: array_init(|i| Sonar { range_meters, orientation: orientations[i], cone_width})
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