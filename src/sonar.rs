use crate::grid_map::BooleanGridMap;
use crate::{RobotPosition, SensorCorrection};
use array_init::array_init;
use counting_ratio::CountingRatio;
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

    pub fn reading_in_range(&self, distance_reading: f64) -> bool {
        distance_reading < self.range_meters
    }

    pub fn contact_points(&self, robot: &RobotPosition, distance_reading: f64, num_points: usize) -> Vec<(f64, f64)> {
        let mut result = Vec::new();
        for i in 0..num_points {
            let cone_width_offset = self.cone_width.radians() * i as f64 / num_points as f64;
            let point_heading = self.orientation.radians() + cone_width_offset - self.cone_width.radians() / 2.0;
            result.push(robot.offset_point(distance_reading, point_heading));
        }
        result
    }
}

#[derive(Clone, Debug)]
pub struct SonarMap<const N: usize> {
    map: BooleanGridMap,
    sonars: [Sonar; N],
    num_sonar_points: usize
}

impl <const N: usize> SonarMap<N> {
    pub fn new(orientations: &[Heading; N], cone_width: Heading, range_meters: f64, cells_per_meter: u64, meters_per_side: u64, num_sonar_points: usize) -> Self {
        SonarMap {
            map: BooleanGridMap::new(cells_per_meter, meters_per_side),
            sonars: array_init(|i| Sonar { range_meters, orientation: orientations[i], cone_width}),
            num_sonar_points
        }
    }
}

impl <const N: usize> SensorCorrection for SonarMap<N> {
    type SensorReading = [f64; N];

    fn fit(&self, position: &RobotPosition, reading: &Self::SensorReading) -> f64 {
        let mut count = CountingRatio::new();
        for (i, d) in reading.iter().enumerate() {
            let observation = self.sonars[i].reading_in_range(*d) && self.sonars[i].contact_points(position, *d, self.num_sonar_points).iter().any(|(x, y)| self.map.is_set(*x, *y));
            count.observe(observation);
        }
        count.into()
    }

    fn update_from(&mut self, position: &RobotPosition, reading: &Self::SensorReading) {
        for i in 0..N {
            if self.sonars[i].reading_in_range(reading[i]) {
                for (x, y) in self.sonars[i].contact_points(position, reading[i], self.num_sonar_points) {
                    self.map.set(x, y);
                }
            }
        }
    }
}