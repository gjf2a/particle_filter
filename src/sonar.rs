use crate::grid_map::GridMap;
use init_with::InitWith;

pub struct Sonar {
    range: f64,
    orientation: f64,
    cone_width: f64
}

pub struct SonarMap<const N: usize> {
    map: GridMap,
    sonars: [Sonar; N]
}

impl <const N: usize> SonarMap<N> {
    pub fn new(orientations: &[f64; N], cone_width: f64, range: f64, cells_per_meter: u64, meters_per_side: u64) -> Self {
        SonarMap {
            map: GridMap::new(cells_per_meter, meters_per_side),
            sonars: <[Sonar; N]>::init_with_indices(|i| Sonar {range, orientation: orientations[i], cone_width})
        }
    }
}