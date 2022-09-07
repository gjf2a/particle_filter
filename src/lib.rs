mod math_types;

use bits::BitArray;

pub struct GridMap {
    cells_per_meter: f64,
    meters_per_side: f64,
    cells: BitArray
}

impl GridMap {
    // Use BitArray::zeros() to initialize the correct number of cells.
}