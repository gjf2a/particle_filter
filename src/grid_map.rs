use bits::BitArray;

#[derive(Clone, Debug)]
pub struct GridMap {
    cells_per_meter: u64,
    meters_per_side: u64,
    cells_per_side: u64,
    cells: BitArray,
    viewed_cells: BitArray
}

impl GridMap {

    pub fn new(cells_per_meter: u64, meters_per_side: u64) -> Self {
        let cells_per_side = cells_per_meter * meters_per_side;
        let total_cells = cells_per_side.pow(2);
        let cells = BitArray::zeros(total_cells);
        let viewed_cells = BitArray::zeros(total_cells);
        GridMap {cells_per_meter, meters_per_side, cells_per_side, cells, viewed_cells}
    }


}