use bits::BitArray;

#[derive(Clone, Debug)]
pub struct BooleanGridMap {
    cells_per_meter: u64,
    meters_per_side: u64,
    cells_per_side: u64,
    cells: BitArray
}

impl BooleanGridMap {

    pub fn new(cells_per_meter: u64, meters_per_side: u64) -> Self {
        let cells_per_side = cells_per_meter * meters_per_side;
        let total_cells = cells_per_side.pow(2);
        let cells = BitArray::zeros(total_cells);
        BooleanGridMap {cells_per_meter, meters_per_side, cells_per_side, cells}
    }

    pub fn set(&mut self, x: f64, y: f64, value: bool) {
        let i = self.point2index(x, y);

        self.cells.set(i, value);
    }

    pub fn is_set(&self, x: f64, y: f64) -> bool {
        self.cells.is_set(self.point2index(x, y))
    }

    fn cell_origin_offset(&self) -> u64 {
        self.cells_per_side / 2
    }

    fn resize(&mut self, resize_factor: u64) {
        let mut resized = BooleanGridMap::new(self.cells_per_meter, self.meters_per_side * resize_factor);
        let resized_start = self.cell_origin_offset();
        for (i, value) in self.cells.iter().enumerate() {
            let (x, y) = self.index2cell(i as u64);
            let resized_i = resized.cell2index(x + resized_start, y + resized_start);
            resized.cells.set(resized_i, value);
        }
        std::mem::swap(self, &mut resized);
    }

    fn coord2cell(&self, coord: f64) -> u64 {
        (coord * (self.cells_per_meter as f64) + (self.cell_origin_offset() as f64)) as u64
    }

    fn point2cell(&self, x: f64, y: f64) -> (u64, u64) {
        (self.coord2cell(x), self.coord2cell(y))
    }

    fn cell2index(&self, x: u64, y: u64) -> u64 {
        y * self.cells_per_side + x
    }

    fn index2cell(&self, index: u64) -> (u64, u64) {
        (index % self.cells_per_side, index / self.cells_per_side)
    }

    fn point2index(&self, x: f64, y: f64) -> u64 {
        let (x, y) = self.point2cell(x, y);
        self.cell2index(x, y)
    }
}

enum IndexAttempt {
    Valid(u64), ResizeFactor(u64)
}

#[cfg(test)]
mod tests {
    use crate::grid_map::BooleanGridMap;

    #[test]
    fn test() {
        let mut map = BooleanGridMap::new(2, 4);

    }
}