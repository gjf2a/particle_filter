use std::cmp::max;
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
        let i = match self.point2index(x, y) {
            IndexAttempt::Valid(i) => {i}
            IndexAttempt::ResizeFactor(r) => {
                self.resize(r);
                match self.point2index(x, y) {
                    IndexAttempt::Valid(i) => {i}
                    IndexAttempt::ResizeFactor(_) => {panic!("Resize failed.")}
                }
            }
        };

        self.cells.set(i, value);
    }

    pub fn is_set(&self, x: f64, y: f64) -> bool {
        match self.point2index(x, y) {
            IndexAttempt::Valid(i) => {self.cells.is_set(i)}
            _ => {false}
        }
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

    fn coord2cell(&self, coord: f64) -> IndexAttempt<u64> {
        let cell = coord * (self.cells_per_meter as f64) + (self.cell_origin_offset() as f64);
        if cell < 0.0 || cell >= self.cells_per_side as f64 {
            IndexAttempt::ResizeFactor(self.get_size_multiplier(cell.abs()))
        } else {
            IndexAttempt::Valid(cell as u64)
        }
    }

    fn get_size_multiplier(&self, requested: f64) -> u64 {
        2 * max(1, (requested / self.cells_per_side as f64) as u64)
    }

    fn point2cell(&self, x: f64, y: f64) -> IndexAttempt<(u64, u64)> {
        let x = self.coord2cell(x);
        let y = self.coord2cell(y);
        match x {
            IndexAttempt::Valid(xv) => {
                match y {
                    IndexAttempt::Valid(yv) => {IndexAttempt::Valid((xv, yv))}
                    IndexAttempt::ResizeFactor(ry) => {IndexAttempt::ResizeFactor(ry)}
                }
            }
            IndexAttempt::ResizeFactor(rx) => {
                match y {
                    IndexAttempt::Valid(_) => {IndexAttempt::ResizeFactor(rx)}
                    IndexAttempt::ResizeFactor(ry) => {
                        IndexAttempt::ResizeFactor(if rx > ry {rx} else {ry})
                    }
                }
            }
        }
    }

    fn cell2index(&self, x: u64, y: u64) -> u64 {
        y * self.cells_per_side + x
    }

    fn index2cell(&self, index: u64) -> (u64, u64) {
        (index % self.cells_per_side, index / self.cells_per_side)
    }

    fn point2index(&self, x: f64, y: f64) -> IndexAttempt<u64> {
        match self.point2cell(x, y) {
            IndexAttempt::Valid((x, y)) => {IndexAttempt::Valid(self.cell2index(x, y))}
            IndexAttempt::ResizeFactor(r) => {IndexAttempt::ResizeFactor(r)}
        }
    }
}

#[derive(Copy, Clone)]
enum IndexAttempt<T: Copy> {
    Valid(T), ResizeFactor(u64)
}

#[cfg(test)]
mod tests {
    use crate::grid_map::BooleanGridMap;

    #[test]
    fn basic_test() {
        let mut map = BooleanGridMap::new(2, 4);
        let targets = [(0.0, 0.0), (0.5, 0.0), (-1.0, 0.5), (-1.0, -0.5)];
        for (x, y) in targets.iter() {
            map.set(*x, *y, true);
            assert!(map.is_set(*x, *y));
        }
        let still_clear = [(1.0, 0.0), (3.0, 2.0), (2.0, 1.0)];
        for (x, y) in still_clear.iter() {
            assert!(!map.is_set(*x, *y));
        }
    }
}