use std::{cmp::Ordering, ops::Index};

use crate::NumType;

pub struct Stats<N: NumType + Into<f64>> {
    sorted_values: Vec<N>,
    mean: f64,
    stdev: f64,
}

impl<N: NumType + Into<f64>> FromIterator<N> for Stats<N> {
    fn from_iter<T: IntoIterator<Item = N>>(iter: T) -> Self {
        let mut sum = 0.0;
        let mut count = 0.0;
        let mut values = vec![];
        for n in iter {
            sum += n.into();
            count += 1.0;
            values.push(n);
        }

        values.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
        let mean = sum / count;
        let ssd = values
            .iter()
            .map(|v| ((*v).into() - mean).powf(2.0))
            .sum::<f64>();
        Self {
            mean,
            stdev: (ssd / (values.len() - 1) as f64).sqrt(),
            sorted_values: values,
        }
    }
}

impl<N: NumType + Into<f64>> Stats<N> {
    pub fn mean(&self) -> f64 {
        self.mean
    }

    pub fn stdev(&self) -> f64 {
        self.stdev
    }

    pub fn min(&self) -> N {
        self.sorted_values[0]
    }

    pub fn max(&self) -> N {
        self.sorted_values[self.sorted_values.len() - 1]
    }

    pub fn median(&self) -> N {
        self.sorted_values[self.sorted_values.len() / 2]
    }
}

impl<N: NumType + Into<f64>> Index<usize> for Stats<N> {
    type Output = N;

    fn index(&self, index: usize) -> &Self::Output {
        &self.sorted_values[index]
    }
}
