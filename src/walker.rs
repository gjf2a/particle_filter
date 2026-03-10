// Walker's algorithm for selection
//
// A. J. Walker, “An efficient method for generating discrete random variables with general distributions,”
//  ACM Transactions on Mathematical Software, vol. 3, no. 3, pp. 253–256, 1977.

use std::iter::repeat_n;

use hash_histogram::HashHistogram;
use rand::{RngExt, rng};

#[derive(Clone, Debug)]
pub struct WalkerAlias {
    n: usize,
    prob: Vec<f64>,
    alias: Vec<usize>,
}

impl WalkerAlias {
    pub fn rank_proportionate(n: usize) -> Self {
        let sum_n = Self::gauss_sum_n(n);
        let mut prob = vec![];
        let mut alias = vec![];
        for i in 0..n {
            let c = n - i;
            let p = (c * n) as f64 / sum_n as f64;
            if p >= 1.0 {
                prob.push(1.0);
                alias.push(i);
            } else {
                prob.push(p);
                alias.push(c - 1);
            }
        }
        Self { n, prob, alias }
    }

    pub fn weighted(weights: &HashHistogram<usize, f64>) -> Self {
        let n = weights.len();
        let total = weights.total_count();
        let probs_falling = weights
            .ranking_with_counts()
            .iter()
            .map(|(i, w)| (*i, *w / total))
            .collect::<Vec<_>>();
        let prob = repeat_n(0.0, n).collect::<Vec<_>>();
        let alias = (0..n).collect::<Vec<_>>();
        let mut result = Self { n, prob, alias };
        result.table_entries_from(&probs_falling);
        result
    }

    fn table_entries_from(&mut self, probs_falling: &Vec<(usize, f64)>) {
        let expected = 1.0 / self.n as f64;
        let mut overages = vec![];
        for (i, p) in probs_falling.iter() {
            if *p > expected {
                self.prob[*i] = 1.0;
                overages.push((*i, *p - expected));
            } else if *p < expected {
                let (over_i, over_w) = overages.last_mut().unwrap();
                *over_w -= expected - *p;
                self.prob[*i] = *p / expected;
                self.alias[*i] = *over_i;
                if *over_w <= 0.0 {
                    overages.pop();
                }
            } else {
                self.prob[*i] = 1.0;
            }
        }
    }

    fn gauss_sum_n(n: usize) -> usize {
        n * (n + 1) / 2
    }

    pub fn n(&self) -> usize {
        self.n
    }

    pub fn sum_n(&self) -> usize {
        Self::gauss_sum_n(self.n)
    }

    pub fn choose(&self) -> usize {
        let mut rng = rng();
        let choice = rng.random_range(0..self.n);
        if rng.random::<f64>() < self.prob[choice] {
            choice
        } else {
            self.alias[choice]
        }
    }
}

#[cfg(test)]
mod tests {
    use hash_histogram::HashHistogram;

    use crate::walker::WalkerAlias;

    #[test]
    fn test_rank_proportionate() {
        for n in 2..=8 {
            let mut histogram: HashHistogram<usize, usize> = HashHistogram::new();
            let distro = WalkerAlias::rank_proportionate(n);
            let num_samples = 100000;
            for _ in 0..num_samples {
                histogram.bump(&distro.choose());
            }
            let denominator = distro.sum_n();
            let tolerance = num_samples / (denominator * 5);
            println!("n: {n} ({})", distro.sum_n());
            for i in 0..distro.n() {
                let numerator = distro.n() - i;
                let target = num_samples * numerator / denominator;
                let lo = target - tolerance;
                let hi = target + tolerance;
                println!("{numerator}/{denominator}\t{lo}\t{target}\t{hi}");
                assert!(lo <= histogram.count(&i) && histogram.count(&i) <= hi);
            }
        }
    }

    #[test]
    fn test_weighted() {
        let weights = [(0, 0.2), (1, 0.25), (2, 0.1), (3, 0.45)]
            .iter()
            .copied()
            .collect::<HashHistogram<_, _>>();
        let distro = WalkerAlias::weighted(&weights);
        let mut histogram: HashHistogram<usize, usize> = HashHistogram::new();
        let num_samples = 100000;
        for _ in 0..num_samples {
            histogram.bump(&distro.choose());
        }
        let denominator = distro.sum_n();
        let tolerance = num_samples / (denominator * 5);
        for i in 0..distro.n() {
            let target = (num_samples as f64 * weights.count(&i) / weights.total_count()) as usize;
            let lo = target - tolerance;
            let hi = target + tolerance;
            println!("{lo}\t{target}\t{hi}");
            println!("count: {}", histogram.count(&i));
            assert!(lo <= histogram.count(&i) && histogram.count(&i) <= hi);
        }
    }
}
