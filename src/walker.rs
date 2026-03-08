// Walker's algorithm for selection
// A. J. Walker, “An efficient method for generating discrete random variables with general distributions,” ACM Transactions on Mathematical Software, vol. 3, no. 3, pp. 253–256, 1977.
// https://crates.io/crates/weighted_rand
//
// Adapted for rank-proportionate selection. Given n ranked values:
// * Find the sum of the ranks
// * Assign a probability of 1/sum for the lowest ranked, 2/sum for the 
//   next-lowest, and so forth.

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
        let last_index = n/2;
        let mut prob = vec![];
        let mut alias = vec![];
        for c in 1..=last_index {
            prob.push((c * n) as f64 / sum_n as f64);
            alias.push(n - c);
        }
        Self {n, prob, alias}
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
        let prob = rng.random_range(0..self.n);
        if prob >= self.prob.len() {
            prob
        } else {
            if rng.random::<f64>() < self.prob[prob] {
                prob 
            } else {
                self.alias[prob]
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use hash_histogram::HashHistogram;

    use crate::walker::WalkerAlias;

    #[test]
    fn test() {
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
                let numerator = i + 1;
                let target = num_samples * numerator / denominator;
                let lo = target - tolerance;
                let hi = target + tolerance;
                println!("{numerator}/{denominator}\t{lo}\t{target}\t{hi}");
                assert!(lo <= histogram.count(&i) && histogram.count(&i) <= hi);
            }
        }
    }
}