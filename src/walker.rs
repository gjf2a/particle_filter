// Walker's algorithm for selection
// A. J. Walker, “An efficient method for generating discrete random variables with general distributions,” ACM Transactions on Mathematical Software, vol. 3, no. 3, pp. 253–256, 1977.
// https://crates.io/crates/weighted_rand
//
// Imagine 4 elements. Prob[0] = 0.4, Prob[1] = 0.3, Prob[2] = 0.2, Prob[1] = 0.1
// Generate a number from 0..=3 uniformly.
// P(0|0) = 0.25 * 0.4 = 0.1
// P(1|1) = 0.25 * 0.3 = 0.075
// P(2|2) = 0.25 * 0.2 = 0.05
// P(3|3) = 0.25 * 0.1 = 0.025
// 
// Complementary probabilities
// P(~0|0) = 0.25 * 0.6 = 0.15
// P(~1|1) = 0.25 * 0.7 = 0.175
// P(~2|2) = 0.25 * 0.8 = 0.2
// P(~3|3) = 0.25 * 0.9 = 0.225
//
// Check work: Sum of all 8 = 1.0
//
// P(3|3) = 0.4, since 0.4 * 0.25 = 0.1. This leaves us 0.6 * 0.25 = 0.15 to use
// P(2|2) = 0.8, since 0.8 * 0.25 = 0.2. Leaves us 0.2 * 0.25 = 0.05 to use
// P(1|1) = 1.0. Set alias[2] = 1 => 1.0 * 0.25 + 0.2 * 0.25 = 0.3
// P(0|0) = 1.0. Set alias[3] = 0 => 1.0 * 0.25 * 0.6 * 0.25 = 0.4
//
// General formula
// Find sum S of 1..=N.
// One probability unit is 1/S.
// Start at the end
// * Last element is 1/S / 1/N == N / S
//   * Alias is 1st element
// * Next element is 2/S / 1/N == 2N / S
//   * Alias is 2nd element
// * Continues up to halfway point

use rand::{RngExt, rng};

#[derive(Clone, Debug)]
pub struct WalkerDistribution {
    n: usize,
    prob: Vec<f64>,
    alias: Vec<usize>,
}

impl WalkerDistribution {
    pub fn new(n: usize) -> Self {
        let s = n * (n + 1) / 2;
        let last_index = n/2;
        let mut prob = vec![];
        let mut alias = vec![];
        for c in (1..=last_index).rev() {
            prob.push((c * n) as f64 / s as f64);
            alias.push(c - 1);
        }
        Self {n, prob, alias}
    }

    pub fn choose(&self) -> usize {
        let mut rng = rng();
        let prob = rng.random_range(0..self.n);
        let top_lower_index = (self.n - 1) / 2;
        if prob <= top_lower_index {
            prob
        } else {
            let lowprob = prob - (top_lower_index + 1);
            if rng.random::<f64>() < self.prob[lowprob] {
                prob 
            } else {
                self.alias[lowprob]
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use hash_histogram::HashHistogram;

    use crate::walker::WalkerDistribution;

    #[test]
    fn test() {
        let mut histogram: HashHistogram<usize, usize> = HashHistogram::new();
        let four = WalkerDistribution::new(4);
        for _ in 0..100000 {
            histogram.bump(&four.choose());
        }
        assert!(39000 <= histogram.count(&0) && histogram.count(&0) <= 41000);
        assert!(29000 <= histogram.count(&1) && histogram.count(&1) <= 31000);
        assert!(19000 <= histogram.count(&2) && histogram.count(&2) <= 21000);
        assert!(9000 <= histogram.count(&3) && histogram.count(&3) <= 11000);
    }
}