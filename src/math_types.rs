use std::ops::{Add, Neg, Sub};

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct Heading {
    degrees: i16
}

impl Heading {
    pub fn new(degrees: i16) -> Self {
        let mut degrees = degrees;
        while degrees < 0 {
            degrees += 360;
        }
        Heading {degrees: degrees % 360}
    }

    pub fn degrees(&self) -> i16 {self.degrees}
    pub fn radians(&self) -> f64 {(self.degrees as f64).to_radians()}
}

impl Add<Heading> for Heading {
    type Output = Heading;

    fn add(self, rhs: Self) -> Self::Output {
        Heading {degrees: self.degrees + rhs.degrees}
    }
}

impl Add<i16> for Heading {
    type Output = Heading;

    fn add(self, rhs: i16) -> Self::Output {
        self + Heading::new(rhs)
    }
}

impl Sub<Heading> for Heading {
    type Output = Heading;

    fn sub(self, rhs: Self) -> Self::Output {
        Heading {degrees: self.degrees - rhs.degrees}
    }
}

impl Sub<i16> for Heading {
    type Output = Heading;

    fn sub(self, rhs: i16) -> Self::Output {
        self - Heading::new(rhs)
    }
}

impl Neg for Heading {
    type Output = Heading;

    fn neg(self) -> Self::Output {
        Heading {degrees: self.degrees + 180}
    }
}

pub struct PolarCoord {
    r: f64, theta: f64
}

impl PolarCoord {
    pub fn new(r: f64, theta: f64) -> Self {
        PolarCoord {r, theta}
    }

    pub fn x(&self) -> f64 {
        self.r * self.theta.cos()
    }

    pub fn y(&self) -> f64 {
        self.r * self.theta.sin()
    }

    pub fn rotated(&self, rotation: f64) -> Self {
        Self::new(self.r, self.theta + rotation)
    }
}

impl Add for PolarCoord {
    type Output = PolarCoord;

    fn add(self, rhs: Self) -> Self::Output {
        let x_sum = self.x() + rhs.x();
        let y_sum = self.y() + rhs.y();
        PolarCoord::new((x_sum.powi(2) + y_sum.powi(2)).sqrt(), y_sum.atan2(x_sum))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_heading() {
        let h = Heading::new(-270);
        assert_eq!(h, Heading::new(90));
        assert_eq!(-h, Heading::new(270));
        assert_eq!(h + 30, Heading::new(120));
        assert_eq!(h - 405, Heading::new(45));
        assert_eq!(h + 765, Heading::new(135));
    }
}
