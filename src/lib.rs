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
