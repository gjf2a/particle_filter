use std::cmp::Ordering;
use std::ops::{Add, Neg, Sub};
use bare_metal_modulo::{MNum, ModNumC};
use float_cmp::{ApproxEq, F64Margin};

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct RobotPosition {
    x: f64, y: f64, heading: Heading
}

impl RobotPosition {
    pub fn new() -> Self {
        RobotPosition {x: 0.0, y: 0.0, heading: Heading::new(0)}
    }

    pub fn from(x: f64, y: f64, heading: Heading) -> Self {
        RobotPosition {x, y, heading}
    }

    pub fn update(&mut self, motion: PolarCoord) {
        self.x += motion.x();
        self.y += motion.y();
        self.heading = self.heading + motion.theta.to_degrees() as i16;
    }

    pub fn updated_by(&self, motion: PolarCoord) -> Self {
        let mut result = self.clone();
        result.update(motion);
        result
    }

    pub fn position(&self) -> (f64, f64) {
        (self.x, self.y)
    }

    pub fn heading(&self) -> Heading {
        self.heading
    }

    pub fn offset_point(&self, distance: f64, offset: f64) -> (f64, f64) {
        let absolute_heading = offset + self.heading.radians();
        let displacement = PolarCoord::new(distance, absolute_heading);
        (self.x + displacement.x(), self.y + displacement.y())
    }
}

impl Add for RobotPosition {
    type Output = RobotPosition;

    fn add(self, rhs: Self) -> Self::Output {
        RobotPosition {x: self.x + rhs.x, y: self.y + rhs.y, heading: self.heading + rhs.heading}
    }
}

impl ApproxEq for RobotPosition {
    type Margin = F64Margin;

    fn approx_eq<M: Into<Self::Margin>>(self, other: Self, margin: M) -> bool {
        let margin = margin.into();
        self.x.approx_eq(other.x, margin) && self.y.approx_eq(other.y, margin) && self.heading == other.heading
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PositionBounds {
    bounds: [f64; 4]
}

pub fn replace_min(stored: &mut f64, other: f64) {
    *stored = f64_min(*stored, other);
}

pub fn replace_max(stored: &mut f64, other: f64) {
    *stored = f64_max(*stored, other)
}

pub fn f64_min(a: f64, b: f64) -> f64 {
    a.partial_cmp(&b).map_or(a, |c| match c {Ordering::Less => a, _ => b})
}

pub fn f64_max(a: f64, b: f64) -> f64 {
    a.partial_cmp(&b).map_or(a, |c| match c {Ordering::Less => b, _ => a})
}

const MIN_X: usize = 0;
const MIN_Y: usize = 1;
const MAX_X: usize = 2;
const MAX_Y: usize = 3;

impl PositionBounds {
    pub fn from_pts(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Self {
        PositionBounds {bounds: [min_x, min_y, max_x, max_y]}
    }

    pub fn from(positions: &Vec<RobotPosition>) -> Self {
        assert!(positions.len() >= 1);
        let mut result = Self::from_pts(positions[0].x, positions[0].y, positions[0].x, positions[0].y);
        for pos in positions.iter().skip(1) {
            result.add(pos);
        }
        result
    }

    pub fn min_x(&self) -> f64 {self.bounds[MIN_X]}
    pub fn min_y(&self) -> f64 {self.bounds[MIN_Y]}
    pub fn max_x(&self) -> f64 {self.bounds[MAX_X]}
    pub fn max_y(&self) -> f64 {self.bounds[MAX_Y]}

    pub fn add(&mut self, pos: &RobotPosition) {
        replace_min(&mut self.bounds[MIN_X], pos.x);
        replace_min(&mut self.bounds[MIN_Y], pos.y);
        replace_max(&mut self.bounds[MAX_X], pos.x);
        replace_max(&mut self.bounds[MAX_Y], pos.y);
    }

    pub fn width(&self) -> f64 {
        self.max_x() - self.min_x()
    }

    pub fn height(&self) -> f64 {
        self.max_y() - self.min_y()
    }

    pub fn max_bound(&self) -> f64 {
        f64_max(self.width(), self.height())
    }
}

impl ApproxEq for PositionBounds {
    type Margin = F64Margin;

    fn approx_eq<M: Into<Self::Margin>>(self, other: Self, margin: M) -> bool {
        let margin = margin.into();
        self.bounds.iter().zip(other.bounds.iter()).all(|(sel, oth)| sel.approx_eq(*oth, margin))
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct Heading {
    degrees: ModNumC<i16, 360>
}

impl Heading {
    pub fn new(degrees: i16) -> Self {
        Heading {degrees: ModNumC::new(degrees)}
    }

    pub fn from_radians(radians: f64) -> Self {
        Heading {degrees: ModNumC::new(radians.to_degrees() as i16)}
    }

    pub fn radians(&self) -> f64 {(self.degrees.a() as f64).to_radians()}
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
        Heading {degrees: -self.degrees}
    }
}

#[derive(Copy, Clone, PartialEq, Debug)]
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

    pub fn translate(&self, x: f64, y: f64) -> (f64, f64) {
        (x + self.x(), y + self.y())
    }

    /*pub fn rotated(&self, rotation: f64) -> Self {
        Self::new(self.r, self.theta + rotation)
    }
     */
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
    use float_cmp::assert_approx_eq;
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

    #[test]
    fn test_position() {
        let mut pos = RobotPosition::new();
        let mut path = vec![pos];
        pos = pos.updated_by(PolarCoord::new(10.0, 0.0));
        assert_eq!(pos, RobotPosition {x: 10.0, y: 0.0, heading: Heading::new(0)});
        path.push(pos);
        pos = pos.updated_by(PolarCoord::new(10.0, 90.0_f64.to_radians()));
        assert_eq!(pos, RobotPosition {x: 10.0, y: 10.0, heading: Heading::new(90)});
        path.push(pos);
        pos = pos.updated_by(PolarCoord::new(10.0, 180.0_f64.to_radians()));
        assert_approx_eq!(RobotPosition, pos, RobotPosition {x: 0.0, y: 10.0, heading: Heading::new(270)});
        path.push(pos);

        let bounds = PositionBounds::from(&path);
        assert_approx_eq!(PositionBounds, bounds, PositionBounds::from_pts(0.0, 0.0, 10.0, 10.0));
    }
}
