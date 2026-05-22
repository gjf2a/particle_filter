use crate::{
    MapInput, MapObstacle, ObstacleNoise,
    angle::{Angle, Degrees, Radians},
};
use std::{f64::consts::PI, str::FromStr};

pub const RADIUS_M: f64 = 0.2032;
pub const RADIUS_IR_M: f64 = RADIUS_M * 1.1;
pub const RADIUS_STDEV_M: f64 = 0.01; // TODO: 1 cm for now, but need to rethink.
pub const HEADING_STDEV_RADIANS: f64 = PI / 8.0;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Bump {
    FrontCenter,
    FrontLeft,
    FrontRight,
    Left,
    Right,
}

impl FromStr for Bump {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> anyhow::Result<Self> {
        let start = s.find('\'').map_or(0, |i| i + 1);
        let end = s.rfind('\'').unwrap_or(s.len());
        let label = &s[start..end];
        match label {
            "bump_front_center"
            | "cliff_front_center"
            | "cliff_front_left', 'cliff_front_right" => Ok(Bump::FrontCenter),
            "bump_front_left" | "cliff_front_left" | "cliff_side_left', 'cliff_front_left" => {
                Ok(Bump::FrontLeft)
            }
            "bump_front_right" | "cliff_front_right" => Ok(Bump::FrontRight),
            "bump_left" | "cliff_side_left" => Ok(Bump::Left),
            "bump_right" | "cliff_side_right" => Ok(Bump::Right),
            _ => Err(anyhow::anyhow!("Did not recognize '{label}'")),
        }
    }
}

impl Bump {
    pub fn angle_offset(&self) -> Radians {
        match self {
            Bump::FrontCenter => Radians::new(0.0),
            Bump::FrontLeft => Radians::new(PI / 4.0),
            Bump::FrontRight => Radians::new(-PI / 4.0),
            Bump::Left => Radians::new(PI / 2.0),
            Bump::Right => Radians::new(-PI / 2.0),
        }
    }

    pub fn obstacle_at(&self) -> MapInput {
        let noise = ObstacleNoise {
            stdev_distance: RADIUS_STDEV_M,
            stdev_heading: Radians::new(HEADING_STDEV_RADIANS),
        };
        MapInput::Collision(MapObstacle {
            distance: RADIUS_M,
            heading: self.angle_offset(),
            noise,
        })
    }
}

pub enum IrHazard {
    SideLeft(u16),
    Left(u16),
    FrontLeft(u16),
    FrontCenterLeft(u16),
    FrontCenterRight(u16),
    FrontRight(u16),
    Right(u16),
}

impl IrHazard {
    // Angle offsets from https://github.com/iRobotEducation/create3_docs/discussions/342
    pub fn angle_offset(&self) -> Radians {
        Degrees::new(match self {
            Self::SideLeft(_) => 65.3,
            Self::Left(_) => 38.0,
            Self::FrontLeft(_) => 20.0,
            Self::FrontCenterLeft(_) => 3.0,
            Self::FrontCenterRight(_) => -14.25,
            Self::FrontRight(_) => -34.0,
            Self::Right(_) => -65.3,
        })
        .radians()
    }

    pub fn intensity(&self) -> u16 {
        match self {
            IrHazard::SideLeft(ir) => *ir,
            IrHazard::Left(ir) => *ir,
            IrHazard::FrontLeft(ir) => *ir,
            IrHazard::FrontCenterLeft(ir) => *ir,
            IrHazard::FrontCenterRight(ir) => *ir,
            IrHazard::FrontRight(ir) => *ir,
            IrHazard::Right(ir) => *ir,
        }
    }

    pub fn reading_at(&self) -> MapInput {
        let noise = ObstacleNoise {
            stdev_distance: RADIUS_STDEV_M,
            stdev_heading: Radians::new(HEADING_STDEV_RADIANS),
        };
        MapInput::RangeObject(MapObstacle {
            distance: RADIUS_IR_M,
            heading: self.angle_offset(),
            noise,
        })
    }
}
