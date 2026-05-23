use crate::{
    MapInput, MapObstacle, ObstacleNoise,
    angle::{Angle, Degrees, Radians},
};
use std::{f64::consts::PI, str::FromStr};

pub const RADIUS_M: f64 = 0.2032;
// This sensor is pretty unreliable, so this is a conservative estimate.
pub const IR_SPACE_M: f64 = 0.02;
pub const RADIUS_IR_M: f64 = RADIUS_M + IR_SPACE_M;
pub const RADIUS_STDEV_M: f64 = 0.01; // TODO: 1 cm for now, but need to rethink.
pub const HEADING_STDEV_RADIANS: f64 = PI / 8.0;
pub const MIN_IR_OBSTACLE_PRESENT: i16 = 30;

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

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct IrReading {
    intensity: i16,
    heading: IrHeading,
}

impl IrReading {
    pub fn new(intensity: i16, heading: IrHeading) -> Self {
        Self { intensity, heading }
    }

    pub fn angle_offset(&self) -> Radians {
        self.heading.angle_offset()
    }

    pub fn intensity(&self) -> i16 {
        self.intensity
    }

    pub fn reading_at(&self) -> MapInput {
        let noise = ObstacleNoise {
            stdev_distance: RADIUS_STDEV_M,
            stdev_heading: Radians::new(HEADING_STDEV_RADIANS),
        };
        if self.intensity > MIN_IR_OBSTACLE_PRESENT {
            MapInput::RangeObject(MapObstacle {
                distance: RADIUS_IR_M,
                heading: self.angle_offset(),
                noise,
            })
        } else {
            MapInput::FreeSpace(IR_SPACE_M, self.angle_offset(), noise)
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum IrHeading {
    SideLeft,
    Left,
    FrontLeft,
    FrontCenterLeft,
    FrontCenterRight,
    FrontRight,
    Right,
}

impl IrHeading {
    // Angle offsets from https://github.com/iRobotEducation/create3_docs/discussions/342
    pub fn angle_offset(&self) -> Radians {
        Degrees::new(match self {
            Self::SideLeft => 65.3,
            Self::Left => 38.0,
            Self::FrontLeft => 20.0,
            Self::FrontCenterLeft => 3.0,
            Self::FrontCenterRight => -14.25,
            Self::FrontRight => -34.0,
            Self::Right => -65.3,
        })
        .radians()
    }
}

impl FromStr for IrHeading {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> anyhow::Result<Self> {
        match s {
            "ir_intensity_side_left" => Ok(Self::SideLeft),
            "ir_intensity_left" => Ok(Self::Left),
            "ir_intensity_front_left" => Ok(Self::FrontLeft),
            "ir_intensity_front_center_left" => Ok(Self::FrontCenterLeft),
            "ir_intensity_front_center_right" => Ok(Self::FrontCenterRight),
            "ir_intensity_front_right" => Ok(Self::FrontRight),
            "ir_intensity_right" => Ok(Self::Right),
            _ => Err(anyhow::anyhow!("Did not recognize '{s}'")),
        }
    }
}
