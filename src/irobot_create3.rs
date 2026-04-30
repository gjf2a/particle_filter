use crate::{MapInput, MapObstacle, ObstacleNoise, angle::{Angle, Radians}};
use std::{f64::consts::PI, str::FromStr};

pub const RADIUS_M: f64 = 0.2032;
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
        let noise = ObstacleNoise {stdev_distance: RADIUS_STDEV_M, stdev_heading: Radians::new(HEADING_STDEV_RADIANS)};
        MapInput::Collision(MapObstacle { distance: RADIUS_M, heading: self.angle_offset(), noise })
    }
}
