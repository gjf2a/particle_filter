use std::f64::consts::PI;
use std::fs::File;
use std::io::{BufRead, BufReader};
use crate::{PolarCoord, RobotPosition};
use crate::position_types::Heading;

pub const COUNTS_PER_ROTATION: f64 = 360.0;

pub const EV3_WHEEL_DIAMETER: f64 = 5.5;
pub const EV3_WHEEL_RADIUS: f64 = EV3_WHEEL_DIAMETER / 2.0;

// This number is very tricky to measure.
//
// Using a ruler, I got 12 cm.
//
// By spinning the robot in a circle, I worked backwards from the wheel radius to calculate
// 10 cm. I know the wheel radius is correct because my straight-line measurements worked great.
//
// Calculation methodology:
//
// Variables:
// r_c = rotation counts
// w_d = wheel diameter
// w_c = wheel circumference
// w_t = distance of wheel travel
// s_c = circumference of circle formed when robot spins in place
// w_s = distance between wheels (wheel separation)
//
// w_c = w_d * PI = 5.5 * PI \approx 17.3 cm
// w_t = w_c * r_c/360 = 17.3 * 165/360 \approx 7.9 cm
// * i.e., what portion of the circumference did we cover?
//
// s_c = PI * w_s
// w_t/s_c = 1/4 (because a 90 degree turn is 1/4 of a circle)
// w_t/(PI * w_s) = 1/4 => PI * w_s = 7.9 * 4 => w_s \approx 10.08 cm
//
pub const EV3_SEPARATION_MODEL_1: f64 = 10.08;

pub const BOT: TwoWheelBase = TwoWheelBase::new(EV3_SEPARATION_MODEL_1, EV3_WHEEL_RADIUS);

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct MotorData {
    pub left_count: i64, pub right_count: i64,
    pub left_speed: i64, pub right_speed: i64
}

impl MotorData {
    pub fn speeds(&self) -> (i64, i64) {
        (self.left_speed, self.right_speed)
    }

    pub fn counts(&self) -> (i64, i64) {
        (self.left_count, self.right_count)
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Sonar3Data {
    pub sonar_front: i64, pub sonar_left: i64, pub sonar_right: i64
}

#[derive(Copy, Clone, Debug)]
pub struct RobotSensorPosition {
    base: TwoWheelBase,
    action_start_left: i64, action_start_right: i64, action_start_pos: RobotPosition,
    pos: RobotPosition,
    num_updates: i64,
    last_action: MotorData
}

impl RobotSensorPosition {
    pub fn new(base: TwoWheelBase) -> Self {
        RobotSensorPosition {
            base,
            action_start_left: 0,
            action_start_right: 0,
            action_start_pos: RobotPosition::new(),
            last_action: MotorData {left_count: 0, left_speed: 0, right_count: 0, right_speed: 0},
            pos: RobotPosition::new(),
            num_updates: 0,
        }
    }

    pub fn motor_update(&mut self, datum: MotorData) {
        if datum.speeds() != self.last_action.speeds() {
            self.action_start_pos = self.pos;
            self.action_start_left = self.last_action.left_count;
            self.action_start_right = self.last_action.right_count;
        }
        self.pos = self.base.updated_position(self.action_start_pos,
                                              datum.left_count - self.action_start_left,
                                              datum.right_count - self.action_start_right);
        self.last_action = datum;
        self.num_updates += 1;
    }

    pub fn get_pos(&self) -> RobotPosition {
        self.pos
    }

    pub fn get_encoder_counts(&self) -> (i64, i64) {
        (self.action_start_left, self.action_start_right)
    }

    pub fn num_updates(&self) -> i64 {
        self.num_updates
    }

    pub fn reset(&mut self) {
        self.action_start_left = 0;
        self.action_start_right = 0;
        self.pos = RobotPosition::new();
        self.num_updates = 0;
    }
}

#[derive(Clone, Debug)]
pub struct RobotPath {
    base: TwoWheelBase,
    points: Vec<MotorData>
}

impl RobotPath {
    pub fn new(base: TwoWheelBase) -> Self {
        RobotPath {points: vec![], base}
    }

    pub fn from_csv(csv_file: &str, base: TwoWheelBase) -> std::io::Result<Self> {
        let mut points = vec![];
        let reader = BufReader::new(File::open(csv_file)?);
        for line in reader.lines().skip(1) {
            let line = line?;
            let mut parts = line.split(",").map(|p| p.trim().parse().unwrap());
            let _sonar_front = parts.next().unwrap();
            let _sonar_left = parts.next().unwrap();
            let _sonar_right = parts.next().unwrap();
            let left_count = parts.next().unwrap();
            let right_count = parts.next().unwrap();
            points.push(MotorData {left_count, right_count, left_speed: 0, right_speed: 0});
        }
        Ok(RobotPath {points, base})
    }

    pub fn add(&mut self, point: MotorData) {
        self.points.push(point);
    }

    pub fn len(&self) -> usize {self.points.len()}

    pub fn position_sequence(&self) -> Vec<RobotPosition> {
        let mut result = vec![RobotPosition::new()];
        for (i, datum) in self.points.iter().enumerate() {
            let (prev_l, prev_r) = if i == 0 {(0, 0)} else {(self.points[i-1].left_count, self.points[i-1].right_count)};
            result.push(self.base.updated_position(*result.last().unwrap(), datum.left_count - prev_l, datum.right_count - prev_r));
        }
        result
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Debug)]
pub struct TwoWheelBase {
    wheel_separation: f64, wheel_radius: f64
}

impl TwoWheelBase {
    pub const fn new(wheel_separation: f64, wheel_radius: f64) -> Self {
        TwoWheelBase {wheel_separation, wheel_radius}
    }

    pub fn wheel_circumference(&self) -> f64 {
        self.wheel_radius * 2.0 * PI
    }

    pub fn wheel_distance_traveled(&self, rotation_counts: i64) -> f64 {
        self.wheel_radius * 2.0 * PI * rotation_counts as f64 / COUNTS_PER_ROTATION
    }

    pub fn updated_position(&self, current_pos: RobotPosition, delta_left: i64, delta_right: i64) -> RobotPosition {
        if delta_left == delta_right {
            let traveled = self.wheel_distance_traveled(delta_left);
            current_pos.updated_by(PolarCoord::new(traveled, 0.0))
        } else {
            let left_arc_length = self.wheel_distance_traveled(delta_left);
            let right_arc_length = self.wheel_distance_traveled(delta_right);
            let left_turn_radius = left_arc_length * self.wheel_separation / (right_arc_length - left_arc_length);
            let right_turn_radius = left_turn_radius + self.wheel_separation;
            let center_turn_radius = (left_turn_radius + right_turn_radius) / 2.0;
            let delta_heading = if left_turn_radius == 0.0 {right_arc_length / right_turn_radius} else {left_arc_length / left_turn_radius};
            let offset = PolarCoord::new(center_turn_radius, delta_heading);
            current_pos + RobotPosition::from(-center_turn_radius + offset.x(), offset.y(), Heading::from_radians(delta_heading))
        }
    }
}

mod tests {
    use std::f64::consts::PI;
    use float_cmp::assert_approx_eq;
    use crate::position_types::Heading;
    use crate::RobotPosition;
    use crate::sonar3bot::*;
    //use crate::sonar3bot::{BOT, COUNTS_PER_ROTATION, EV3_SEPARATION_MODEL_1, EV3_WHEEL_RADIUS, MotorData, RobotPath, RobotSensorPosition, TwoWheelBase};

    #[test]
    fn test_basic_read() {
        let rows_500 = RobotPath::from_csv("office_500_ms.csv", BOT).unwrap();
        assert_eq!(rows_500.len(), 107);
        assert_eq!(format!("{rows_500:?}"), "RobotPath { base: TwoWheelBase { wheel_separation: 10.08, wheel_radius: 2.75 }, points: [MotorData { left_count: 122, right_count: 122, left_speed: 0, right_speed: 0 }, MotorData { left_count: 306, right_count: 305, left_speed: 0, right_speed: 0 }, MotorData { left_count: 490, right_count: 489, left_speed: 0, right_speed: 0 }, MotorData { left_count: 675, right_count: 673, left_speed: 0, right_speed: 0 }, MotorData { left_count: 858, right_count: 857, left_speed: 0, right_speed: 0 }, MotorData { left_count: 1043, right_count: 1042, left_speed: 0, right_speed: 0 }, MotorData { left_count: 1227, right_count: 1226, left_speed: 0, right_speed: 0 }, MotorData { left_count: 1413, right_count: 1412, left_speed: 0, right_speed: 0 }, MotorData { left_count: 1598, right_count: 1597, left_speed: 0, right_speed: 0 }, MotorData { left_count: 1781, right_count: 1780, left_speed: 0, right_speed: 0 }, MotorData { left_count: 1970, right_count: 1969, left_speed: 0, right_speed: 0 }, MotorData { left_count: 2156, right_count: 2155, left_speed: 0, right_speed: 0 }, MotorData { left_count: 2342, right_count: 2342, left_speed: 0, right_speed: 0 }, MotorData { left_count: 2527, right_count: 2526, left_speed: 0, right_speed: 0 }, MotorData { left_count: 2711, right_count: 2710, left_speed: 0, right_speed: 0 }, MotorData { left_count: 2894, right_count: 2893, left_speed: 0, right_speed: 0 }, MotorData { left_count: 3081, right_count: 3081, left_speed: 0, right_speed: 0 }, MotorData { left_count: 3265, right_count: 3264, left_speed: 0, right_speed: 0 }, MotorData { left_count: 3450, right_count: 3450, left_speed: 0, right_speed: 0 }, MotorData { left_count: 3636, right_count: 3635, left_speed: 0, right_speed: 0 }, MotorData { left_count: 3822, right_count: 3822, left_speed: 0, right_speed: 0 }, MotorData { left_count: 4006, right_count: 4006, left_speed: 0, right_speed: 0 }, MotorData { left_count: 4194, right_count: 4193, left_speed: 0, right_speed: 0 }, MotorData { left_count: 4378, right_count: 4377, left_speed: 0, right_speed: 0 }, MotorData { left_count: 4563, right_count: 4563, left_speed: 0, right_speed: 0 }, MotorData { left_count: 4750, right_count: 4750, left_speed: 0, right_speed: 0 }, MotorData { left_count: 4937, right_count: 4936, left_speed: 0, right_speed: 0 }, MotorData { left_count: 5122, right_count: 5122, left_speed: 0, right_speed: 0 }, MotorData { left_count: 5308, right_count: 5307, left_speed: 0, right_speed: 0 }, MotorData { left_count: 5491, right_count: 5490, left_speed: 0, right_speed: 0 }, MotorData { left_count: 5678, right_count: 5677, left_speed: 0, right_speed: 0 }, MotorData { left_count: 5862, right_count: 5862, left_speed: 0, right_speed: 0 }, MotorData { left_count: 6048, right_count: 6047, left_speed: 0, right_speed: 0 }, MotorData { left_count: 6234, right_count: 6232, left_speed: 0, right_speed: 0 }, MotorData { left_count: 6419, right_count: 6265, left_speed: 0, right_speed: 0 }, MotorData { left_count: 6605, right_count: 6372, left_speed: 0, right_speed: 0 }, MotorData { left_count: 6792, right_count: 6558, left_speed: 0, right_speed: 0 }, MotorData { left_count: 6975, right_count: 6669, left_speed: 0, right_speed: 0 }, MotorData { left_count: 7160, right_count: 6533, left_speed: 0, right_speed: 0 }, MotorData { left_count: 7346, right_count: 6664, left_speed: 0, right_speed: 0 }, MotorData { left_count: 7535, right_count: 6853, left_speed: 0, right_speed: 0 }, MotorData { left_count: 7721, right_count: 7038, left_speed: 0, right_speed: 0 }, MotorData { left_count: 7904, right_count: 7221, left_speed: 0, right_speed: 0 }, MotorData { left_count: 8090, right_count: 7408, left_speed: 0, right_speed: 0 }, MotorData { left_count: 8276, right_count: 7556, left_speed: 0, right_speed: 0 }, MotorData { left_count: 8462, right_count: 7445, left_speed: 0, right_speed: 0 }, MotorData { left_count: 8645, right_count: 7557, left_speed: 0, right_speed: 0 }, MotorData { left_count: 8830, right_count: 7742, left_speed: 0, right_speed: 0 }, MotorData { left_count: 9014, right_count: 7926, left_speed: 0, right_speed: 0 }, MotorData { left_count: 9204, right_count: 8116, left_speed: 0, right_speed: 0 }, MotorData { left_count: 9393, right_count: 8305, left_speed: 0, right_speed: 0 }, MotorData { left_count: 9577, right_count: 8489, left_speed: 0, right_speed: 0 }, MotorData { left_count: 9762, right_count: 8674, left_speed: 0, right_speed: 0 }, MotorData { left_count: 9949, right_count: 8861, left_speed: 0, right_speed: 0 }, MotorData { left_count: 10133, right_count: 9045, left_speed: 0, right_speed: 0 }, MotorData { left_count: 10321, right_count: 9232, left_speed: 0, right_speed: 0 }, MotorData { left_count: 10506, right_count: 9418, left_speed: 0, right_speed: 0 }, MotorData { left_count: 10693, right_count: 9605, left_speed: 0, right_speed: 0 }, MotorData { left_count: 10877, right_count: 9789, left_speed: 0, right_speed: 0 }, MotorData { left_count: 11062, right_count: 9974, left_speed: 0, right_speed: 0 }, MotorData { left_count: 11250, right_count: 10162, left_speed: 0, right_speed: 0 }, MotorData { left_count: 11437, right_count: 10348, left_speed: 0, right_speed: 0 }, MotorData { left_count: 11620, right_count: 10532, left_speed: 0, right_speed: 0 }, MotorData { left_count: 11807, right_count: 10718, left_speed: 0, right_speed: 0 }, MotorData { left_count: 11991, right_count: 10903, left_speed: 0, right_speed: 0 }, MotorData { left_count: 12175, right_count: 11087, left_speed: 0, right_speed: 0 }, MotorData { left_count: 12361, right_count: 11272, left_speed: 0, right_speed: 0 }, MotorData { left_count: 12546, right_count: 11457, left_speed: 0, right_speed: 0 }, MotorData { left_count: 12732, right_count: 11644, left_speed: 0, right_speed: 0 }, MotorData { left_count: 12886, right_count: 11827, left_speed: 0, right_speed: 0 }, MotorData { left_count: 12834, right_count: 12016, left_speed: 0, right_speed: 0 }, MotorData { left_count: 13007, right_count: 12202, left_speed: 0, right_speed: 0 }, MotorData { left_count: 13195, right_count: 12390, left_speed: 0, right_speed: 0 }, MotorData { left_count: 13380, right_count: 12576, left_speed: 0, right_speed: 0 }, MotorData { left_count: 13503, right_count: 12762, left_speed: 0, right_speed: 0 }, MotorData { left_count: 13627, right_count: 12945, left_speed: 0, right_speed: 0 }, MotorData { left_count: 13535, right_count: 13130, left_speed: 0, right_speed: 0 }, MotorData { left_count: 13700, right_count: 13321, left_speed: 0, right_speed: 0 }, MotorData { left_count: 13884, right_count: 13506, left_speed: 0, right_speed: 0 }, MotorData { left_count: 14070, right_count: 13691, left_speed: 0, right_speed: 0 }, MotorData { left_count: 14259, right_count: 13880, left_speed: 0, right_speed: 0 }, MotorData { left_count: 14444, right_count: 14065, left_speed: 0, right_speed: 0 }, MotorData { left_count: 14630, right_count: 14250, left_speed: 0, right_speed: 0 }, MotorData { left_count: 14816, right_count: 14436, left_speed: 0, right_speed: 0 }, MotorData { left_count: 15001, right_count: 14620, left_speed: 0, right_speed: 0 }, MotorData { left_count: 15185, right_count: 14806, left_speed: 0, right_speed: 0 }, MotorData { left_count: 15373, right_count: 14993, left_speed: 0, right_speed: 0 }, MotorData { left_count: 15559, right_count: 15180, left_speed: 0, right_speed: 0 }, MotorData { left_count: 15746, right_count: 15367, left_speed: 0, right_speed: 0 }, MotorData { left_count: 15931, right_count: 15552, left_speed: 0, right_speed: 0 }, MotorData { left_count: 16119, right_count: 15740, left_speed: 0, right_speed: 0 }, MotorData { left_count: 16305, right_count: 15926, left_speed: 0, right_speed: 0 }, MotorData { left_count: 16493, right_count: 16112, left_speed: 0, right_speed: 0 }, MotorData { left_count: 16677, right_count: 16297, left_speed: 0, right_speed: 0 }, MotorData { left_count: 16863, right_count: 16483, left_speed: 0, right_speed: 0 }, MotorData { left_count: 17048, right_count: 16668, left_speed: 0, right_speed: 0 }, MotorData { left_count: 17236, right_count: 16856, left_speed: 0, right_speed: 0 }, MotorData { left_count: 17423, right_count: 17044, left_speed: 0, right_speed: 0 }, MotorData { left_count: 17612, right_count: 17233, left_speed: 0, right_speed: 0 }, MotorData { left_count: 17935, right_count: 17333, left_speed: 0, right_speed: 0 }, MotorData { left_count: 18126, right_count: 17320, left_speed: 0, right_speed: 0 }, MotorData { left_count: 18313, right_count: 17508, left_speed: 0, right_speed: 0 }, MotorData { left_count: 18360, right_count: 17692, left_speed: 0, right_speed: 0 }, MotorData { left_count: 18529, right_count: 17880, left_speed: 0, right_speed: 0 }, MotorData { left_count: 18759, right_count: 18131, left_speed: 0, right_speed: 0 }, MotorData { left_count: 18907, right_count: 18318, left_speed: 0, right_speed: 0 }, MotorData { left_count: 19091, right_count: 18503, left_speed: 0, right_speed: 0 }] }");
    }

    #[test]
    fn test_wheel_dist() {
        let bot = TwoWheelBase::new(EV3_SEPARATION_MODEL_1, EV3_WHEEL_RADIUS);
        for (rotations, expected) in [
            (180, EV3_WHEEL_RADIUS * PI),
            (360, EV3_WHEEL_RADIUS * 2.0 * PI),
            (-180, -EV3_WHEEL_RADIUS * PI),
            (270, EV3_WHEEL_RADIUS * 1.5 * PI)
        ] {
            assert_approx_eq!(f64, bot.wheel_distance_traveled(rotations), expected);
        }

        // Actual measurement
        assert_approx_eq!(f64, bot.wheel_distance_traveled(3602), 172.0, epsilon = 1.0);
    }

    fn test_straight(counts: i64) {
        let bot = TwoWheelBase::new(EV3_SEPARATION_MODEL_1, EV3_WHEEL_RADIUS);
        let end = bot.updated_position(RobotPosition::new(), counts, counts);
        assert_approx_eq!(RobotPosition, end, RobotPosition::from(counts as f64 / COUNTS_PER_ROTATION * EV3_WHEEL_RADIUS * 2.0 * PI, 0.0, Heading::new(0)));
    }

    #[test]
    fn test_straight_forward() {
        test_straight(100);
    }

    #[test]
    fn test_straight_back() {
        test_straight(-100);
    }

    #[test]
    fn test_turn() {
        let bot = TwoWheelBase::new(EV3_SEPARATION_MODEL_1, EV3_WHEEL_RADIUS);
        let turn_circumference = bot.wheel_separation * 2.0 * PI;
        let travel_distance = turn_circumference / 4.0;
        let mut turn_rotations = (travel_distance / bot.wheel_circumference() * COUNTS_PER_ROTATION) as i64;
        turn_rotations += 1; // Rounding error adjustment

        let expected = 5.04;

        let end = bot.updated_position(RobotPosition::new(), 0, turn_rotations);
        assert_approx_eq!(RobotPosition, end, RobotPosition::from(-expected, expected, Heading::new(90)), epsilon = 0.01);

        let end = bot.updated_position(RobotPosition::new(), 0, -turn_rotations);
        assert_approx_eq!(RobotPosition, end, RobotPosition::from(-expected, -expected, Heading::new(-90)), epsilon = 0.01);

        let end = bot.updated_position(RobotPosition::new(), turn_rotations, 0);
        assert_approx_eq!(RobotPosition, end, RobotPosition::from(expected, expected, Heading::new(-90)), epsilon = 0.01);

        let end = bot.updated_position(RobotPosition::new(), -turn_rotations, 0);
        assert_approx_eq!(RobotPosition, end, RobotPosition::from(expected, -expected, Heading::new(90)), epsilon = 0.01);
    }

    #[test]
    fn test_spin() {
        let spin_circumference = BOT.wheel_separation * PI;
        let travel_distance = spin_circumference / 4.0;
        let mut spin_rotations = (travel_distance / BOT.wheel_circumference() * COUNTS_PER_ROTATION) as i64;
        spin_rotations += 1; // Rounding error adjustment

        assert_eq!(spin_rotations, 165);

        let end = BOT.updated_position(RobotPosition::new(), -spin_rotations, spin_rotations);
        assert_approx_eq!(RobotPosition, end, RobotPosition::from(0.0, 0.0, Heading::new(90)));

        let end = BOT.updated_position(RobotPosition::new(), spin_rotations, -spin_rotations);
        assert_approx_eq!(RobotPosition, end, RobotPosition::from(0.0, 0.0, Heading::new(-90)));
    }

    #[test]
    fn view_trail() {
        let rows_500 = RobotPath::from_csv("office_500_ms.csv", BOT).unwrap();
        let positions = rows_500.position_sequence();
        println!("start:{:?} end:{:?}", positions.first().unwrap(), positions.last().unwrap());
    }

    #[test]
    fn test_robot_sensor_position() {
        let mut r = RobotSensorPosition::new(BOT);
        let s = MotorData {
            left_count: 3084,
            right_count: 3085,
            left_speed: 0,
            right_speed: 0
        };
        r.motor_update(s);
        println!("r: {r:?}");
        let s = MotorData {
            left_count: 3084 + 271,
            right_count: 3085 - 261,
            left_speed: 0,
            right_speed: 0
        };
        r.motor_update(s);
        println!("r: {r:?}");
        // TODO: Write a test here.
        assert!(false)
    }
}