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
pub struct SensorData {
    pub sonar_front: i64, pub sonar_left: i64, pub sonar_right: i64, pub motor_left: i64, pub motor_right: i64, pub action_tag: i64
}

impl SensorData {
    pub fn new(sonar_front: i64, sonar_left: i64, sonar_right: i64, motor_left: i64, motor_right: i64, action_tag: i64) -> Self {
        SensorData {
            sonar_front,
            sonar_left,
            sonar_right,
            motor_left,
            motor_right,
            action_tag
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct RobotSensorPosition {
    base: TwoWheelBase,
    action_start_left: i64, action_start_right: i64, action_start_pos: RobotPosition,
    last_left: i64, last_right: i64, pos: RobotPosition,
    num_updates: i64,
    action_tag: i64
}

impl RobotSensorPosition {
    pub fn new(base: TwoWheelBase) -> Self {
        RobotSensorPosition {
            base,
            action_start_left: 0,
            action_start_right: 0,
            action_start_pos: RobotPosition::new(),
            last_left: 0,
            last_right: 0,
            pos: RobotPosition::new(),
            num_updates: 0,
            action_tag: 0
        }
    }

    pub fn update(&mut self, datum: SensorData) {
        if datum.action_tag != self.action_tag {
            self.action_start_pos = self.pos;
            self.action_tag = datum.action_tag;
            self.action_start_left = self.last_left;
            self.action_start_right = self.last_right;
        }
        self.pos = self.base.updated_position(self.action_start_pos,
                                              datum.motor_left - self.action_start_left,
                                              datum.motor_right - self.action_start_right);
        self.last_left = datum.motor_left;
        self.last_right = datum.motor_right;
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
    points: Vec<SensorData>
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
            let sonar_front = parts.next().unwrap();
            let sonar_left = parts.next().unwrap();
            let sonar_right = parts.next().unwrap();
            let motor_left = parts.next().unwrap();
            let motor_right = parts.next().unwrap();
            points.push(SensorData {sonar_front, sonar_left, sonar_right, motor_left, motor_right, action_tag: 0});
        }
        Ok(RobotPath {points, base})
    }

    pub fn add(&mut self, point: SensorData) {
        self.points.push(point);
    }

    pub fn len(&self) -> usize {self.points.len()}

    pub fn position_sequence(&self) -> Vec<RobotPosition> {
        let mut result = vec![RobotPosition::new()];
        for (i, datum) in self.points.iter().enumerate() {
            let (prev_l, prev_r) = if i == 0 {(0, 0)} else {(self.points[i-1].motor_left, self.points[i-1].motor_right)};
            result.push(self.base.updated_position(*result.last().unwrap(), datum.motor_left - prev_l, datum.motor_right - prev_r));
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
    use crate::sonar3bot::{BOT, COUNTS_PER_ROTATION, EV3_SEPARATION_MODEL_1, EV3_WHEEL_RADIUS, RobotPath, RobotSensorPosition, SensorData, TwoWheelBase};

    #[test]
    fn test_basic_read() {
        let rows_500 = RobotPath::from_csv("office_500_ms.csv", BOT).unwrap();
        assert_eq!(rows_500.len(), 107);
        assert_eq!(format!("{:?}", rows_500), "RobotPath { base: TwoWheelBase { wheel_separation: 10.08, wheel_radius: 2.75 }, points: [SensorData { sonar_front: 2550, sonar_left: 2550, sonar_right: 2550, motor_left: 122, motor_right: 122 }, SensorData { sonar_front: 1805, sonar_left: 2550, sonar_right: 2550, motor_left: 306, motor_right: 305 }, SensorData { sonar_front: 1343, sonar_left: 2550, sonar_right: 2550, motor_left: 490, motor_right: 489 }, SensorData { sonar_front: 1297, sonar_left: 2550, sonar_right: 2550, motor_left: 675, motor_right: 673 }, SensorData { sonar_front: 1550, sonar_left: 2550, sonar_right: 2550, motor_left: 858, motor_right: 857 }, SensorData { sonar_front: 1474, sonar_left: 2550, sonar_right: 2550, motor_left: 1043, motor_right: 1042 }, SensorData { sonar_front: 1379, sonar_left: 2550, sonar_right: 530, motor_left: 1227, motor_right: 1226 }, SensorData { sonar_front: 792, sonar_left: 2550, sonar_right: 500, motor_left: 1413, motor_right: 1412 }, SensorData { sonar_front: 1278, sonar_left: 2550, sonar_right: 2550, motor_left: 1598, motor_right: 1597 }, SensorData { sonar_front: 1160, sonar_left: 2550, sonar_right: 2550, motor_left: 1781, motor_right: 1780 }, SensorData { sonar_front: 756, sonar_left: 2550, sonar_right: 2550, motor_left: 1970, motor_right: 1969 }, SensorData { sonar_front: 1106, sonar_left: 2550, sonar_right: 2550, motor_left: 2156, motor_right: 2155 }, SensorData { sonar_front: 954, sonar_left: 2550, sonar_right: 2550, motor_left: 2342, motor_right: 2342 }, SensorData { sonar_front: 2550, sonar_left: 2550, sonar_right: 2550, motor_left: 2527, motor_right: 2526 }, SensorData { sonar_front: 1273, sonar_left: 2550, sonar_right: 410, motor_left: 2711, motor_right: 2710 }, SensorData { sonar_front: 1101, sonar_left: 2550, sonar_right: 450, motor_left: 2894, motor_right: 2893 }, SensorData { sonar_front: 1628, sonar_left: 2550, sonar_right: 430, motor_left: 3081, motor_right: 3081 }, SensorData { sonar_front: 2550, sonar_left: 2550, sonar_right: 410, motor_left: 3265, motor_right: 3264 }, SensorData { sonar_front: 1513, sonar_left: 2550, sonar_right: 2550, motor_left: 3450, motor_right: 3450 }, SensorData { sonar_front: 1404, sonar_left: 2550, sonar_right: 2550, motor_left: 3636, motor_right: 3635 }, SensorData { sonar_front: 947, sonar_left: 2550, sonar_right: 430, motor_left: 3822, motor_right: 3822 }, SensorData { sonar_front: 1458, sonar_left: 2550, sonar_right: 430, motor_left: 4006, motor_right: 4006 }, SensorData { sonar_front: 1301, sonar_left: 2550, sonar_right: 2550, motor_left: 4194, motor_right: 4193 }, SensorData { sonar_front: 1189, sonar_left: 2550, sonar_right: 2550, motor_left: 4378, motor_right: 4377 }, SensorData { sonar_front: 1111, sonar_left: 2550, sonar_right: 2550, motor_left: 4563, motor_right: 4563 }, SensorData { sonar_front: 1031, sonar_left: 880, sonar_right: 2550, motor_left: 4750, motor_right: 4750 }, SensorData { sonar_front: 968, sonar_left: 820, sonar_right: 690, motor_left: 4937, motor_right: 4936 }, SensorData { sonar_front: 968, sonar_left: 1030, sonar_right: 2550, motor_left: 5122, motor_right: 5122 }, SensorData { sonar_front: 754, sonar_left: 970, sonar_right: 2550, motor_left: 5308, motor_right: 5307 }, SensorData { sonar_front: 679, sonar_left: 640, sonar_right: 2550, motor_left: 5491, motor_right: 5490 }, SensorData { sonar_front: 573, sonar_left: 550, sonar_right: 2550, motor_left: 5678, motor_right: 5677 }, SensorData { sonar_front: 506, sonar_left: 450, sonar_right: 560, motor_left: 5862, motor_right: 5862 }, SensorData { sonar_front: 399, sonar_left: 370, sonar_right: 680, motor_left: 6048, motor_right: 6047 }, SensorData { sonar_front: 326, sonar_left: 290, sonar_right: 580, motor_left: 6234, motor_right: 6232 }, SensorData { sonar_front: 326, sonar_left: 210, sonar_right: 450, motor_left: 6419, motor_right: 6265 }, SensorData { sonar_front: 445, sonar_left: 210, sonar_right: 420, motor_left: 6605, motor_right: 6372 }, SensorData { sonar_front: 364, sonar_left: 200, sonar_right: 320, motor_left: 6792, motor_right: 6558 }, SensorData { sonar_front: 265, sonar_left: 200, sonar_right: 250, motor_left: 6975, motor_right: 6669 }, SensorData { sonar_front: 340, sonar_left: 220, sonar_right: 2550, motor_left: 7160, motor_right: 6533 }, SensorData { sonar_front: 740, sonar_left: 230, sonar_right: 2550, motor_left: 7346, motor_right: 6664 }, SensorData { sonar_front: 326, sonar_left: 220, sonar_right: 2550, motor_left: 7535, motor_right: 6853 }, SensorData { sonar_front: 542, sonar_left: 210, sonar_right: 2550, motor_left: 7721, motor_right: 7038 }, SensorData { sonar_front: 446, sonar_left: 200, sonar_right: 2550, motor_left: 7904, motor_right: 7221 }, SensorData { sonar_front: 353, sonar_left: 200, sonar_right: 700, motor_left: 8090, motor_right: 7408 }, SensorData { sonar_front: 265, sonar_left: 200, sonar_right: 2550, motor_left: 8276, motor_right: 7556 }, SensorData { sonar_front: 2550, sonar_left: 220, sonar_right: 2550, motor_left: 8462, motor_right: 7445 }, SensorData { sonar_front: 2550, sonar_left: 2550, sonar_right: 2550, motor_left: 8645, motor_right: 7557 }, SensorData { sonar_front: 1044, sonar_left: 2550, sonar_right: 1110, motor_left: 8830, motor_right: 7742 }, SensorData { sonar_front: 2335, sonar_left: 2550, sonar_right: 1070, motor_left: 9014, motor_right: 7926 }, SensorData { sonar_front: 1406, sonar_left: 2550, sonar_right: 1070, motor_left: 9204, motor_right: 8116 }, SensorData { sonar_front: 1450, sonar_left: 2550, sonar_right: 1310, motor_left: 9393, motor_right: 8305 }, SensorData { sonar_front: 1589, sonar_left: 2550, sonar_right: 2550, motor_left: 9577, motor_right: 8489 }, SensorData { sonar_front: 2550, sonar_left: 2550, sonar_right: 2550, motor_left: 9762, motor_right: 8674 }, SensorData { sonar_front: 1217, sonar_left: 2550, sonar_right: 2550, motor_left: 9949, motor_right: 8861 }, SensorData { sonar_front: 1680, sonar_left: 2550, sonar_right: 2550, motor_left: 10133, motor_right: 9045 }, SensorData { sonar_front: 1597, sonar_left: 2550, sonar_right: 2550, motor_left: 10321, motor_right: 9232 }, SensorData { sonar_front: 993, sonar_left: 2550, sonar_right: 2550, motor_left: 10506, motor_right: 9418 }, SensorData { sonar_front: 1337, sonar_left: 2550, sonar_right: 2550, motor_left: 10693, motor_right: 9605 }, SensorData { sonar_front: 1263, sonar_left: 2550, sonar_right: 2550, motor_left: 10877, motor_right: 9789 }, SensorData { sonar_front: 994, sonar_left: 2550, sonar_right: 2550, motor_left: 11062, motor_right: 9974 }, SensorData { sonar_front: 1062, sonar_left: 2550, sonar_right: 2550, motor_left: 11250, motor_right: 10162 }, SensorData { sonar_front: 973, sonar_left: 2550, sonar_right: 2550, motor_left: 11437, motor_right: 10348 }, SensorData { sonar_front: 916, sonar_left: 2550, sonar_right: 1230, motor_left: 11620, motor_right: 10532 }, SensorData { sonar_front: 774, sonar_left: 2550, sonar_right: 1170, motor_left: 11807, motor_right: 10718 }, SensorData { sonar_front: 696, sonar_left: 2550, sonar_right: 2550, motor_left: 11991, motor_right: 10903 }, SensorData { sonar_front: 345, sonar_left: 2550, sonar_right: 2550, motor_left: 12175, motor_right: 11087 }, SensorData { sonar_front: 525, sonar_left: 2550, sonar_right: 2550, motor_left: 12361, motor_right: 11272 }, SensorData { sonar_front: 435, sonar_left: 2550, sonar_right: 2550, motor_left: 12546, motor_right: 11457 }, SensorData { sonar_front: 347, sonar_left: 2550, sonar_right: 2550, motor_left: 12732, motor_right: 11644 }, SensorData { sonar_front: 263, sonar_left: 2550, sonar_right: 290, motor_left: 12886, motor_right: 11827 }, SensorData { sonar_front: 1189, sonar_left: 2550, sonar_right: 220, motor_left: 12834, motor_right: 12016 }, SensorData { sonar_front: 757, sonar_left: 2550, sonar_right: 210, motor_left: 13007, motor_right: 12202 }, SensorData { sonar_front: 496, sonar_left: 2550, sonar_right: 210, motor_left: 13195, motor_right: 12390 }, SensorData { sonar_front: 433, sonar_left: 2550, sonar_right: 110, motor_left: 13380, motor_right: 12576 }, SensorData { sonar_front: 326, sonar_left: 2550, sonar_right: 220, motor_left: 13503, motor_right: 12762 }, SensorData { sonar_front: 326, sonar_left: 2550, sonar_right: 50, motor_left: 13627, motor_right: 12945 }, SensorData { sonar_front: 2550, sonar_left: 2550, sonar_right: 2550, motor_left: 13535, motor_right: 13130 }, SensorData { sonar_front: 849, sonar_left: 2550, sonar_right: 2550, motor_left: 13700, motor_right: 13321 }, SensorData { sonar_front: 929, sonar_left: 2550, sonar_right: 2550, motor_left: 13884, motor_right: 13506 }, SensorData { sonar_front: 2550, sonar_left: 2550, sonar_right: 2550, motor_left: 14070, motor_right: 13691 }, SensorData { sonar_front: 721, sonar_left: 2550, sonar_right: 2550, motor_left: 14259, motor_right: 13880 }, SensorData { sonar_front: 849, sonar_left: 2550, sonar_right: 2550, motor_left: 14444, motor_right: 14065 }, SensorData { sonar_front: 2550, sonar_left: 2550, sonar_right: 2550, motor_left: 14630, motor_right: 14250 }, SensorData { sonar_front: 2550, sonar_left: 2550, sonar_right: 2550, motor_left: 14816, motor_right: 14436 }, SensorData { sonar_front: 1577, sonar_left: 2550, sonar_right: 2550, motor_left: 15001, motor_right: 14620 }, SensorData { sonar_front: 2550, sonar_left: 2550, sonar_right: 2550, motor_left: 15185, motor_right: 14806 }, SensorData { sonar_front: 2211, sonar_left: 2550, sonar_right: 2550, motor_left: 15373, motor_right: 14993 }, SensorData { sonar_front: 2205, sonar_left: 2550, sonar_right: 2550, motor_left: 15559, motor_right: 15180 }, SensorData { sonar_front: 2033, sonar_left: 2550, sonar_right: 2550, motor_left: 15746, motor_right: 15367 }, SensorData { sonar_front: 2122, sonar_left: 2550, sonar_right: 2550, motor_left: 15931, motor_right: 15552 }, SensorData { sonar_front: 853, sonar_left: 2550, sonar_right: 2550, motor_left: 16119, motor_right: 15740 }, SensorData { sonar_front: 1201, sonar_left: 1030, sonar_right: 2550, motor_left: 16305, motor_right: 15926 }, SensorData { sonar_front: 1092, sonar_left: 980, sonar_right: 2550, motor_left: 16493, motor_right: 16112 }, SensorData { sonar_front: 1839, sonar_left: 980, sonar_right: 2550, motor_left: 16677, motor_right: 16297 }, SensorData { sonar_front: 1509, sonar_left: 880, sonar_right: 2550, motor_left: 16863, motor_right: 16483 }, SensorData { sonar_front: 960, sonar_left: 850, sonar_right: 2550, motor_left: 17048, motor_right: 16668 }, SensorData { sonar_front: 1198, sonar_left: 770, sonar_right: 2550, motor_left: 17236, motor_right: 16856 }, SensorData { sonar_front: 1231, sonar_left: 690, sonar_right: 2550, motor_left: 17423, motor_right: 17044 }, SensorData { sonar_front: 1000, sonar_left: 640, sonar_right: 2550, motor_left: 17612, motor_right: 17233 }, SensorData { sonar_front: 1022, sonar_left: 840, sonar_right: 2550, motor_left: 17935, motor_right: 17333 }, SensorData { sonar_front: 536, sonar_left: 2550, sonar_right: 2550, motor_left: 18126, motor_right: 17320 }, SensorData { sonar_front: 469, sonar_left: 2550, sonar_right: 250, motor_left: 18313, motor_right: 17508 }, SensorData { sonar_front: 420, sonar_left: 2550, sonar_right: 240, motor_left: 18360, motor_right: 17692 }, SensorData { sonar_front: 833, sonar_left: 2550, sonar_right: 310, motor_left: 18529, motor_right: 17880 }, SensorData { sonar_front: 483, sonar_left: 2550, sonar_right: 230, motor_left: 18759, motor_right: 18131 }, SensorData { sonar_front: 687, sonar_left: 2550, sonar_right: 2550, motor_left: 18907, motor_right: 18318 }, SensorData { sonar_front: 546, sonar_left: 2550, sonar_right: 2550, motor_left: 19091, motor_right: 18503 }] }");
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
        let s = SensorData {
            sonar_front: 0,
            sonar_left: 0,
            sonar_right: 0,
            motor_left: 3084,
            motor_right: 3085,
            action_tag: 0
        };
        r.update(s);
        println!("r: {r:?}");
        let s = SensorData {
            sonar_front: 0,
            sonar_left: 0,
            sonar_right: 0,
            motor_left: 3084 + 271,
            motor_right: 3085 - 261,
            action_tag: 0
        };
        r.update(s);
        println!("r: {r:?}");
        // TODO: Write a test here.
        assert!(false)
    }
}