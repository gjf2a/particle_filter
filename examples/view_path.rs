use minifb::{Window, WindowOptions};
use raqote::{DrawTarget, SolidSource, Source, DrawOptions, PathBuilder};
use particle_filter::position_types::PositionBounds;
use particle_filter::RobotPosition;

const SIZE: usize = 400;

use particle_filter::sonar3bot::*;

fn main() {
    let data = RobotPath::from_csv("office_500_ms.csv", TwoWheelBase::new(EV3_SEPARATION_MODEL_1, EV3_WHEEL_RADIUS)).unwrap();
    let positions = data.position_sequence();
    let bounds = PositionBounds::from(&positions);

    let mut window = Window::new("Raqote", SIZE, SIZE, WindowOptions {
        ..WindowOptions::default()
    }).unwrap();

    let size = window.get_size();
    let mut dt = DrawTarget::new(size.0 as i32, size.1 as i32);
    loop {
        if !window.is_open() {
            break;
        }
        dt.clear(SolidSource::from_unpremultiplied_argb(0xff, 0xff, 0xff, 0xff));
        let mut pb = PathBuilder::new();
        for pos in positions.iter() {
            plot(pos, &mut pb, &bounds);
        }
        let path = pb.finish();
        dt.fill(&path, &Source::Solid(SolidSource::from_unpremultiplied_argb(0xff, 0, 0xff, 0)), &DrawOptions::new());

        let mut pb = PathBuilder::new();
        plot(positions.first().unwrap(), &mut pb, &bounds);
        let path = pb.finish();
        dt.fill(&path, &Source::Solid(SolidSource::from_unpremultiplied_argb(0xff, 0xff, 0, 0)), &DrawOptions::new());

        let mut pb = PathBuilder::new();
        plot(positions.last().unwrap(), &mut pb, &bounds);
        let path = pb.finish();
        dt.fill(&path, &Source::Solid(SolidSource::from_unpremultiplied_argb(0xff, 0, 0, 0xff)), &DrawOptions::new());

        window.update_with_buffer(dt.get_data(), size.0, size.1).unwrap();
    }
}

pub fn plot(pos: &RobotPosition, pb: &mut PathBuilder, bounds: &PositionBounds) {
    let (x, y) = pos.position();
    let (x, y) = scale2pixel(&bounds, x, y, SIZE);
    pb.rect(x, y, 10.0, 10.0);
}

pub fn scale2pixel(bounds: &PositionBounds, x: f64, y: f64, side: usize) -> (f32, f32) {
    let buffer = 0.1;
    let bufpix = buffer * side as f64 / 2.0;
    let side = side as f64 * (1.0 - buffer);
    ((side * (x - bounds.min_x() + bufpix) / bounds.max_bound()) as f32,
     (side * (y - bounds.min_y() + bufpix) / bounds.max_bound()) as f32)
}