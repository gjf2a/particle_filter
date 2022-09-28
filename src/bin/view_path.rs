use minifb::{MouseMode, Window, WindowOptions, ScaleMode, Scale};
use raqote::{DrawTarget, SolidSource, Source, DrawOptions, PathBuilder, Point, Transform, StrokeStyle};
use font_kit::family_name::FamilyName;
use font_kit::properties::Properties;
use font_kit::source::SystemSource;
use particle_filter::position_types::PositionBounds;

const SIZE: usize = 400;

use particle_filter::sonar3bot::*;

fn main() {
    let data = RobotPath::from_csv("office_500_ms.csv", TwoWheelBase::new(EV3_SEPARATION_MODEL_1, EV3_WHEEL_RADIUS)).unwrap();
    let positions = data.position_sequence();
    let bounds = PositionBounds::from(&positions);

    let mut window = Window::new("Raqote", SIZE, SIZE, WindowOptions {
        ..WindowOptions::default()
    }).unwrap();
    let font = SystemSource::new()
        .select_best_match(&[FamilyName::SansSerif], &Properties::new())
        .unwrap()
        .load()
        .unwrap();

    let size = window.get_size();
    let mut dt = DrawTarget::new(size.0 as i32, size.1 as i32);
    loop {
        if !window.is_open() {
            break;
        }
        dt.clear(SolidSource::from_unpremultiplied_argb(0xff, 0xff, 0xff, 0xff));
        let mut pb = PathBuilder::new();
        for pos in positions.iter() {
            let (x, y) = pos.position();
            let (x, y) = scale2pixel(&bounds, x, y, SIZE);
            pb.rect(x, y, 10.0, 10.0);
        }
        let path = pb.finish();
        dt.fill(&path, &Source::Solid(SolidSource::from_unpremultiplied_argb(0xff, 0, 0xff, 0)), &DrawOptions::new());
        window.update_with_buffer(dt.get_data(), size.0, size.1).unwrap();
    }
}


pub fn scale2pixel(bounds: &PositionBounds, x: f64, y: f64, side: usize) -> (f32, f32) {
    ((side as f64 * (x - bounds.min_x()) / bounds.max_bound()) as f32,
     (side as f64 * (y - bounds.min_y()) / bounds.max_bound()) as f32)
}