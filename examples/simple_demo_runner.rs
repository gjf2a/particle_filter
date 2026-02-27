use particle_filter::{
    Degrees, Noise, ParticleFilter,
    simple_demo::{CircleSimulator, DummyMap, Move},
};

const RADIUS: f64 = 100.0;

fn main() {
    let noise = Noise {
        stdev_x_y: 0.01,
        stdev_angle: Degrees::new(1.0),
    };
    let mut demo = CircleSimulator::new(
        vec![Move::Forward, Move::Left, Move::Forward, Move::Right],
        RADIUS,
        noise,
    );
    let starting_map = DummyMap::new(RADIUS, noise);
    let mut particle_filter = ParticleFilter::new(10000, &starting_map);

    for i in 0..100 {
        println!("Step {i}");
        if let Some(distance) = demo.tick_sensor() {
            particle_filter.iterate(Some(demo.odometry_pose()), Some(&distance));
            println!("actual:   {}", demo.ground_truth_pose());
            println!(
                "estimate: {}",
                particle_filter.current_best().estimated_pose()
            );
            println!("odometry: {}", demo.odometry_pose());
        }
    }
}
