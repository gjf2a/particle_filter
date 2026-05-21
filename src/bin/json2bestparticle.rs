use particle_filter::ParticleFilter;
use std::env;

fn main() -> anyhow::Result<()> {
    let args = env::args().collect::<Vec<_>>();
    if args.len() == 1 {
        println!("Usage: json2mapinputs filename.json ...");
    } else {
        for arg in args.iter() {
            if let Some(_) = arg.find(".json") {
                let text = std::fs::read_to_string(arg)?;
                let particle_filter = serde_json::from_str::<ParticleFilter>(&text)?;
                let best_particle = particle_filter.particles().next().unwrap();
                let best_particle_json = serde_json::to_string(best_particle)?;
                println!("{arg}");
                println!("{best_particle_json}");
            }
        }
    }
    Ok(())
}