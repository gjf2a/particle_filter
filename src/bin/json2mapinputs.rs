use std::env;
use particle_filter::ParticleFilter;

fn main() -> anyhow::Result<()> {
    let args = env::args().collect::<Vec<_>>();
    if args.len() == 1 {
        println!("Usage: json2mapinputs filename.json ...");
    } else {
        for arg in args.iter() {
            if let Some(suffix) = arg.find(".json") {
                let text = std::fs::read_to_string(arg)?;
                let particle_filter = serde_json::from_str::<ParticleFilter>(&text)?;
                if let Some(inputs) = particle_filter.inputs() {
                    let inputs = inputs.iter().map(|input| format!("{input}")).collect::<Vec<_>>();
                    let filename_prefix = &arg[..suffix];
                    let output_filename = format!("{filename_prefix}.out");
                    std::fs::write(&output_filename, inputs.join("\n"))?;
                    println!("Wrote to {output_filename}");
                }
            }
        }
    }
    Ok(())
}