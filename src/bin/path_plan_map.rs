use particle_filter::{
    Particle,
    clear_path_graph::{self, ClearPathGraph},
};

fn main() -> anyhow::Result<()> {
    let filename = std::env::args().skip(1).next().unwrap();
    let particle =
        serde_json::from_str::<Particle>(std::fs::read_to_string(filename)?.as_str())?;
    let start = particle.map().to_point(particle.estimated_pose().pos);
    println!("start: {start}");
    let cpg = ClearPathGraph::corner_graph(particle.map());
    println!("Created corner graph");
    print_graph_info(&cpg);
    match ClearPathGraph::corner_current_graph(particle.map(), start) {
        None => println!("Couldn't create corner current graph"),
        Some(cpg) => {
            println!("Created corner current graph; {}", cpg.is_connected());
            print_graph_info(&cpg);
            match ClearPathGraph::corner_current_frontier_graph(particle.map(), start) {
                None => println!("Couldn't create corner current frontier graph"),
                Some(cpg) => {
                    println!(
                        "Created corner current frontier graph: {}",
                        cpg.is_connected()
                    );
                    print_graph_info(&cpg);
                    match clear_path_graph::ClearPathGraph::longest_frontier_path(
                        particle.map(),
                        start,
                    ) {
                        None => {
                            println!("Sorry, couldn't find a path.");
                        }
                        Some(path) => {
                            println!("{path:?}");
                        }
                    }
                }
            }
        }
    }

    Ok(())
}

fn print_graph_info(graph: &ClearPathGraph) {
    println!("Is it connected? {}", graph.is_connected());
    println!("# nodes: {}", graph.vertices().count());
    let edges = graph
        .vertices()
        .map(|v| graph.edges_from(v).count())
        .sum::<usize>();
    println!("# edges: {edges}");
}
