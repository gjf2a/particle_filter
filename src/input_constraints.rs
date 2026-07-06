use z3::{Solver, ast::Float};

use crate::{MapInput, Noises, angle::Polar};

pub fn input_constraints_from(inputs: &Vec<MapInput>, noises: Noises) -> Solver {
    // Figure out a data structure for the constraints
    // The Z3 constraints will be derived from it.
    //
    // Sketch of the idea:
    // * The first robot position is fixed.
    // * Later positions are expressed as dependencies on earlier positions, bounded by x noise and y noise.
    // * Each obstacle has a constraint for every robot position.
    //   * The obstacle cannot be closer to the robot position than the robot's radius.

    let mut positions = vec![];
    let mut pxs = vec![];
    let mut pys = vec![];
    let mut pts = vec![];
    let mut obstacles = vec![];
    let mut obstacle_vars = vec![];
    let solver = Solver::new();
    for input in inputs.iter() {
        if let Some(pose) = input.pose() {
            let x = Float::fresh_const_double(&format!("space_{}_x", positions.len()));
            let y = Float::fresh_const_double(&format!("space_{}_y", positions.len()));
            let t = Float::fresh_const_double(&format!("space_{}_theta", positions.len()));
            if let Some(prev) = positions.last() {
                let diff = pose - *prev;
                // From here, add assertions that each dimension of the position has a lower and upper bound:
                // * previous variable plus diff minus noise
                // * previous variable plus diff plus noise
            } else {
                solver.assert(x.eq(pose.pos[0]));
                solver.assert(y.eq(pose.pos[1]));
                solver.assert(t.eq(f64::from(pose.theta)));
            }
            positions.push(pose);
            pxs.push(x);
            pys.push(y);
            pts.push(t);
        } 
        if let Some(obstacle) = input.obstacle() {
            if let Some(position) = positions.last() {
                obstacles.push(position.pos + Polar::new(obstacle.distance(), obstacle.heading()));
                obstacle_vars.push(Float::fresh_const_double(format!("obstacle_{}_x", positions.len()).as_str()));
                obstacle_vars.push(Float::fresh_const_double(format!("obstacle_{}_y", positions.len()).as_str()));
            }
        }
    }
    solver
}