#![feature(let_chains)]
#![feature(btree_drain_filter)]

use chumsky::Parser;
use cim::{
    /*cim_finder::CimNeighborFinder, */ neighbor_finder::NeighborFinder, particles::ID,
    simple_finder::SimpleNeighborFinder,
};
use gear_predictor_corrector::{GearCorrector, GearPredictor};
use std::{
    collections::{BTreeMap, HashMap},
    fs::{self, File},
    io::{stdout, Write},
};

use nalgebra::Vector2;
use pool::{
    models::{Ball, InputData as SimpleInputData, IterableFrame},
    parser::input_parser,
    Float, HOLE_POSITIONS,
};

use clap::{Args, Parser as _parser, Subcommand};

#[derive(Subcommand, Debug)]
#[clap(rename_all = "kebab_case")]
enum OutputCondition {
    Every(EveryArgs),
    WhenBallCountHits { counts: Vec<usize> },
}

#[derive(Debug, Args)]
struct EveryArgs {
    steps: u64,
    #[arg(long)]
    last: bool,
}

#[derive(clap::Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Arguments {
    #[arg(short, long)]
    input: String,

    #[arg(short, long)]
    output: Option<String>,

    #[arg(short, long)]
    delta_time_n: u16,

    #[command(subcommand)]
    output_condition: OutputCondition,

    #[arg(short, long)]
    with_holes: bool,

    #[arg(short, long)]
    max_duration: Option<f64>,

    #[arg(long)]
    min_ball_amount: Option<usize>,
}

struct InputData {
    simple_input_data: SimpleInputData,
    output_condition: OutputCondition,
    delta_time_n: u16,
    with_holes: bool,
}

#[derive(Debug, Copy, Clone)]
enum Wall {
    Left,
    Right,
    Top,
    Bottom,
}

const K: Float = 10e4 * 1000.0;

fn did_ball_go_outside(ball: &Ball, config: &InputData) -> Vec<Wall> {
    let mut collisions = Vec::with_capacity(2);
    let radius = config.simple_input_data.ball_radius;

    if ball.position.x - radius < 0.0 {
        collisions.push(Wall::Left)
    } else if ball.position.x + radius > config.simple_input_data.table_width {
        collisions.push(Wall::Right)
    }

    if ball.position.y - radius < 0.0 {
        collisions.push(Wall::Bottom)
    } else if ball.position.y + radius > config.simple_input_data.table_height {
        collisions.push(Wall::Top)
    }

    collisions
}

fn calculate_force(b: &Ball, other: &Ball, radius_sum: Float) -> Vector2<Float> {
    let r_hat = (other.position - b.position).normalize();
    K * ((b.position - other.position).magnitude() - radius_sum) * r_hat
}

trait PredictorFromBall: Sized {
    fn from_ball(
        ball: &Ball,
        r2: Vector2<f64>,
        r3: Vector2<f64>,
        r4: Vector2<f64>,
        r5: Vector2<f64>,
    ) -> Self;
}

impl PredictorFromBall for GearPredictor<Vector2<f64>> {
    fn from_ball(
        ball: &Ball,
        r2: Vector2<f64>,
        r3: Vector2<f64>,
        r4: Vector2<f64>,
        r5: Vector2<f64>,
    ) -> Self {
        Self {
            rs: [ball.position, ball.velocity, r2, r3, r4, r5],
        }
    }
}

fn run<W: Write, F: FnMut(&BTreeMap<ID, (Ball, [Vector2<f64>; 4])>, Float) -> bool>(
    config: InputData,
    mut output_writer: W,
    mut stop_condition: F,
) {
    let mut time = 0.0;
    let mut state: BTreeMap<_, _> = config
        .simple_input_data
        .balls
        .iter()
        .copied()
        .map(|p| (p.id, (p, [Vector2::zeros(); 4])))
        .collect();

    let holes = if config.with_holes {
        HOLE_POSITIONS
            .map(|v| {
                v.component_mul(&Vector2::new(
                    config.simple_input_data.table_width,
                    config.simple_input_data.table_height,
                ))
            })
            .to_vec()
    } else {
        vec![]
    };

    if match &config.output_condition {
        OutputCondition::Every(..) => true,
        OutputCondition::WhenBallCountHits { counts } => {
            counts.iter().any(|&count| state.len() <= count)
        }
    } {
        // Write to output
        IterableFrame {
            time,
            balls: state.values().map(|(b, _)| b),
        }
        .write_to(&mut output_writer)
        .unwrap();
    }

    let delta_time = (10.0 as Float).powi(-(config.delta_time_n as i32));
    let mut iteration = 0;

    let mut predictions = BTreeMap::new();
    let mut predicted_balls = Vec::new();
    let mut forces = HashMap::new();

    while !stop_condition(&state, time) {
        let radius_sum = config.simple_input_data.ball_radius * 2.0;

        predictions.clear();
        predictions.extend(state.iter().map(|(&id, (b, [r2, r3, r4, r5]))| {
            (
                id,
                GearPredictor::from_ball(b, *r2, *r3, *r4, *r5).predict(delta_time),
            )
        }));

        predicted_balls.clear();
        predicted_balls.extend(predictions.iter().map(|(&id, pred)| Ball {
            id,
            radius: state[&id].0.radius,
            position: pred.predictions[0],
            velocity: pred.predictions[1],
        }));

        let neighbors = SimpleNeighborFinder::find_neighbors(
            &predicted_balls,
            cim::simple_finder::SystemInfo {
                cyclic: false,
                interaction_radius: 0.0,
                space_width: config.simple_input_data.table_width,
                space_height: config.simple_input_data.table_height,
            },
        );

        /*
         * It's slower for normal ball count but faster for more balls
        let neighbors = CimNeighborFinder::find_neighbors(
            &state.values().cloned().collect_vec(),
            cim::cim_finder::SystemInfo {
                cyclic: false,
                interaction_radius: 0.0,
                space_width: config.simple_input_data.table_width,
                space_height: config.simple_input_data.table_height,
                columns: (config.simple_input_data.table_width / radius_sum).floor() as usize,
                rows: (config.simple_input_data.table_height / radius_sum).floor() as usize,
            },
        );
        */

        forces.clear();
        forces.extend(state.iter().map(|(&k, _)| (k, Vector2::zeros())));

        let get_predicted_ball = |corrector: &GearCorrector<_>, original_ball: &Ball| {
            let &Ball { id, radius, .. } = original_ball;
            let &GearCorrector {
                predictions: [position, velocity, ..],
            } = corrector;
            Ball {
                id,
                radius,
                position,
                velocity,
            }
        };

        for (id, ball) in predictions
            .iter()
            .map(|(&id, corrector)| (id, get_predicted_ball(corrector, &state[&id].0)))
        {
            let neighs = neighbors.get_neighbors(id);

            for other in neighs
                .filter(|other_id| id > **other_id)
                .map(|id| get_predicted_ball(&predictions[id], &state[id].0))
            {
                let force = calculate_force(&ball, &other, radius_sum);
                *forces.get_mut(&ball.id).unwrap() += force;
                *forces.get_mut(&other.id).unwrap() -= force;
            }

            let walls = did_ball_go_outside(&ball, &config);
            for wall in walls {
                match wall {
                    Wall::Left => {
                        let depth = -(ball.position.x - ball.radius);
                        forces.get_mut(&ball.id).unwrap().x += K * depth;
                    }
                    Wall::Right => {
                        let depth =
                            ball.position.x - config.simple_input_data.table_width + ball.radius;
                        forces.get_mut(&ball.id).unwrap().x -= K * depth;
                    }
                    Wall::Bottom => {
                        let depth = -(ball.position.y - ball.radius);
                        forces.get_mut(&ball.id).unwrap().y += K * depth;
                    }
                    Wall::Top => {
                        let depth =
                            ball.position.y - config.simple_input_data.table_height + ball.radius;
                        forces.get_mut(&ball.id).unwrap().y -= K * depth;
                    }
                }
            }
        }

        for (id, (ball, higher_order)) in state.iter_mut() {
            let force = forces.get(id).cloned().unwrap_or_else(Vector2::zeros);
            let acceleration = force / config.simple_input_data.ball_mass;
            let [p, v, r2, r3, r4, r5] = predictions[id].correct(acceleration, delta_time);
            ball.position = p;
            ball.velocity = v;
            *higher_order = [r2, r3, r4, r5];
        }

        let removed_balls = state
            .drain_filter(|_, (ball, _)| {
                holes.iter().any(|hole| {
                    (hole - ball.position).magnitude_squared()
                        <= (config.simple_input_data.hole_radius + ball.radius).powi(2)
                })
            })
            .count();

        iteration += 1;
        time = iteration as f64 * delta_time;

        if match &config.output_condition {
            OutputCondition::Every(EveryArgs { steps, .. }) => iteration % steps == 0,
            OutputCondition::WhenBallCountHits { counts } => counts
                .iter()
                .any(|&count| state.len() <= count && state.len() + removed_balls > count),
        } {
            // Write to output
            IterableFrame {
                time,
                balls: state.values().map(|(b, _)| b),
            }
            .write_to(&mut output_writer)
            .unwrap();
        }
    }

    // Write last frame in case it wasnt
    if match config.output_condition {
        OutputCondition::Every(EveryArgs { steps, last }) => last && iteration % steps != 0,
        OutputCondition::WhenBallCountHits { .. } => false,
    } {
        IterableFrame {
            time,
            balls: state.values().map(|(b, _)| b),
        }
        .write_to(&mut output_writer)
        .unwrap();
    }
}

fn main() {
    let args = Arguments::parse();

    let input = fs::read_to_string(args.input).unwrap();
    let input = InputData {
        simple_input_data: input_parser()
            .parse(&input)
            .into_result()
            .expect("Error parsing input data."),
        delta_time_n: args.delta_time_n,
        with_holes: args.with_holes,
        output_condition: args.output_condition,
    };

    let writer = if let Some(output) = args.output {
        Box::new(File::create(output).unwrap()) as Box<dyn Write>
    } else {
        Box::new(stdout())
    };

    run(input, writer, |state, t| {
        args.max_duration
            .is_some_and(|max_duration| t > max_duration)
            || args
                .min_ball_amount
                .is_some_and(|min_ball_amount| state.len() < min_ball_amount)
    });
}
