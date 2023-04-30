#![feature(let_chains)]

use chumsky::{prelude::Input, Parser};
use cim::particles::ID;
use std::{
    collections::{BTreeMap, HashMap},
    fs::{self, File},
    io::{stdout, Write},
    thread,
    time::Duration,
};

use itertools::Itertools;
use nalgebra::Vector2;
use pool::{
    models::{Ball, Frame, InputData as SimpleInputData},
    parser::input_parser,
    Float, HOLE_POSITIONS,
};

use clap::Parser as _parser;

#[derive(clap::Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    input: String,

    #[arg(short, long)]
    output: Option<String>,

    #[arg(short, long)]
    delta_time_n: u16,

    #[arg(short, long)]
    with_holes: bool,
}

struct InputData {
    simple_input_data: SimpleInputData,
    delta_time_n: u16,
    with_holes: bool,
}

fn are_balls_colliding(b1: &Ball, b2: &Ball, radius_sum: Float) -> bool {
    (b1.position - b2.position).magnitude_squared() < radius_sum.powi(2)
}

#[derive(Debug, Copy, Clone)]
enum Wall {
    Left,
    Right,
    Top,
    Bottom,
}

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
    let k = 10e10 as Float;
    let r_hat = (other.position - b.position).normalize();
    k * ((b.position - other.position).magnitude() - radius_sum) * r_hat
}

/*
fn find_earliest_collision(
    state: &[&Ball],
    holes: &[Vector2<Float>],
    config: &InputData,
) -> Option<Collision> {
    let mut earliest: Option<Collision> = None;
    for (ball_1, ball_2) in state.iter().tuple_combinations() {
        if let Some(time) = find_collision_between_balls(ball_1, ball_2, config.ball_radius * 2.0)
            && earliest.map(|e| time < e.time).unwrap_or(true)
        {
            earliest = Some(Collision {
                time,
                info: CollisionAgainst::Ball(ball_1.id, ball_2.id)
            });
        }
    }

    for (ball, hole) in state.iter().cartesian_product(holes.iter()) {
        if let Some(time) = find_collision_between_balls(ball, &Ball {
            id: 0,
            position: *hole,
            velocity: Vector2::zeros()
        }, config.ball_radius + config.hole_radius)
            && earliest.map(|e| time < e.time).unwrap_or(true)
        {
            earliest = Some(Collision {
                time,
                info: CollisionAgainst::Hole(ball.id)
            });
        }
    }

    for ball in state.iter() {
        if let Some((time, wall_type)) = find_collision_against_wall(ball, config)
            && earliest.map(|e| time < e.time).unwrap_or(true)
        {
            earliest = Some(Collision {
                time,
                info: CollisionAgainst::Wall(ball.id, wall_type)
            });
        }
    }

    earliest
}
*/

/*
fn apply_collision(state: &mut BTreeMap<ID, Ball>, config: &InputData, collision: Collision) {
    match collision.info {
        CollisionAgainst::Ball(id1, id2) => {
            let delta_v = state[&id2].velocity - state[&id1].velocity;
            let delta_r = state[&id2].position - state[&id1].position;
            let sigma = config.ball_radius * 2.0;

            let j = (2.0 * config.ball_mass.powi(2) * (delta_v.dot(&delta_r)))
                / (sigma * (config.ball_mass * 2.0));
            let j_vec = delta_r * j / sigma;

            let ball_1 = state.get_mut(&id1).unwrap();
            ball_1.velocity += j_vec / config.ball_mass;
            let ball_2 = state.get_mut(&id2).unwrap();
            ball_2.velocity -= j_vec / config.ball_mass;
        }
        CollisionAgainst::Wall(id, wall_type) => match wall_type {
            WallType::Horizontal => state.get_mut(&id).unwrap().velocity.y *= -1.0,
            WallType::Vertical => state.get_mut(&id).unwrap().velocity.x *= -1.0,
        },
        CollisionAgainst::Hole(id) => {
            state.remove(&id);
        }
    }
}
*/

fn euler_algorythm(
    p: Vector2<Float>,
    v: Vector2<Float>,
    a: Vector2<Float>,
    delta_time: Float,
) -> (Vector2<Float>, Vector2<Float>) {
    let v_next = v + delta_time * a;

    let p_next = p + delta_time * v_next + delta_time.powi(2) / 2.0 * a;

    (p_next, v_next)
}

fn run<W: Write, F: FnMut(&BTreeMap<ID, Ball>, Float) -> bool>(
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
        .map(|p| (p.id, p))
        .collect();

    let holes = HOLE_POSITIONS.map(|v| {
        v.component_mul(&Vector2::new(
            config.simple_input_data.table_width,
            config.simple_input_data.table_height,
        ))
    });

    {
        // Write to output
        let frame = Frame {
            time,
            balls: state.values().copied().collect_vec(),
        };
        output_writer.write_fmt(format_args!("{frame}")).unwrap();
    }

    let delta_time = (10.0 as Float).powi(-(config.delta_time_n as i32));
    let mut iteration = 0;
    dbg!(delta_time);

    while !stop_condition(&state, time) {
        let mut forces: HashMap<_, _> = state.iter().map(|(&k, _)| (k, Vector2::zeros())).collect();

        let radius_sum = config.simple_input_data.ball_radius * 2.0;
        for (ball, other) in state.values().tuple_combinations() {
            if are_balls_colliding(ball, other, radius_sum) {
                let force = calculate_force(ball, other, radius_sum);
                *forces.get_mut(&ball.id).unwrap() += force;
                *forces.get_mut(&other.id).unwrap() -= force;
            }
        }

        for (id, ball) in state.iter_mut() {
            let force = forces.get(id).cloned().unwrap_or_else(Vector2::zeros);
            let acceleration = force / config.simple_input_data.ball_mass;
            let (p, v) = euler_algorythm(ball.position, ball.velocity, acceleration, delta_time);
            ball.position = p;
            ball.velocity = v;

            let walls = did_ball_go_outside(ball, &config);
            for wall in walls {
                match wall {
                    Wall::Left => {
                        ball.position.x = -ball.position.x + radius_sum;
                        ball.velocity.x *= -1.0;
                    }
                    Wall::Right => {
                        ball.position.x =
                            2.0 * config.simple_input_data.table_width - radius_sum - ball.position.x;
                        ball.velocity.x *= -1.0;
                    }
                    Wall::Bottom => {
                        ball.position.y = -ball.position.y + radius_sum;
                        ball.velocity.y *= -1.0;
                    }
                    Wall::Top => {
                        ball.position.y =
                            2.0 * config.simple_input_data.table_height - radius_sum - ball.position.y;
                        ball.velocity.y *= -1.0;
                    }
                }
            }
        }

        time += delta_time;
        iteration += 1;

        if iteration % 10000 == 0 {
            // Write to output
            let frame = Frame {
                time,
                balls: state.values().copied().collect_vec(),
            };
            output_writer.write_fmt(format_args!("{frame}")).unwrap();
        }
    }
}

fn main() {
    let args = Args::parse();

    let input = fs::read_to_string(args.input).unwrap();
    let input = InputData {
        simple_input_data: input_parser()
            .parse(&input)
            .into_result()
            .expect("Error parsing input data."),
        delta_time_n: args.delta_time_n,
        with_holes: args.with_holes,
    };

    let writer = if let Some(output) = args.output {
        Box::new(File::create(output).unwrap()) as Box<dyn Write>
    } else {
        Box::new(stdout())
    };

    run(input, writer, |_state, _t| false);
}