use capturable_visualization::VisualizationBuilder;
use gear_predictor_corrector::{GearCorrector, GearPredictor};
use itertools::Itertools;
use nannou::{color, prelude::*, App, Draw};
use ndarray::{s, Array2, Axis};

const K: f64 = 10e4;

type Vector2 = nalgebra::Vector2<f64>;

fn link_force(position1: Vector2, position2: Vector2) -> f64 {
    let dist = (position1 - position2).magnitude() - 0.01;
    if dist > 0.0 {
        K * dist
    } else {
        0.0
    }
}

const GRAVITY: f64 = 9.8;

#[derive(Debug, Default)]
struct MovingNode {
    position: Vector2,
    velocity: Vector2,
    higher_order: [Vector2; 4],
    weight: f64,
}

#[derive(Debug)]
enum Node {
    Moving(MovingNode),
    Fixed { position: Vector2 },
}

fn main() {
    VisualizationBuilder::new(model)
        .update(update)
        .draw(draw)
        .run();
}

struct Model {
    mesh_nodes: Array2<Node>,
}

fn model(app: &App) -> Model {
    let mesh_nodes = Array2::<Node>::from_shape_fn((11, 11), |(y, x)| {
        let position = Vector2::new(x as f64, y as f64 + 3.5);
        if y == 10 {
            Node::Fixed { position }
        } else {
            Node::Moving(MovingNode {
                position,
                weight: 0.01,
                ..Default::default()
            })
        }
    });

    Model { mesh_nodes }
}

fn update(_app: &App, model: &mut Model, update: Update) {
    let dt = update.since_last.as_secs_f64();
    for node in model.mesh_nodes.iter_mut() {
        match node {
            Node::Moving(MovingNode {
                position,
                velocity,
                higher_order,
                ..
            }) => {
                let predictions = GearPredictor {
                    rs: [
                        *position,
                        *velocity,
                        higher_order[0],
                        higher_order[1],
                        higher_order[2],
                        higher_order[3],
                    ],
                }
                .predict(dt)
                .predictions;
                *position = predictions[0];
                *velocity = predictions[1];
                higher_order.copy_from_slice(&predictions[2..]);
            }
            Node::Fixed { .. } => (),
        };
    }
    let mut accelerations = Array2::<Vector2>::zeros(model.mesh_nodes.dim());
    for ((y, x), node) in model.mesh_nodes.indexed_iter() {
        accelerations[[y, x]] += Vector2::new(0.0, -GRAVITY);
    }
    model
        .mesh_nodes
        .zip_mut_with(&accelerations, |mut node, acceleration| {
            if let Node::Moving(MovingNode {
                position,
                velocity,
                higher_order,
                ..
            }) = &mut node
            {
                let corrected = GearCorrector {
                    predictions: [
                        *position,
                        *velocity,
                        higher_order[0],
                        higher_order[1],
                        higher_order[2],
                        higher_order[3],
                    ],
                }
                .correct(*acceleration, dt);
                *position = corrected[0];
                *velocity = corrected[1];
                higher_order.copy_from_slice(&corrected[2..]);
            }
        });
}

fn draw(_app: &App, model: &Model, draw: &Draw) {
    draw.background().color(BLACK);
    let draw = draw.x(0.5).scale(1.0 / 14.0).x(-5.0);
    for node in &model.mesh_nodes {
        let position = match node {
            Node::Moving(MovingNode { position, .. }) | Node::Fixed { position } => position,
        }
        .cast();
        draw.ellipse().radius(0.05).x(position.x).y(position.y);
    }

    for row in model
        .mesh_nodes
        .axis_iter(Axis(0))
        .chain(model.mesh_nodes.axis_iter(Axis(1)))
    {
        for (node1, node2) in row.iter().tuple_windows() {
            let position1 = match node1 {
                Node::Moving(MovingNode { position, .. }) | Node::Fixed { position } => position,
            }
            .cast();
            let position2 = match node2 {
                Node::Moving(MovingNode { position, .. }) | Node::Fixed { position } => position,
            }
            .cast();

            draw.line()
                .start(pt2(position1.x, position1.y))
                .end(pt2(position2.x, position2.y))
                .stroke_weight(0.02)
                .color(color::WHITE);
        }
    }
}
