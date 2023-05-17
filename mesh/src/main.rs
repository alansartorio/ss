use capturable_visualization::VisualizationBuilder;
use gear_predictor_corrector::{GearCorrector, GearPredictor};
use itertools::Itertools;
use nannou::{color, prelude::*, App, Draw};
use ndarray::{s, Array2, Axis};

const K: f64 = 10e3;

type Vector2 = nalgebra::Vector2<f64>;

fn link_force(position1: Vector2, position2: Vector2) -> Vector2 {
    let delta = position2 - position1;
    let dist = delta.magnitude();
    let expansion = dist - 0.01;
    if expansion > 0.0 {
        K * expansion * delta / dist
    } else {
        Vector2::zeros()
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
        let position = Vector2::new(x as f64, y as f64) / 10.0 * 0.1 + Vector2::new(0.0, 0.1);
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

fn step(mesh_nodes: &mut Array2<Node>, dt: f64) {
    for node in mesh_nodes.iter_mut() {
        if let Node::Moving(MovingNode {
            position,
            velocity,
            higher_order,
            ..
        }) = node
        {
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
    }
    let mut accelerations = Array2::<Vector2>::zeros(mesh_nodes.dim());
    for ((y, x), node) in mesh_nodes.indexed_iter() {
        let position = match node {
            Node::Moving(MovingNode { position, .. }) | Node::Fixed { position } => position,
        };

        // Link forces
        for [ny, nx] in [[0, 1], [1, 0]] {
            let nx = x + nx;
            let ny = y + ny;
            if nx < mesh_nodes.dim().1 && ny < mesh_nodes.dim().0 {
                let neighbor = &mesh_nodes[[ny, nx]];
                let neigh_position = match neighbor {
                    Node::Moving(MovingNode { position, .. }) | Node::Fixed { position } => {
                        position
                    }
                };
                let force = link_force(*position, *neigh_position);

                if let Node::Moving(MovingNode { weight, .. }) = node {
                    accelerations[[y, x]] += force / *weight;
                }
                if let Node::Moving(MovingNode { weight, .. }) = neighbor {
                    accelerations[[ny, nx]] -= force / *weight;
                }
            }
        }

        // Gravity
        accelerations[[y, x]] += Vector2::new(0.0, -GRAVITY);

        // Wind
        if let Node::Moving(MovingNode { weight, .. }) = node {
            accelerations[[y, x]] += Vector2::new(0.01, 0.0) / *weight;
        }
    }
    mesh_nodes.zip_mut_with(&accelerations, |mut node, acceleration| {
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

fn update(_app: &App, model: &mut Model, update: Update) {
    const STEPS: usize = 1000;
    let dt = update.since_last.as_secs_f64() / STEPS as f64;
    for _ in 0..100 {
        step(&mut model.mesh_nodes, dt);
    }
}

fn draw(_app: &App, model: &Model, draw: &Draw) {
    draw.background().color(BLACK);
    let draw = draw.x(0.5).scale(5.0).x(-0.05);
    for node in &model.mesh_nodes {
        let position = match node {
            Node::Moving(MovingNode { position, .. }) | Node::Fixed { position } => position,
        }
        .cast();
        draw.ellipse()
            .radius(0.001)
            .resolution(8.0)
            .x(position.x)
            .y(position.y);
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
                .stroke_weight(0.0002)
                .color(color::WHITE);
        }
    }
}
