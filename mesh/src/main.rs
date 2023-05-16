use capturable_visualization::VisualizationBuilder;
use itertools::Itertools;
use nannou::{color, prelude::*, App, Draw};
use ndarray::{Array2, Axis};

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
        let position = Vector2::new(x as f64, y as f64);
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

fn update(_app: &App, _model: &mut Model, _update: Update) {}

fn draw(_app: &App, model: &Model, draw: &Draw) {
    let draw = draw.scale(0.1);
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
