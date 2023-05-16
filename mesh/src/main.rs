use capturable_visualization::VisualizationBuilder;
use nannou::{
    prelude::{self, Update},
    App, Draw,
};
use ndarray::Array2;

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
        };
        draw.ellipse()
            .radius(0.05)
            .x(position.x as f32)
            .y(position.y as f32);
    }
}
