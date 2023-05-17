use capturable_visualization::VisualizationBuilder;
use gear_predictor_corrector::{GearCorrector, GearPredictor};
use nalgebra::{Matrix3, Point2, Scale2, Translation2};
use nannou::{color, prelude::*, App, Draw};
use ndarray::Array2;

const K: f64 = 1e3;

type Vector2 = nalgebra::Vector2<f64>;

fn link_force(position1: Vector2, position2: Vector2) -> Option<Vector2> {
    let delta = position2 - position1;
    let dist = delta.magnitude();
    let expansion = dist - 0.1 / (MESH_SIZE - 1) as f64;
    if expansion > 0.0 {
        let force = K * expansion;
        (force < 2.0).then(|| force * (delta / dist))
    } else {
        Some(Vector2::zeros())
    }
}

const GRAVITY: f64 = 9.8e-1;

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
    horizontal_edges: Array2<bool>,
    vertical_edges: Array2<bool>,
}

const MESH_SIZE: usize = 30;

fn model(_app: &App) -> Model {
    initial_model()
}

fn initial_model() -> Model {
    let mesh_nodes = Array2::<Node>::from_shape_fn((MESH_SIZE, MESH_SIZE), |(y, x)| {
        let position = Vector2::new(x as f64, y as f64) / (MESH_SIZE - 1) as f64 * 0.1
            + Vector2::new(0.0, 0.1);
        if y == MESH_SIZE - 1 {
            Node::Fixed { position }
        } else {
            Node::Moving(MovingNode {
                position,
                weight: 0.01,
                ..Default::default()
            })
        }
    });

    Model {
        mesh_nodes,
        horizontal_edges: Array2::from_shape_fn((MESH_SIZE, MESH_SIZE - 1), |_| true),
        vertical_edges: Array2::from_shape_fn((MESH_SIZE - 1, MESH_SIZE), |_| true),
    }
}

fn smoothstep(x: f64, start: f64, end: f64) -> f64 {
    let x = ((x - start) / (end - start)).clamp(0.0, 1.0);

    x * x * (3.0 - 2.0 * x)
}

fn step(
    mesh_nodes: &mut Array2<Node>,
    horizontal_edges: &mut Array2<bool>,
    vertical_edges: &mut Array2<bool>,
    cursor_pos: Vector2,
    dt: f64,
) {
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

        let right =
            (x + 1 < mesh_nodes.dim().1).then(|| (&mut horizontal_edges[[y, x]], [y, x + 1]));
        let up = (y + 1 < mesh_nodes.dim().0).then(|| (&mut vertical_edges[[y, x]], [y + 1, x]));
        // Link forces
        for (edge, [ny, nx]) in [right, up].into_iter().flatten().filter(|(edge, _)| **edge) {
            let neighbor = &mesh_nodes[[ny, nx]];
            let neigh_position = match neighbor {
                Node::Moving(MovingNode { position, .. }) | Node::Fixed { position } => position,
            };
            if let Some(force) = link_force(*position, *neigh_position) {
                if let Node::Moving(MovingNode { weight, .. }) = node {
                    accelerations[[y, x]] += force / *weight;
                }
                if let Node::Moving(MovingNode { weight, .. }) = neighbor {
                    accelerations[[ny, nx]] -= force / *weight;
                }
            } else {
                *edge = false;
            }
        }

        // Gravity
        accelerations[[y, x]] += Vector2::y() * -GRAVITY;

        if let Node::Moving(MovingNode {
            weight, velocity, ..
        }) = node
        {
            //// Wind
            //accelerations[[y, x]] += Vector2::x() * 0.01 / *weight;

            // Cursor
            let delta = position - cursor_pos;
            let magnitude = delta.magnitude();
            if magnitude > 0.0001 {
                accelerations[[y, x]] += (delta / magnitude)
                    * (smoothstep(magnitude, 0.01, 0.0).powi(1) * 0.5 / *weight);
            }

            // Drag
            accelerations[[y, x]] += -velocity * 0.03 / *weight;
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

fn update(app: &App, model: &mut Model, update: Update) {
    const STEPS: usize = 100;
    let dt = update.since_last.as_secs_f64() / STEPS as f64;
    let cursor_pos = Vector2::new(app.mouse.x as f64, app.mouse.y as f64);

    let transform = {
        let window = app.main_window();
        let scale = 1.0 / {
            let h = 1.0;
            let w = 1.0;
            let (win_w, win_h) = window.inner_size_pixels();
            let win_w = win_w as f64;
            let win_h = win_h as f64;
            f64::min(win_w / w, win_h / h)
        };

        nalgebra::convert::<_, Matrix3<f64>>(Translation2::new(0.05, 0.0))
            * nalgebra::convert::<_, Matrix3<f64>>(Scale2::new(0.2, 0.2))
            * nalgebra::convert::<_, Matrix3<f64>>(Translation2::new(0.0, 0.5))
            * nalgebra::convert::<_, Matrix3<f64>>(Scale2::new(scale, scale))
    };
    let cursor_pos =
        Point2::from_homogeneous(transform * Point2::from(cursor_pos).to_homogeneous())
            .unwrap()
            .coords;
    for _ in 0..STEPS {
        step(
            &mut model.mesh_nodes,
            &mut model.horizontal_edges,
            &mut model.vertical_edges,
            cursor_pos,
            dt,
        );
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
            .radius(0.0005)
            .resolution(8.0)
            .x(position.x)
            .y(position.y);
    }

    for ((y, x), node1) in model.mesh_nodes.indexed_iter() {
        let right = (x + 1 < model.mesh_nodes.dim().1 && model.horizontal_edges[[y, x]])
            .then_some([y, x + 1]);
        let up = (y + 1 < model.mesh_nodes.dim().0 && model.vertical_edges[[y, x]])
            .then_some([y + 1, x]);
        // Link forces
        for [ny, nx] in [right, up].into_iter().flatten() {
            let node2 = &model.mesh_nodes[[ny, nx]];
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
