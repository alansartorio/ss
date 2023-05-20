use std::collections::VecDeque;

use capturable_visualization::VisualizationBuilder;
use gear_predictor_corrector::{GearCorrector, GearPredictor};
use nalgebra::{Matrix3, Point2, Scale2, Translation2};
use nannou::state::mouse::ButtonPosition;
use nannou::winit::dpi::PhysicalPosition;
use nannou::{color, prelude::*, App, Draw};
use ndarray::parallel::prelude::*;
use ndarray::{Array2, Axis, Zip};

const K: f64 = 1e3;

type Vector2 = nalgebra::Vector2<f64>;

fn link_force(position1: Vector2, position2: Vector2, natural_length: f64) -> Option<Vector2> {
    let delta = position2 - position1;
    let dist = delta.magnitude();
    let expansion = dist - natural_length;
    if expansion > 0.0 {
        let force = K * expansion;
        (force < 0.3).then(|| force * (delta / dist))
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

impl Node {
    fn position(&self) -> &Vector2 {
        match self {
            Self::Moving(MovingNode { position, .. }) | Self::Fixed { position } => position,
        }
    }
}

fn main() {
    VisualizationBuilder::new(model)
        .update(update)
        .draw(draw)
        .event(event)
        .run();
}

fn event(_app: &App, model: &mut Model, event: WindowEvent) {
    match event {
        Resized(size) => {
            model.window_transform = {
                let scale = 1.0 / {
                    let h = 1.0;
                    let w = 1.0;
                    let (win_w, win_h) = (size.x as f64, size.y as f64);
                    f64::min(win_w / w, win_h / h)
                };

                nalgebra::convert::<_, Matrix3<f64>>(Translation2::new(0.05, 0.0))
                    * nalgebra::convert::<_, Matrix3<f64>>(Scale2::new(0.2, 0.2))
                    * nalgebra::convert::<_, Matrix3<f64>>(Translation2::new(0.0, 0.5))
                    * nalgebra::convert::<_, Matrix3<f64>>(Scale2::new(scale, scale))
            }
        }
        MouseWheel(scroll_delta, _) => {
            let scroll = match scroll_delta {
                MouseScrollDelta::LineDelta(_, y) => y as f64,
                MouseScrollDelta::PixelDelta(PhysicalPosition { y, .. }) => y,
            };
            model.interaction_radius *= 1.1.pow(scroll / 100.0);
        }
        KeyPressed(key) => match key {
            Key::R => {
                model.mesh_model = initial_model(model.mesh_size);
            }
            key @ (Key::Plus | Key::Minus) => {
                model.mesh_size = match key {
                    Key::Plus => model.mesh_size.saturating_add(5),
                    Key::Minus => model.mesh_size.saturating_sub(5).max(2),
                    _ => unreachable!(),
                };
                println!("mesh_size = {}", model.mesh_size);
                model.mesh_model = initial_model(model.mesh_size);
            }
            _ => {}
        },
        _ => {}
    }
}

struct Model {
    window_transform: Matrix3<f64>,
    interaction_radius: f64,
    mesh_size: usize,
    mesh_model: MeshModel,
    frame_counter: usize,
    frame_times: VecDeque<f64>,
}

struct MeshModel {
    mesh_nodes: Array2<Node>,
    horizontal_edges: Array2<bool>,
    vertical_edges: Array2<bool>,
}

const PRINT_FRAMES_EVERY: usize = 10;
const FPS_MEAN_WINDOW: usize = 100;

fn model(_app: &App) -> Model {
    let initial_mesh_size = 10;
    Model {
        mesh_model: initial_model(initial_mesh_size),
        window_transform: Matrix3::identity(),
        interaction_radius: 0.03,
        mesh_size: initial_mesh_size,
        frame_counter: 0,
        frame_times: VecDeque::with_capacity(FPS_MEAN_WINDOW),
    }
}

fn initial_model(mesh_size: usize) -> MeshModel {
    let mesh_nodes = Array2::<Node>::from_shape_fn((mesh_size, mesh_size), |(y, x)| {
        let position = Vector2::new(x as f64, y as f64) / (mesh_size - 1) as f64 * 0.1
            + Vector2::new(0.0, 0.1);
        if y == mesh_size - 1 {
            Node::Fixed { position }
        } else {
            Node::Moving(MovingNode {
                position,
                weight: 0.0005,
                ..Default::default()
            })
        }
    });
    MeshModel {
        mesh_nodes,
        horizontal_edges: Array2::from_shape_fn((mesh_size, mesh_size - 1), |_| true),
        vertical_edges: Array2::from_shape_fn((mesh_size - 1, mesh_size), |_| true),
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
    cursor_pos: Option<Vector2>,
    interaction_radius: f64,
    natural_length: f64,
    dt: f64,
    time: f64,
) {
    mesh_nodes.par_iter_mut().for_each(|node| {
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
    });
    let mut accelerations = Array2::<Vector2>::zeros(mesh_nodes.dim());
    let mut accelerations_neighbors = accelerations.clone();
    //mesh_nodes.indexed_iter().par_bridge().for_each(|((y, x), node)| {

    let calculate_apply_acceleration = |node: &Node,
                                        neighbor: &Node,
                                        acceleration: &mut Vector2,
                                        neighbor_acceleration: &mut Vector2,
                                        edge: &mut bool| {
        if *edge {
            if let Some(force) = link_force(*node.position(), *neighbor.position(), natural_length)
            {
                if let Node::Moving(MovingNode { weight, .. }) = node {
                    *acceleration += force / *weight;
                }
                if let Node::Moving(MovingNode { weight, .. }) = neighbor {
                    *neighbor_acceleration -= force / *weight;
                }
            } else {
                *edge = false;
            }
        }
    };

    Zip::from(mesh_nodes.slice_axis(Axis(1), (..-1).into()))
        .and(mesh_nodes.slice_axis(Axis(1), (1..).into()))
        .and(&mut accelerations.slice_axis_mut(Axis(1), (..-1).into()))
        .and(&mut accelerations_neighbors.slice_axis_mut(Axis(1), (1..).into()))
        .and(horizontal_edges.view_mut())
        .par_for_each(calculate_apply_acceleration);

    Zip::from(mesh_nodes.slice_axis(Axis(0), (..-1).into()))
        .and(mesh_nodes.slice_axis(Axis(0), (1..).into()))
        .and(&mut accelerations.slice_axis_mut(Axis(0), (..-1).into()))
        .and(&mut accelerations_neighbors.slice_axis_mut(Axis(0), (1..).into()))
        .and(vertical_edges.view_mut())
        .par_for_each(calculate_apply_acceleration);

    accelerations += &accelerations_neighbors;

    // Apply simple accelerations
    //par_azip!((acceleration in &mut accelerations, node in mesh_nodes), {

    Zip::from(&mut accelerations)
        .and(mesh_nodes.view())
        .par_for_each(|acceleration, node| {
            // Gravity
            *acceleration += Vector2::y() * -GRAVITY;

            if let Node::Moving(MovingNode {
                weight,
                velocity,
                position,
                ..
            }) = node
            {
                // Wind
                //accelerations[[y, x]] +=
                //Vector2::x() * 0.002 * (0.5 + (1.0 + (time * 10.0).sin()) * 0.2) / *weight;

                // Cursor
                if let Some(cursor_pos) = cursor_pos {
                    let delta = *position - cursor_pos;
                    let magnitude = delta.magnitude();
                    if magnitude > 0.0001 {
                        *acceleration += (delta / magnitude)
                            * (smoothstep(magnitude, interaction_radius, 0.0).powi(4) * 0.2
                                / *weight);
                    }
                }

                // Drag
                *acceleration += -*velocity * 0.003 / *weight;
            }
        });
    Zip::from(mesh_nodes)
        .and(&accelerations)
        .par_for_each(|node, &acceleration| {
            if let Node::Moving(MovingNode {
                position,
                velocity,
                higher_order,
                ..
            }) = node
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
                .correct(acceleration, dt);
                *position = corrected[0];
                *velocity = corrected[1];
                higher_order.copy_from_slice(&corrected[2..]);
            }
        });
}

fn update(app: &App, model: &mut Model, update: Update) {
    let dt = update.since_last.as_secs_f64();
    model.frame_times.push_front(dt);
    if model.frame_times.len() > FPS_MEAN_WINDOW {
        model.frame_times.pop_back();
    }
    if model.frame_counter % PRINT_FRAMES_EVERY == PRINT_FRAMES_EVERY - 1 {
        let fps = (model.frame_times.len() as f64 / model.frame_times.iter().sum::<f64>()) as usize;
        println!("fps = {fps}");
        model.frame_counter = 0;
    } else {
        model.frame_counter += 1;
    }
    const STEPS: usize = 100;
    let mut dt = dt / STEPS as f64;
    const MAX_DT: f64 = 0.0002;
    if dt > MAX_DT {
        //println!("slowing!");
        dt = MAX_DT;
    }
    let cursor_pos = matches!(app.mouse.buttons.left(), ButtonPosition::Down(..))
        .then(|| Vector2::new(app.mouse.x as f64, app.mouse.y as f64))
        .map(|pos| {
            Point2::from_homogeneous(model.window_transform * Point2::from(pos).to_homogeneous())
                .unwrap()
                .coords
        });

    for _ in 0..STEPS {
        step(
            &mut model.mesh_model.mesh_nodes,
            &mut model.mesh_model.horizontal_edges,
            &mut model.mesh_model.vertical_edges,
            cursor_pos,
            model.interaction_radius,
            0.1 / (model.mesh_size - 1) as f64,
            dt,
            update.since_start.as_secs_f64(),
        );
    }
}

fn draw(app: &App, model: &Model, draw: &Draw) {
    draw.background().color(BLACK);
    let draw = draw.x(0.5).scale(5.0).x(-0.05);
    for node in &model.mesh_model.mesh_nodes {
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

    for ((y, x), node1) in model.mesh_model.mesh_nodes.indexed_iter() {
        let right = (x + 1 < model.mesh_model.mesh_nodes.dim().1
            && model.mesh_model.horizontal_edges[[y, x]])
        .then_some([y, x + 1]);
        let up = (y + 1 < model.mesh_model.mesh_nodes.dim().0
            && model.mesh_model.vertical_edges[[y, x]])
        .then_some([y + 1, x]);
        // Link forces
        for [ny, nx] in [right, up].into_iter().flatten() {
            let node2 = &model.mesh_model.mesh_nodes[[ny, nx]];
            let position1 = node1.position().cast();
            let position2 = node2.position().cast();

            draw.line()
                .start(pt2(position1.x, position1.y))
                .end(pt2(position2.x, position2.y))
                .stroke_weight(0.0002)
                .color(color::WHITE);
        }
    }
    let mouse = Point2::from_homogeneous(
        model.window_transform
            * Point2::new(app.mouse.x, app.mouse.y)
                .cast()
                .to_homogeneous(),
    )
    .unwrap()
    .coords;
    draw.ellipse()
        .radius(model.interaction_radius as f32)
        .resolution(20.0)
        .x(mouse.x as f32)
        .y(mouse.y as f32)
        .color(rgba(1.0, 1.0, 1.0, 0.3));
}