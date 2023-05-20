use bevy::math::{vec2, vec3, Vec3Swizzles};
use bevy::prelude::*;
use bevy::sprite::MaterialMesh2dBundle;
use gear_predictor_corrector::GearPredictor;

#[derive(Component, PartialEq, Eq)]
enum Node {
    Fixed,
    Moving,
}

#[derive(Component)]
struct Edge(Entity, Entity);

#[derive(Component, Default)]
struct Integration {
    rs: [Vec2; 5],
}

fn generate_grid_mesh(
    start: Vec2,
    end: Vec2,
    columns: usize,
    rows: usize,
) -> (Vec<(Vec2, Node)>, Vec<(usize, usize)>) {
    let diagonal = (end - start) / vec2((columns - 1) as f32, (rows - 1) as f32);
    let mut nodes = vec![];
    let mut edges = vec![];
    for y in 0..rows {
        let row_index = y * columns;
        for x in 0..columns {
            let position = start + vec2(diagonal.x * x as f32, diagonal.y * y as f32);
            nodes.push((position, if y == 0 { Node::Fixed } else { Node::Moving }));
        }
        for x in 1..columns {
            edges.push((row_index + x - 1, row_index + x));
        }
    }
    for y in 1..rows {
        let row_index = y * columns;
        for x in 0..columns {
            edges.push((row_index + x - columns, row_index + x));
        }
    }
    (nodes, edges)
}

fn add_nodes(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let (nodes, edges) = generate_grid_mesh(vec2(-0.05, 0.1), vec2(0.05, 0.0), 10, 10);

    let mut add_node = |pos: Vec2, node_type: Node| {
        commands
            .spawn((
                MaterialMesh2dBundle {
                    mesh: meshes.add(shape::Circle::new(0.001).into()).into(),
                    material: materials.add(ColorMaterial::from(Color::WHITE)),
                    transform: Transform {
                        translation: pos.extend(0.5),
                        ..default()
                    },
                    ..default()
                },
                node_type,
                Integration::default(),
            ))
            .id()
    };

    let mut node_entities = vec![];
    for (pos, node_type) in nodes {
        node_entities.push(add_node(pos, node_type));
    }

    let mut add_edge = |start, end| {
        commands
            .spawn((
                MaterialMesh2dBundle {
                    mesh: meshes.add(shape::Quad::new(vec2(1.0, 0.001)).into()).into(),
                    material: materials.add(ColorMaterial::from(Color::BLACK)),
                    transform: Transform {
                        translation: vec3(0.0, 0.0, 0.4),
                        ..default()
                    },
                    ..default()
                },
                Edge(start, end),
            ))
            .id()
    };

    for (node1, node2) in edges {
        add_edge(node_entities[node1], node_entities[node2]);
    }
}

fn add_camera(mut commands: Commands) {
    commands.spawn(Camera2dBundle {
        projection: OrthographicProjection {
            scale: 5e-4,
            ..default()
        },
        ..default()
    });
}

fn update_nodes(time: Res<Time>, mut nodes: Query<(&Node, &mut Integration, &mut Transform)>) {
    const STEPS: usize = 100;
    let dt = time.delta_seconds().max(1e-6) / STEPS as f32;
    //let dt = 1e-3 / STEPS as f32;
    for _ in 0..STEPS {
        for (node, mut integration, mut transform) in nodes.iter_mut() {
            if *node == Node::Moving {
                let mut rs = [Vec2::ZERO; 6];
                rs[0] = transform.translation.xy();
                rs[1..].copy_from_slice(integration.rs.as_slice());
                let predictor = GearPredictor { rs };
                let acceleration = vec2(0.0, -0.098);
                let predicted = predictor.predict(dt);
                let corrected = predicted.correct(acceleration, dt);
                integration.rs.copy_from_slice(&corrected[1..]);
                transform.translation.x = corrected[0].x;
                transform.translation.y = corrected[0].y;
            }
        }
    }
}

fn update_edges(
    nodes: Query<&Transform, With<Node>>,
    mut edges: Query<(&Edge, &mut Transform), Without<Node>>,
) {
    for (edge, mut transform) in edges.iter_mut() {
        let node1 = nodes.get(edge.0).unwrap();
        let node2 = nodes.get(edge.1).unwrap();
        transform.scale.x = node2.translation.distance(node1.translation);
        let delta = node2.translation.xy() - node1.translation.xy();
        let start = node1.translation.xy() + delta / 2.0;
        transform.translation.x = start.x;
        transform.translation.y = start.y;
        transform.rotation = Quat::from_rotation_z(f32::atan2(delta.y, delta.x));
    }
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, add_camera)
        .add_systems(Startup, add_nodes)
        .add_systems(Update, (update_nodes, update_edges.after(update_nodes)))
        .run();
}
