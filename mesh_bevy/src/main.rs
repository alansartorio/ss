use bevy::math::{vec2, vec3, Vec3Swizzles};
use bevy::prelude::*;
use bevy::sprite::MaterialMesh2dBundle;

#[derive(Component)]
struct Node;

#[derive(Component)]
struct Edge(Entity, Entity);

fn add_nodes(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let mut add_node = |pos: Vec2| {
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
                Node,
            ))
            .id()
    };

    let node1 = add_node(vec2(0.00, 0.0));
    let node2 = add_node(vec2(0.01, 0.0));
    let node3 = add_node(vec2(0.01, 0.01));

    let mut add_edge = |start, end| {
        commands
            .spawn((
                MaterialMesh2dBundle {
                    mesh: meshes
                        .add(shape::Quad::new(vec2(1.0, 0.0005)).into())
                        .into(),
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

    add_edge(node1, node2);
    add_edge(node2, node3);
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
        .add_systems(Update, update_edges)
        .run();
}
