#![feature(portable_simd)]

use bevy::input::mouse::{MouseMotion, MouseScrollUnit, MouseWheel};
use bevy::math::{DVec3, VectorSpace};
use bevy::prelude::*;
use bevy_egui::egui::MouseWheelUnit;
use bevy_egui::{EguiContexts, EguiPlugin, egui};
use bevy_polyline::prelude::*;
use nbody_simd::{OrbitParams, UniversalPos};

#[derive(Component)]
struct BodyRadius(f64);

#[derive(Component)]
struct MainCamera;

#[derive(Resource, Default)]
struct FollowedBody(Option<usize>);

#[derive(Resource)]
struct UniversalCamera {
    center: UniversalPos,
    position: UniversalPos,
    distance: f64,
    pitch: f64,
    yaw: f64,
}

impl UniversalCamera {
    fn view_dir(&self) -> DVec3 {
        DVec3::new(
            self.pitch.sin() * self.yaw.sin(),
            self.pitch.cos(),
            self.pitch.sin() * self.yaw.cos(),
        )
    }

    fn compute_position(&mut self) {
        let vector = self.view_dir() * self.distance;
        self.position = self.center + nbody_simd::Vec3::new(vector.x, vector.y, vector.z);
    }

    fn rotate_yaw_pitch(&mut self, yaw: f32, pitch: f32) {
        self.pitch += (pitch as f64) / 100.0;
        self.yaw += (yaw as f64) / 100.0;
    }
}

fn convert_vec(vec: nbody_simd::Vec3<f64>) -> DVec3 {
    DVec3::new(vec.x, vec.y, vec.z)
}

use std::simd::Simd;

const PLANET_INFO: &[(&str, f64)] = &[
    ("Mercury", 2_439_700.0),
    ("Venus", 6_051_800.0),
    ("Earth", 6_371_000.0),
    ("Mars", 69_911_000.0),
    ("Jupiter", 69_911_000.0),
    ("Saturn", 69_911_000.0),
    ("Uranus", 69_911_000.0),
    ("Neptune", 69_911_000.0),
    ("Luna", 1_737_000.4),
    ("Luna", 1_737_000.4),
    ("Luna", 1_737_000.4),
    ("Luna", 1_737_000.4),
    ("Luna", 1_737_000.4),
    ("Luna", 1_737_000.4),
    ("Luna", 1_737_000.4),
    ("Luna", 1_737_000.4),
];

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PolylinePlugin)
        .add_plugins(EguiPlugin)
        .add_systems(Startup, setup)
        .init_resource::<FollowedBody>()
        .add_systems(
            Update,
            (
                get_state,
                set_positions.after(get_state).after(update_camera),
                set_path_positions.after(update_camera),
                handle_mouse_scroll.before(compute_camera_position),
                handle_mouse_drags.before(compute_camera_position),
                compute_camera_position.before(update_camera),
                update_camera,
                update_ui,
            ),
        )
        .run();
}

#[derive(Component)]
struct CalcutedOrbit(nbody_simd::Vec3<Simd<f64, 64>>);

#[derive(Component)]
struct SystemBody(usize);

#[derive(Resource)]
struct System {
    system: nbody_simd::System,
    state: nbody_simd::SystemState,
}

#[derive(Component)]
struct ComputedPath(nbody_simd::Vec3<Simd<f64, 64>>);

#[derive(Component)]
struct ParentBody(usize);

fn get_state(mut system: ResMut<System>, time: Res<Time>) {
    system.state = system
        .system
        .state_at(time.elapsed_secs_f64() * 60.0 * 60.0 * 24.0 * 1.0);
}

fn compute_camera_position(
    mut camera: ResMut<UniversalCamera>,
    system: Res<System>,
    followed: Res<FollowedBody>,
) {
    if let Some(body) = followed.0 {
        let position = system.state.planet_and_moon_positions.get(body);
        camera.center = UniversalPos::from(position);
    }

    camera.compute_position();
}

const AU: f64 = 1.495978707e11;

fn set_path_positions(
    camera: Res<UniversalCamera>,
    mut polylines: ResMut<Assets<Polyline>>,
    system: Res<System>,
    query: Query<(&ComputedPath, &PolylineHandle, Option<&ParentBody>)>,
) {
    let positions = system.state.planet_and_moon_positions;

    for (ComputedPath(path), handle, parent) in query.iter() {
        let line = polylines.get_mut(&handle.0).unwrap();
        line.vertices.clear();
        for i in 0..64 {
            let pos = parent
                .map(|&ParentBody(parent)| positions.get(parent))
                .unwrap_or_default()
                + path.get(i);

            let pos = pos - camera.position.as_vec3();
            line.vertices
                // Move the lines closer to the camera for better stability on opengl.
                .push(convert_vec(pos / (AU / 1000.0)).as_vec3());
        }
    }
}

fn set_positions(
    system: Res<System>,
    mut query: Query<(&SystemBody, &mut Transform, &BodyRadius)>,
    camera: Res<UniversalCamera>,
) {
    let positions = system.state.planet_and_moon_positions;

    let get_pos = |i| {
        let pos = positions.get(i);
        let pos = pos - camera.position.as_vec3();
        let pos = convert_vec(pos);
        let length = pos.length();
        (pos.as_vec3(), length)
    };

    for (body, mut transform, radius) in query.iter_mut() {
        let (pos, distance) = get_pos(body.0);
        *transform = transform
            .with_translation(pos)
            .with_scale(Vec3::splat((radius.0).max(distance / 200.0) as f32));
    }
}

fn setup(
    mut commands: Commands,
    mut polyline_materials: ResMut<Assets<PolylineMaterial>>,
    mut polylines: ResMut<Assets<Polyline>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let system = nbody_simd::System::sol();

    commands.insert_resource(UniversalCamera {
        center: Default::default(),
        distance: AU,
        position: Default::default(),
        pitch: 45.0_f64.to_radians(),
        yaw: 0.0_f64.to_radians(),
    });

    commands.insert_resource(System {
        system,
        state: system.state_at(0.0),
    });

    let mut sphere_mesh = meshes.add(Sphere::new(1.0).mesh().build());

    let colour = Color::hsl(55.0, 0.75, 1.5).to_linear();

    for i in 0..8 {
        let (name, radius) = PLANET_INFO[i];

        let positions = nbody_simd::orbital_position_from_mean_anomaly(
            OrbitParams::from_array([system.planets.get(i); 64]),
            Simd::from_array(std::array::from_fn(|i| {
                i as f64 / 63.0 * std::f64::consts::TAU
            })),
            sleef::f64x::sincos_u35,
        );

        commands.spawn((
            ComputedPath(positions),
            PolylineBundle {
                polyline: PolylineHandle(polylines.add(Polyline::default())),
                material: PolylineMaterialHandle(polyline_materials.add(PolylineMaterial {
                    width: 1.0,
                    color: colour,
                    perspective: false,
                    ..Default::default()
                })),
                ..Default::default()
            },
        ));

        commands.spawn((
            Mesh3d(sphere_mesh.clone()),
            MeshMaterial3d(materials.add(StandardMaterial { ..default() })),
            Transform::IDENTITY,
            SystemBody(i),
            Name::new(name),
            BodyRadius(radius),
        ));

        for j in 0..8 {
            if system.moon_parent_swizzles[j] != i {
                continue;
            }

            let position = nbody_simd::orbital_position_from_mean_anomaly(
                OrbitParams::from_array([system.moons.get(j); 64]),
                Simd::from_array(std::array::from_fn(|i| {
                    i as f64 / 63.0 * std::f64::consts::TAU
                })),
                sleef::f64x::sincos_u35,
            );

            commands.spawn((
                ComputedPath(position),
                ParentBody(i),
                PolylineBundle {
                    polyline: PolylineHandle(polylines.add(Polyline::default())),
                    material: PolylineMaterialHandle(polyline_materials.add(PolylineMaterial {
                        width: 1.0,
                        color: colour,
                        perspective: false,
                        ..Default::default()
                    })),
                    ..Default::default()
                },
            ));
        }
    }

    for i in 8..16 {
        let (name, radius) = PLANET_INFO[i];

        commands.spawn((
            Mesh3d(sphere_mesh.clone()),
            MeshMaterial3d(materials.add(StandardMaterial { ..default() })),
            Transform::IDENTITY,
            SystemBody(i),
            Name::new(name),
            BodyRadius(radius),
        ));
    }

    commands.spawn((Camera3dBundle::default(),));
}

fn update_camera(mut trans: Query<&mut Transform, With<Camera3d>>, camera: Res<UniversalCamera>) {
    let mut t = trans.single_mut();
    *t = Transform::from_translation(Vec3::ZERO).looking_at(-camera.view_dir().as_vec3(), Vec3::Y);
}

fn handle_mouse_drags(
    mut mouse_motion_events: EventReader<MouseMotion>,
    mut camera: ResMut<UniversalCamera>,
    buttons: Res<ButtonInput<MouseButton>>,
) {
    let sensitivity = Vec2::splat(2.0);

    let mut delta = Vec2::ZERO;
    for event in mouse_motion_events.read() {
        delta += event.delta;
    }

    if buttons.pressed(MouseButton::Left) {
        camera.rotate_yaw_pitch(
            -0.1 * delta.x * sensitivity.x,
            -0.1 * delta.y * sensitivity.y,
        );
    }

    if buttons.pressed(MouseButton::Right) {
        camera.rotate_yaw_pitch(
            -0.1 * delta.x * sensitivity.x,
            -0.1 * delta.y * sensitivity.y,
        );
    }
}

fn handle_mouse_scroll(
    mut mouse_wheel_events: EventReader<MouseWheel>,
    mut camera: ResMut<UniversalCamera>,
) {
    for mouse_wheel_event in mouse_wheel_events.read() {
        let factor = match mouse_wheel_event.unit {
            MouseScrollUnit::Line => 1.0,
            MouseScrollUnit::Pixel => 0.005,
        };
        camera.distance *= (1.0 + mouse_wheel_event.y as f64 * -0.1 * factor);
    }
}

fn update_ui(mut contexts: EguiContexts, mut followed: ResMut<FollowedBody>) {
    egui::Window::new("Hello").show(contexts.ctx_mut(), |ui| {
        if ui.button("None").clicked() {
            followed.0 = None;
        }
        for i in 0..16 {
            if ui
                .button(&if let Some((name, _)) = PLANET_INFO.get(i) {
                    name.to_string()
                } else {
                    format!("{}", i)
                })
                .clicked()
            {
                followed.0 = Some(i);
            }
        }
    });
}
