#![feature(portable_simd, mpmc_channel)]

mod paths;
mod ui;

use paths::*;
use ui::update_ui;

use bevy::input::mouse::{MouseMotion, MouseScrollUnit, MouseWheel};
use bevy::math::DVec3;
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, egui};
use bevy_polyline::prelude::*;
use nbody_simd::{OrbitParams, UniversalPos, UniversalScalar, sleef};

#[derive(Resource, Default)]
struct SystemTime(f64);

#[derive(Component)]
struct BodyRadius(f64);

#[derive(Resource, Default)]
struct FollowedBody(Option<usize>);

#[derive(Resource, Default)]
struct ReferenceFrameBody(Option<usize>);

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
        self.pitch =
            (self.pitch + (pitch as f64) / 100.0).clamp(std::f64::EPSILON, std::f64::consts::PI);
        self.yaw += (yaw as f64) / 100.0;
    }
}

fn convert_vec(vec: nbody_simd::Vec3<f64>) -> DVec3 {
    DVec3::new(vec.x, vec.y, vec.z)
}

#[derive(Component)]
struct SystemStar;

use std::simd::Simd;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::BLACK))
        .insert_resource(ColorChooser { hue: 16.666666 })
        .insert_resource(UniversalCamera {
            center: Default::default(),
            distance: AU,
            position: Default::default(),
            pitch: 45.0_f64.to_radians(),
            yaw: 0.0_f64.to_radians(),
        })
        .insert_resource({
            let system = nbody_simd::System::sol();
            System {
                system,
                state: system.state_at(0.0),
            }
        })
        .insert_resource(PathSettings {
            decimation: 10,
            selection_decimation: 10,
            max_segments: 1_000_000,
        })
        .init_resource::<CursorOnUiElement>()
        .init_resource::<SystemTime>()
        .init_resource::<FollowedBody>()
        .init_resource::<ReferenceFrameBody>()
        .add_plugins(DefaultPlugins)
        .add_plugins(PolylinePlugin)
        .add_plugins(EguiPlugin)
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                get_state,
                set_star_positions.after(update_camera),
                set_universal_positions.after(update_camera),
                set_positions.after(get_state).after(update_camera),
                set_path_positions.after(update_camera),
                set_ship_path_positions
                    .after(update_camera)
                    .after(get_state)
                    .after(receive_trajectories),
                handle_mouse_scroll.before(compute_camera_position),
                handle_mouse_drags.before(compute_camera_position),
                compute_camera_position.before(update_camera),
                update_camera,
                get_closest_path_point.after(update_camera),
                update_ui,
                receive_trajectories
                    .after(update_ui)
                    .after(adjust_trajectory),
                trajectory_handles.after(set_universal_positions),
                adjust_trajectory,
                adapt_paths_to_new_reference_frame.after(update_ui),
            ),
        )
        .run();
}

#[derive(Resource)]
struct ColorChooser {
    hue: f32,
}

impl ColorChooser {
    fn next(&mut self) -> LinearRgba {
        let col = Color::hsl(self.hue, 0.75, 0.5).to_linear();
        self.hue += (360.0 / 4.4);
        col
    }
}

#[derive(Component)]
struct SystemBody(usize);

#[derive(Resource)]
struct System {
    system: nbody_simd::System,
    state: nbody_simd::SystemState,
}

#[derive(Component)]
struct ComputedPath([DVec3; 512]);

#[derive(Component)]
struct UniversalObjectPos {
    pos: UniversalPos,
    offset: DVec3,
}

#[derive(Resource)]
struct PathSettings {
    decimation: usize,
    selection_decimation: usize,
    max_segments: usize,
}

impl ComputedPath {
    fn compute(body: OrbitParams<f64>) -> Self {
        let get_for_offset = |start| {
            nbody_simd::orbital_position_from_mean_anomaly(
                OrbitParams::from_array([body; 64]),
                Simd::from_array(std::array::from_fn(|i| {
                    (start + i) as f64 / 511.0 * std::f64::consts::TAU
                })),
                sleef::f64x::sincos_u35,
            )
        };

        let chunks = [
            get_for_offset(0),
            get_for_offset(64),
            get_for_offset(128),
            get_for_offset(192),
            get_for_offset(256),
            get_for_offset(320),
            get_for_offset(384),
            get_for_offset(448),
        ];

        Self(std::array::from_fn(|i| {
            DVec3::new(
                chunks[i / 64].x[i % 64],
                chunks[i / 64].y[i % 64],
                chunks[i / 64].z[i % 64],
            )
        }))
    }
}

#[derive(Component)]
struct ParentBody(usize);

fn get_state(mut system: ResMut<System>, time: Res<SystemTime>) {
    system.state = system.system.state_at(time.0);
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

#[inline]
fn get_reference_frame_offset(
    start_time: f64,
    system: &System,
    body: usize,
    step: f64,
) -> impl Iterator<Item = nbody_simd::Vec3<f64>> {
    (0..).flat_map(move |run| {
        (-system.system.position_for_single_body(
            body,
            Simd::from_array(std::array::from_fn(|i| {
                (run * 64 + i) as f64 * step + start_time
            })),
        ))
        .as_array()
    })
}

fn set_universal_positions(
    mut query: Query<(&UniversalObjectPos, &mut Transform)>,
    camera: Res<UniversalCamera>,
) {
    for (univ_pos, mut transform) in query.iter_mut() {
        let pos = univ_pos.pos - camera.position;
        let pos = convert_vec(pos / camera.distance);
        let distance = pos.length();
        let pos = (pos + univ_pos.offset / 200.0).as_vec3();
        *transform = transform
            .with_translation(pos)
            .with_scale(Vec3::splat((distance / 200.0) as f32));
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
        let pos = convert_vec(pos / camera.distance);
        let length = pos.length();
        (pos.as_vec3(), length)
    };

    for (body, mut transform, radius) in query.iter_mut() {
        let (pos, distance) = get_pos(body.0);
        *transform = transform.with_translation(pos).with_scale(Vec3::splat(
            (radius.0 / camera.distance).max(distance / 200.0) as f32,
        ));
    }
}

#[derive(Resource)]
struct SphereMesh(Handle<Mesh>);

fn set_star_positions(
    mut query: Query<(&mut Transform, &BodyRadius), With<SystemStar>>,
    camera: Res<UniversalCamera>,
) {
    for (mut transform, radius) in query.iter_mut() {
        let pos = -convert_vec(camera.position.as_vec3()) / camera.distance;
        let distance = pos.length();
        *transform = transform
            .with_translation(pos.as_vec3())
            .with_scale(Vec3::splat(
                (radius.0 / camera.distance).max(distance / 75.0) as f32,
            ));
    }
}

fn setup(
    mut commands: Commands,
    mut polyline_materials: ResMut<Assets<PolylineMaterial>>,
    mut polylines: ResMut<Assets<Polyline>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    mut color_chooser: ResMut<ColorChooser>,
) {
    let planet_info = [
        ("Mercury", "2k_mercury.jpg"),
        ("Venus", "2k_venus_atmosphere.jpg"),
        ("Earth", "2k_earth_daymap.jpg"),
        ("Mars", "2k_mars.jpg"),
        ("Jupiter", "2k_jupiter.jpg"),
        ("Saturn", "2k_saturn.jpg"),
        ("Uranus", "2k_uranus.jpg"),
        ("Neptune", "2k_neptune.jpg"),
    ];

    let sphere_mesh = asset_server.load("planet.glb#Mesh0/Primitive0");
    let saturn_rings = asset_server.load("saturn_rings.glb#Mesh0/Primitive0");

    commands.insert_resource(SphereMesh(sphere_mesh.clone()));

    let system = nbody_simd::System::sol();

    let colour = Color::hsl(55.0, 0.75, 1.5).to_linear();

    {
        let mut pos = UniversalPos::new_3(57_909_048_000.0 / 2.0, 0.0, 0.0);
        let mut vel = nbody_simd::Vec3::new(0.0, 0.0, 100000.0);
        let start_pos = pos;
        let start_vel = vel;
        let mut positions = vec![pos];
        let mut velocities = vec![vel];
        let timestep = 10.0;
        for i in 0..1_000_000 {
            vel += system
                .state_at(i as f64 * timestep)
                .acceleration_at(pos.as_vec3())
                * timestep;
            pos += vel * timestep;
            positions.push(pos);
            velocities.push(vel);
        }
        commands.spawn((
            ShipPath {
                start: 0.0,
                start_pos,
                start_vel,
                total_duration: positions.len() as f64 * timestep,
                batches: vec![PathBatch {
                    positions,
                    velocities,
                    adapted_positions: Default::default(),
                    render_positions: Default::default(),
                    min: Default::default(),
                    max: Default::default(),
                    step: timestep
                }],
            },
            PolylineBundle {
                polyline: PolylineHandle(polylines.add(Polyline::default())),
                material: PolylineMaterialHandle(polyline_materials.add(PolylineMaterial {
                    width: 1.0,
                    color: color_chooser.next(),
                    perspective: false,
                    ..Default::default()
                })),
                ..Default::default()
            },
        ));
    }

    commands.spawn((
        Mesh3d(sphere_mesh.clone()),
        MeshMaterial3d(materials.add(StandardMaterial {
            perceptual_roughness: 1.0,
            base_color_texture: Some(asset_server.load("2k_sun.jpg")),
            unlit: true,
            ..Default::default()
        })),
        Transform::IDENTITY,
        Name::new("Sol"),
        BodyRadius(nbody_simd::System::SOL_RADIUS),
        SystemStar,
    ));

    for i in 0..8 {
        let (name, image_filename) = planet_info[i];
        let radius = nbody_simd::System::SOL_PLANETS[i].1;

        commands.spawn((
            ComputedPath::compute(system.planets.get(i)),
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

        let mut entity_commands = commands.spawn((
            Mesh3d(sphere_mesh.clone()),
            MeshMaterial3d(materials.add(StandardMaterial {
                perceptual_roughness: 1.0,
                base_color_texture: Some(asset_server.load(image_filename)),
                unlit: true,
                ..Default::default()
            })),
            Transform::IDENTITY,
            SystemBody(i),
            Name::new(name),
            BodyRadius(radius),
        ));

        if i == 2 {
            entity_commands.with_child((
                Mesh3d(sphere_mesh.clone()),
                MeshMaterial3d(materials.add(StandardMaterial {
                    perceptual_roughness: 1.0,
                    base_color_texture: Some(asset_server.load("clouds_alpha.png")),
                    unlit: true,
                    double_sided: true,
                    alpha_mode: bevy::render::alpha::AlphaMode::Blend,
                    ..Default::default()
                })),
                Transform::from_scale(Vec3::splat(1.015)),
            ));
        }

        if i == 5 {
            entity_commands.with_child((
                Mesh3d(saturn_rings.clone()),
                MeshMaterial3d(materials.add(StandardMaterial {
                    perceptual_roughness: 1.0,
                    base_color_texture: Some(asset_server.load("2k_saturn_ring_alpha.png")),
                    unlit: true,
                    double_sided: true,
                    alpha_mode: bevy::render::alpha::AlphaMode::Blend,
                    cull_mode: None,
                    ..Default::default()
                })),
            ));
        }

        for j in 0..8 {
            if system.moon_parent_swizzles[j] != i {
                continue;
            }

            commands.spawn((
                ComputedPath::compute(system.moons.get(j)),
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

    let moon_info = [
        ("Luna", "2k_moon.jpg"),
        ("Phobos", "phobos.jpg"),
        ("Deimos", "deimos.jpg"),
        ("Io", "io.jpg"),
        ("Europa", "europa.png"),
        ("Ganymede", "ganymede_2k_downscaled.png"),
        ("Callisto", "callisto.jpg"),
        ("Titan", "2k_titan.png"),
    ];

    for i in 0..8 {
        let (name, image_filename) = moon_info[i];
        let radius = nbody_simd::System::SOL_MOONS[i].1;

        commands.spawn((
            Mesh3d(sphere_mesh.clone()),
            MeshMaterial3d(materials.add(StandardMaterial {
                perceptual_roughness: 1.0,
                base_color_texture: Some(asset_server.load(image_filename)),
                unlit: true,
                ..Default::default()
            })),
            Transform::IDENTITY,
            SystemBody(i + 8),
            Name::new(name),
            BodyRadius(radius),
        ));
    }

    commands.spawn((Camera3d::default(),));
}

fn update_camera(mut trans: Query<&mut Transform, With<Camera3d>>, camera: Res<UniversalCamera>) {
    let mut t = trans.single_mut();
    *t = Transform::from_translation(Vec3::ZERO).looking_at(-camera.view_dir().as_vec3(), Vec3::Y);
}

fn handle_mouse_drags(
    mut mouse_motion_events: EventReader<MouseMotion>,
    mut camera: ResMut<UniversalCamera>,
    buttons: Res<ButtonInput<MouseButton>>,
    mut contexts: EguiContexts,
    cursor_on_ui_element: Res<CursorOnUiElement>,
) {
    let ctx_mut = if let Some(ctx_mut) = contexts.try_ctx_mut() {
        ctx_mut
    } else {
        return;
    };

    if ctx_mut.is_using_pointer() || cursor_on_ui_element.0.is_some() {
        return;
    }

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
        camera.distance *= 1.0 + mouse_wheel_event.y as f64 * -0.1 * factor;
    }
}

#[derive(Component)]
struct ShipPathBefore;

#[derive(Component)]
struct ShipPathAfter;

struct PathBatch {
    positions: Vec<UniversalPos>,
    adapted_positions: Vec<UniversalPos>,
    render_positions: Vec<UniversalPos>,
    velocities: Vec<nbody_simd::Vec3<f64>>,
    min: UniversalPos,
    max: UniversalPos,
    step: f64,
}

#[derive(Component)]
struct ShipPath {
    start: f64,
    start_pos: UniversalPos,
    start_vel: nbody_simd::Vec3<f64>,
    batches: Vec<PathBatch>,
    total_duration: f64,
}

#[derive(Component)]
struct Burn {
    vector: nbody_simd::Vec3<f64>,
}

fn receive_trajectories(
    mut query: Query<(&mut ShipPath, &TrajectoryReceiver)>,
    settings: Res<PathSettings>,
    reference_frame: Res<ReferenceFrameBody>,
    system: Res<System>,
) {
    for (mut path, recv) in query.iter_mut() {
        for batch in recv.inner.try_iter() {
            if batch.round != recv.expected_round {
                path.batches.clear();
                path.total_duration = 0.0;
                continue;
            }

            let mut batch = PathBatch {
                step: batch.step,
                adapted_positions: Vec::with_capacity(batch.positions.len()),
                positions: batch.positions,
                velocities: batch.velocities,
                render_positions: Vec::new(),
                min: UniversalPos::splat(UniversalScalar::MAX),
                max: UniversalPos::splat(UniversalScalar::MIN),
            };

            adapt_batch_to_reference_frame(
                path.start + path.total_duration,
                &mut batch,
                &reference_frame,
                &system,
            );
            path.total_duration += batch.positions.len() as f64 * batch.step;
            path.batches.push(batch);
        }
    }
}

#[derive(Component)]
struct TrajectoryReceiver {
    expected_round: usize,
    inner: Receiver<TrajectoryBatch>,
}

#[derive(Component)]
struct BurnTx(Sender<(nbody_simd::Vec3<f64>, usize)>);

use std::sync::mpmc::{Receiver, Sender, sync_channel};

struct TrajectoryBatch {
    round: usize,
    positions: Vec<UniversalPos>,
    velocities: Vec<nbody_simd::Vec3<f64>>,
    collides: bool,
    step: f64,
}

fn trajectory_calculator(
    output: Sender<TrajectoryBatch>,
    commands: Receiver<(nbody_simd::Vec3<f64>, usize)>,
    start: f64,
    starting_pos: UniversalPos,
    starting_vel: nbody_simd::Vec3<f64>,
    system: nbody_simd::System,
) {
    let mut pos = starting_pos;
    let mut vel = starting_vel;
    let mut iteration = 0;
    let mut round = 0;
    let mut wait_for_next = true;

    loop {
        if wait_for_next {
            let (burn, new_round) = if let Ok((burn, new_round)) = commands.recv() {
                (burn, new_round)
            } else {
                return;
            };
            pos = starting_pos;
            vel = starting_vel + burn;
            iteration = 0;
            round = new_round;
            wait_for_next = false;
        }

        while let Ok((burn, new_round)) = commands.try_recv() {
            pos = starting_pos;
            vel = starting_vel + burn;
            iteration = 0;
            round = new_round;
        }

        let mut positions = Vec::with_capacity(10000);
        let mut velocities = Vec::with_capacity(10000);

        let timestep = 10.0;
        let mut collides = false;
        for i in 0..10000 {
            positions.push(pos);
            velocities.push(vel);

            let state = system.state_at((iteration + i) as f64 * timestep + start);
            if state.collides(pos.as_vec3()) {
                collides = true;
                break;
            }
            vel += state.acceleration_at(pos.as_vec3()) * timestep;
            pos += vel * timestep;
        }
        iteration += 10000;
        output.send(TrajectoryBatch {
            round,
            positions,
            velocities,
            collides,
            step: 10.0,
        });

        wait_for_next = iteration >= 10_000_000 || collides;
    }
}

#[derive(Resource, Default)]
struct CursorOnUiElement(Option<Entity>);

#[test]
fn test_trajectory_thread() {
    let (output_tx, output_rx) = sync_channel(100);
    let (burn_tx, burn_rx) = sync_channel(100);
    let pos = UniversalPos::new_3(57_909_048_000.0 / 2.0, 0.0, 0.0);
    let vel = nbody_simd::Vec3::new(0.0, 0.0, 100000.0);

    std::thread::spawn(move || {
        trajectory_calculator(output_tx, burn_rx, 0.0, pos, vel, nbody_simd::System::sol());
    });

    for i in 0..10 {
        dbg!(output_rx.recv().unwrap().0);
    }

    burn_tx
        .send((nbody_simd::Vec3::new(0.0, 50_000.0, 0.0), 1))
        .unwrap();

    dbg!(());

    for i in 0..10 {
        dbg!(output_rx.recv().unwrap().0);
    }

    panic!();
}
