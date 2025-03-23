#![feature(portable_simd, mpmc_channel)]

use std::simd::num::SimdFloat;

mod controls;
mod paths;
mod rendering_prep;
mod setup;
mod ui;

use controls::*;
use paths::*;
use rendering_prep::*;
use setup::*;
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
struct SphereMesh(Handle<Mesh>);

#[derive(Resource, Default)]
struct SelectedBurn(Option<Entity>);

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
            (self.pitch + (pitch as f64) / 100.0).clamp(f64::EPSILON, std::f64::consts::PI);
        self.yaw += (yaw as f64) / 100.0;
    }
}

fn convert_vec(vec: nbody_simd::Vec3<f64>) -> DVec3 {
    DVec3::new(vec.x, vec.y, vec.z)
}

#[derive(Component)]
struct SystemStar;

use std::simd::Simd;

#[derive(Component)]
struct BurnPreviewLocation;

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
            max_segments: 1_000_000,
        })
        .init_resource::<CursorOnUiElement>()
        .init_resource::<SystemTime>()
        .init_resource::<FollowedBody>()
        .init_resource::<ReferenceFrameBody>()
        .init_resource::<SelectedBurn>()
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
                set_body_positions.after(get_state).after(update_camera),
                set_body_path_positions.after(update_camera),
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
        self.hue += 360.0 / 4.4;
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

struct PathBatch {
    positions: Vec<UniversalPos>,
    adapted_positions: Vec<UniversalPos>,
    render_positions: Vec<UniversalPos>,
    velocities: Vec<nbody_simd::Vec3<f64>>,
    min: UniversalPos,
    max: UniversalPos,
    step: f64,
}

impl PathBatch {
    fn duration(&self) -> f64 {
        self.positions.len() as f64 * self.step
    }
}

#[derive(Component)]
struct ShipPath {
    start: f64,
    start_pos: UniversalPos,
    start_vel: nbody_simd::Vec3<f64>,
    batches: Vec<PathBatch>,
    total_duration: f64,
}

impl ShipPath {
    fn clear(&mut self) {
        self.batches.clear();
        self.total_duration = 0.0;
    }
}

#[derive(Component)]
struct Burn {
    vector: nbody_simd::Vec3<f64>,
}

fn receive_trajectories(
    mut query: Query<(&mut ShipPath, &TrajectoryReceiver)>,
    reference_frame: Res<ReferenceFrameBody>,
    system: Res<System>,
) {
    for (mut path, recv) in query.iter_mut() {
        while let Ok(batch) = recv.inner.try_recv() {
            if batch.round != recv.expected_round {
                path.clear();
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
            path.total_duration += batch.duration();
            path.batches.push(batch);
        }
    }
}

#[derive(Component)]
struct TrajectoryReceiver {
    expected_round: usize,
    inner: Receiver<TrajectoryBatch>,
    burn_tx: Sender<(nbody_simd::Vec3<f64>, usize)>,
    handle: bevy::tasks::Task<()>,
}

use async_channel::{Receiver, Sender, bounded};

#[derive(Debug)]
struct TrajectoryBatch {
    round: usize,
    positions: Vec<UniversalPos>,
    velocities: Vec<nbody_simd::Vec3<f64>>,
    collides: bool,
    step: f64,
}

async fn trajectory_calculator(
    output: Sender<TrajectoryBatch>,
    commands: Receiver<(nbody_simd::Vec3<f64>, usize)>,
    start: f64,
    starting_pos: UniversalPos,
    starting_vel: nbody_simd::Vec3<f64>,
    system: nbody_simd::System,
) {
    let batch_size = 10_000;
    let accel_per_sec = 9.8;

    struct CalculatorState {
        pos: UniversalPos,
        vel: nbody_simd::Vec3<f64>,
        iteration: usize,
        round: usize,
        time: f64,
        burn_dir: nbody_simd::Vec3<f64>,
        burn_delta_v: f64,
    }

    let mut calc_state = CalculatorState {
        pos: starting_pos,
        vel: starting_vel,
        time: start,
        iteration: Default::default(),
        round: Default::default(),
        burn_dir: Default::default(),
        burn_delta_v: Default::default(),
    };

    let mut wait_for_next = true;

    loop {
        if wait_for_next {
            let (burn, new_round) = if let Ok((burn, new_round)) = commands.recv().await {
                (burn, new_round)
            } else {
                return;
            };
            let burn_delta_v = burn.length();
            calc_state = CalculatorState {
                pos: starting_pos,
                vel: starting_vel + burn,
                iteration: 0,
                round: new_round,
                time: start,
                burn_dir: burn / burn_delta_v,
                burn_delta_v,
            };
        }

        while let Ok((burn, new_round)) = commands.try_recv() {
            let burn_delta_v = burn.length();
            calc_state = CalculatorState {
                pos: starting_pos,
                vel: starting_vel,
                iteration: 0,
                round: new_round,
                time: start,
                burn_dir: burn / burn_delta_v,
                burn_delta_v,
            };
        }

        let mut positions = Vec::with_capacity(batch_size);
        let mut velocities = Vec::with_capacity(batch_size);

        let mut state = system.state_at(calc_state.time);
        let nearest = ((state.planet_and_moon_positions
            - nbody_simd::Vec3::splat(calc_state.pos.as_vec3()))
        .length_squared())
        .reduce_min()
        .sqrt();

        let seconds_to_nearest_body = nearest / convert_vec(calc_state.vel).length();

        // Large timesteps are allowed but we should only reach 90% of the way to the nearest body with them.
        let timestep =
            ((seconds_to_nearest_body / batch_size as f64) * 0.9).clamp(1.0, 24.0 * 60.0 * 60.0);
        let mut collides = false;
        for i in 0..batch_size {
            positions.push(calc_state.pos);
            velocities.push(calc_state.vel);

            if state.collides(calc_state.pos.as_vec3()) {
                collides = true;
                break;
            }

            let mut accel = state.acceleration_at(calc_state.pos.as_vec3()) * timestep;

            if calc_state.burn_delta_v > 0.0 {
                let burn_amount = (timestep * accel_per_sec).min(calc_state.burn_delta_v);
                accel += calc_state.burn_dir * burn_amount;
                calc_state.burn_delta_v -= burn_amount;
            }

            calc_state.vel += accel;
            calc_state.pos += calc_state.vel * timestep;
            state = system.state_at(calc_state.time + timestep * (i + 1) as f64);
        }
        calc_state.iteration += batch_size;
        calc_state.time += batch_size as f64 * timestep;
        let _ = output
            .send(TrajectoryBatch {
                round: calc_state.round,
                positions,
                velocities,
                collides,
                step: timestep,
            })
            .await;

        wait_for_next = calc_state.iteration >= 1_000_000 || collides;
    }
}

#[derive(Resource, Default)]
struct CursorOnUiElement(Option<Entity>);
/*
#[test]
fn test_trajectory_thread() {
    let (output_tx, output_rx) = bounded(100);
    let (burn_tx, burn_rx) = bounded(100);
    let pos = UniversalPos::new_3(57_909_048_000.0 / 2.0, 0.0, 0.0);
    let vel = nbody_simd::Vec3::new(0.0, 0.0, 100000.0);

    let task_pool = bevy::tasks::TaskPool::new();

    task_pool.spawn(
        trajectory_calculator(output_tx, burn_rx, 0.0, pos, vel, nbody_simd::System::sol())
    ).detach();

    task_pool.spawn(async move {

        for i in 0..10 {
            dbg!(output_rx.recv_blocking().unwrap());
        }

        burn_tx
            .send_blocking((nbody_simd::Vec3::new(0.0, 50_000.0, 0.0), 1))
            .unwrap();

        dbg!(());

        for i in 0..10 {
            dbg!(output_rx.recv_blocking().unwrap());
        }

        panic!();
    }).
}
*/

fn create_transform<S: Fn(f64) -> f64>(
    pos: UniversalPos,
    camera: &UniversalCamera,
    scale: S,
) -> Transform {
    let pos = pos - camera.position;
    let pos = convert_vec(pos / camera.distance);
    let distance = pos.length();
    let pos = pos.as_vec3();

    Transform::from_translation(pos).with_scale(Vec3::splat(scale(distance) as f32))
}
