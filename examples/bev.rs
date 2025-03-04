#![feature(portable_simd)]

use bevy::input::mouse::{MouseMotion, MouseScrollUnit, MouseWheel};
use bevy::math::DVec3;
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, egui};
use bevy_polyline::prelude::*;
use nbody_simd::{OrbitParams, UniversalPos};

#[derive(Resource, Default)]
struct SystemTime(f64);

#[derive(Component)]
struct BodyRadius(f64);

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

#[derive(Component)]
struct SystemStar;

use std::simd::Simd;

const PLANET_INFO: &[(&str, f64, &str)] = &[
    ("Mercury", 2_439_700.0, "2k_mercury.jpg"),
    ("Venus", 6_051_800.0, "2k_venus_atmosphere.jpg"),
    ("Earth", 6_371_000.0, "2k_earth_daymap.jpg"),
    ("Mars", 3_389_500.0, "2k_mars.jpg"),
    ("Jupiter", 69_911_000.0, "2k_jupiter.jpg"),
    ("Saturn", 69_911_000.0, "2k_saturn.jpg"),
    ("Uranus", 69_911_000.0, "2k_uranus.jpg"),
    ("Neptune", 69_911_000.0, "2k_neptune.jpg"),
    ("Luna", 1_737_000.4, "2k_moon.jpg"),
    ("Phobos", 11_080.0, "phobos.jpg"),
    ("Deimos", 6_270.0, "deimos.jpg"),
    ("Io", 1_821_600.0, "io.jpg"),
    ("Europa", 1_560_800.0, "europa.png"),
    ("Ganymede", 2_634_100.0, "ganymede_2k_downscaled.png"),
    ("Callisto", 2_410_300.0, "callisto.jpg"),
    ("Titan", 2_574_730.0, "2k_titan.png"),
];

// As values get converted to f32 for rendering, it's important to scale them down so they're not crazy high.
// This helps prevent visual errors, mostly on the web/opengl.
const SCALE: f64 = AU / 100_000.0;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::BLACK))
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
        .init_resource::<SystemTime>()
        .init_resource::<FollowedBody>()
        .add_plugins(DefaultPlugins)
        .add_plugins(PolylinePlugin)
        .add_plugins(EguiPlugin)
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                get_state,
                set_star_positions.after(update_camera),
                set_positions.after(get_state).after(update_camera),
                set_path_positions.after(update_camera),
                set_ship_path_positions.after(update_camera),
                handle_mouse_scroll.before(compute_camera_position),
                handle_mouse_drags.before(compute_camera_position),
                compute_camera_position.before(update_camera),
                update_camera,
                get_closest_path_point.after(update_camera),
                update_ui,
                recalulate_burns.after(update_ui),
            ),
        )
        .run();
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

fn set_ship_path_positions(
    camera: Res<UniversalCamera>,
    mut polylines: ResMut<Assets<Polyline>>,
    paths: Query<(&ShipPath, &PolylineHandle)>,
    time: Res<SystemTime>,
) {
    for (path, handle) in paths.iter() {
        let line = polylines.get_mut(&handle.0).unwrap();
        line.vertices.clear();

        for point in path.positions.iter().step_by(10).chain(
            path.positions
                .get(path.positions.len() - 1..)
                .iter()
                .flat_map(|&inner| inner),
        ) {
            let pos = *point - camera.position;
            line.vertices
                // Move the lines closer to the camera for better stability on opengl.
                .push(convert_vec(pos / SCALE).as_vec3());
        }
    }
}
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
        let parent_pos = convert_vec(
            parent
                .map(|&ParentBody(parent)| positions.get(parent))
                .unwrap_or_default(),
        );
        for point in path {
            let pos = (parent_pos + point) - convert_vec(camera.position.as_vec3());
            line.vertices
                // Move the lines closer to the camera for better stability on opengl.
                .push((pos / SCALE).as_vec3());
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
        let pos = convert_vec(pos / SCALE);
        let length = pos.length();
        (pos.as_vec3(), length)
    };

    for (body, mut transform, radius) in query.iter_mut() {
        let (pos, distance) = get_pos(body.0);
        *transform = transform
            .with_translation(pos)
            .with_scale(Vec3::splat((radius.0 / SCALE).max(distance / 200.0) as f32));
    }
}

fn set_star_positions(
    mut query: Query<(&mut Transform, &BodyRadius), With<SystemStar>>,
    camera: Res<UniversalCamera>,
) {
    for (mut transform, radius) in query.iter_mut() {
        let pos = -convert_vec(camera.position.as_vec3()) / SCALE;
        let distance = pos.length();
        *transform = transform
            .with_translation(pos.as_vec3())
            .with_scale(Vec3::splat((radius.0 / SCALE).max(distance / 75.0) as f32));
    }
}

fn setup(
    mut commands: Commands,
    mut polyline_materials: ResMut<Assets<PolylineMaterial>>,
    mut polylines: ResMut<Assets<Polyline>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
    let sphere_mesh = asset_server.load("planet.glb#Mesh0/Primitive0");
    let saturn_rings = asset_server.load("saturn_rings.glb#Mesh0/Primitive0");

    let system = nbody_simd::System::sol();

    let colour = Color::hsl(55.0, 0.75, 1.5).to_linear();

    {
        let mut pos = UniversalPos::new_3(57_909_048_000.0 / 2.0, 0.0, 0.0);
        let mut vel = nbody_simd::Vec3::new(0.0, 0.0, 100000.0);
        let mut points = vec![pos];
        let mut velocities = vec![vel];
        let timestep = 100.0;
        for i in 0..100_000 {
            vel += system
                .state_at(i as f64 * timestep)
                .acceleration_at(pos.as_vec3())
                * timestep;
            pos += vel * timestep;
            points.push(pos);
            velocities.push(vel);
        }
        commands.spawn((
            ShipPath {
                start: 0.0,
                velocities,
                positions: points,
            },
            PolylineBundle {
                polyline: PolylineHandle(polylines.add(Polyline::default())),
                material: PolylineMaterialHandle(polyline_materials.add(PolylineMaterial {
                    width: 1.0,
                    color: Color::hsl(90.0, 1.0, 0.5).to_linear(),
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
        let (name, radius, image_filename) = PLANET_INFO[i];

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

    for i in 8..16 {
        let (name, radius, image_filename) = PLANET_INFO[i];

        commands.spawn((
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
) {
    if contexts.ctx_mut().is_using_pointer() {
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

fn update_ui(
    mut contexts: EguiContexts,
    mut followed: ResMut<FollowedBody>,
    mut time: ResMut<SystemTime>,
    mut burns: Query<(Entity, &mut Burn)>,
    mut commands: Commands,
) {
    egui::Window::new("Hello").show(contexts.ctx_mut(), |ui| {
        ui.add(egui::Slider::new(&mut time.0, 0.0..=100_000_000.0));

        for (entity, mut burn) in burns.iter_mut() {
            ui.label("Burn");
            if ui.button("delete").clicked() {
                commands.entity(entity).despawn();
            }
            burn.dirty |= ui
                .add(egui::Slider::new(
                    &mut burn.vector.x,
                    -100_000.0..=100_000.0_f64,
                ))
                .changed();
            burn.dirty |= ui
                .add(egui::Slider::new(
                    &mut burn.vector.y,
                    -100_000.0..=100_000.0_f64,
                ))
                .changed();
            burn.dirty |= ui
                .add(egui::Slider::new(
                    &mut burn.vector.z,
                    -100_000.0..=100_000.0_f64,
                ))
                .changed();
        }

        if ui.button("None").clicked() {
            followed.0 = None;
        }

        for i in 0..16 {
            if ui
                .button(&if let Some((name, ..)) = PLANET_INFO.get(i) {
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

#[derive(Component)]
struct ShipPathBefore;

#[derive(Component)]
struct ShipPathAfter;

fn get_closest_path_point(
    mut contexts: EguiContexts,
    camera: Res<UniversalCamera>,
    render_cam: Single<(&Camera, &GlobalTransform)>,
    paths: Query<&ShipPath>,
    mut time: ResMut<SystemTime>,
    primary_window: Option<Single<&Window>>,
    buttons: Res<ButtonInput<MouseButton>>,
    system: Res<System>,
    mut polyline_materials: ResMut<Assets<PolylineMaterial>>,
    mut polylines: ResMut<Assets<Polyline>>,
    mut commands: Commands,
) {
    if contexts.ctx_mut().is_using_pointer() {
        return;
    }

    let mut cursor_position =
        if let Some(cursor_position) = primary_window.and_then(|window| window.cursor_position()) {
            cursor_position
        } else {
            return;
        };

    let (render_cam, render_cam_transform) = *render_cam;

    let world_to_ndc =
        render_cam.clip_from_view() * render_cam_transform.compute_matrix().inverse();

    let target_size = render_cam.logical_viewport_size().unwrap();

    // flip the cursor position on the y axis to match
    cursor_position.y = target_size.y - cursor_position.y;

    let compare_dist = 20.0;

    let picked_position = paths
        .iter()
        .flat_map(|(path)| {
            path.positions
                .iter()
                .enumerate()
                .map(move |(i, pos)| (i, path, pos))
        })
        .filter_map(|(i, path, &pos)| {
            let pos = pos - camera.position;
            let world_pos = convert_vec(pos / SCALE).as_vec3();
            let ndc_pos = world_to_ndc.project_point3(world_pos);
            if ndc_pos.z < 0.0 || ndc_pos.z > 1.0 {
                return None;
            }
            let screen_pos = (ndc_pos.truncate() + Vec2::ONE) / 2.0 * target_size;

            let distance_sq = cursor_position.distance_squared(screen_pos);

            if distance_sq > compare_dist * compare_dist {
                return None;
            }

            Some((i, path, distance_sq))
        })
        .min_by_key(|&(_, _, distance_sq)| ordered_float::OrderedFloat(distance_sq));

    if let Some((index, origin_path, _)) = picked_position {
        if buttons.just_pressed(MouseButton::Left) {
            let start = index as f64 * 100.0 + origin_path.start;
            time.0 = start;
            let mut vel = origin_path.velocities[index];
            let mut pos = origin_path.positions[index];
            commands.spawn((
                ShipPath {
                    start,
                    velocities: vec![vel],
                    positions: vec![pos],
                },
                Burn {
                    vector: Default::default(),
                    dirty: false,
                },
                PolylineBundle {
                    polyline: PolylineHandle(polylines.add(Polyline::default())),
                    material: PolylineMaterialHandle(polyline_materials.add(PolylineMaterial {
                        width: 1.0,
                        color: Color::hsl(90.0, 1.0, 0.5).to_linear(),
                        perspective: false,
                        ..Default::default()
                    })),
                    ..Default::default()
                },
            ));
        }
    }
}

#[derive(Component)]
struct ShipPath {
    start: f64,
    velocities: Vec<nbody_simd::Vec3<f64>>,
    positions: Vec<UniversalPos>,
}

#[derive(Component)]
struct Burn {
    vector: nbody_simd::Vec3<f64>,
    dirty: bool,
}

fn recalulate_burns(mut query: Query<(&mut Burn, &mut ShipPath)>, system: Res<System>) {
    for (mut burn, mut path) in query.iter_mut() {
        if !burn.dirty {
            continue;
        }
        burn.dirty = false;

        let mut pos = path.positions[0];
        let mut vel = path.velocities[0] + burn.vector;
        path.positions.truncate(1);
        path.velocities.truncate(1);
        let timestep = 100.0;
        for i in 0..100_000 {
            let state = system.system.state_at(i as f64 * timestep + path.start);
            if state.collides(pos.as_vec3()) {
                break;
            }
            vel += state.acceleration_at(pos.as_vec3()) * timestep;
            pos += vel * timestep;
            path.positions.push(pos);
            path.velocities.push(vel);
        }
    }
}
