mod orbit_math;

use bevy::math::{DVec2, DVec3};
use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use bevy_egui::EguiPlugin;
use rand::{Rng, SeedableRng};
use std::collections::{BTreeMap, HashMap, VecDeque};
use std::f64::consts::TAU;
use std::ops::{Add, AddAssign, Sub};

fn main() {
    let updates_per_sec = 1000;

    App::new()
        .insert_resource(TimeSinceStartMs(0))
        .insert_resource(Timestep(
            /*(60 * 60 * 24 * 30) * */ 500_000 * (1000 / updates_per_sec),
        ))
        .insert_resource(FixedTime::new_from_secs(1.0 / updates_per_sec as f32))
        .insert_resource(CameraCenter(Default::default()))
        .add_plugins(DefaultPlugins)
        .add_plugin(EguiPlugin)
        .add_startup_system(setup)
        .add_system(update_pos)
        .add_system(update_ship_pos)
        .add_system(handle_camera)
        .add_system(update_body_scales)
        .add_system(update_ship_scales)
        .add_system(draw_ui)
        .add_system(update_time.in_schedule(CoreSchedule::FixedUpdate))
        .run();
}

#[derive(Resource)]
struct CameraCenter(FixedVec2);

#[derive(Component)]
struct StandardAcceleration(f64);

#[derive(Resource)]
struct Timestep(u64);

impl Timestep {
    fn as_secs(&self) -> f32 {
        self.0 as f32 / 1000.0
    }

    fn as_secs_f64(&self) -> f64 {
        self.0 as f64 / 1000.0
    }
}

#[derive(Resource)]
struct TimeSinceStartMs(u64);

impl TimeSinceStartMs {
    fn as_secs(&self) -> f32 {
        self.0 as f32 / 1000.0
    }

    fn as_secs_f64(&self) -> f64 {
        self.0 as f64 / 1000.0
    }
}

fn is_zero_u64(t: &u64) -> bool {
    *t == 0
}

#[derive(serde::Serialize, serde::Deserialize, Clone, Debug)]
struct SerializedBody {
    semi_major_axis_km: f64,
    eccentricity: f64,
    mass_kg: f64,
    radius_km: f64,
    #[serde(default, skip_serializing_if = "is_zero_u64")]
    population: u64,
    #[serde(default, skip_serializing_if = "is_zero_u64")]
    ice_kg: u64,
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    satelites: HashMap<String, SerializedBody>,
    periapsis_rotation_r: Option<f64>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    asteroid_belts: Vec<AsteroidBelt>,
}

#[derive(serde::Serialize, serde::Deserialize, Clone, Debug)]
struct AsteroidBelt {
    num: std::num::NonZeroU32,
    semi_major_axis_mean_km: f64,
    #[serde(default)]
    semi_major_axis_stddev: f64,
    eccentricity_mean: f64,
    #[serde(default)]
    eccentricity_stddev: f64,
    radius_mean_km: f64,
}

#[derive(serde::Serialize, serde::Deserialize, Clone, Debug)]
struct System {
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    bodies: BTreeMap<String, SerializedBody>,
    x: u64,
    y: u64,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    asteroid_belts: Vec<AsteroidBelt>,
}

type FixedScalar = fixed::types::I108F20;

#[derive(Clone, Copy, Default)]
struct FixedVec2 {
    x: FixedScalar,
    y: FixedScalar,
}

use az::Cast;

impl FixedVec2 {
    fn new_2(x: f64, y: f64) -> Self {
        Self::new(FixedScalar::from_num(x), FixedScalar::from_num(y))
    }

    fn new(x: FixedScalar, y: FixedScalar) -> Self {
        Self { x, y }
    }

    fn distance_squared(self, other: Self) -> f64 {
        (self - other).length_squared()
    }

    fn distance(self, other: Self) -> f64 {
        self.distance_squared(other).sqrt()
    }

    // This is essentially an add, but discouraged unless there's a good reason
    // (like treating one position as an orbit around another).
    fn offset_by_pos(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl Add<DVec2> for FixedVec2 {
    type Output = Self;

    fn add(self, other: DVec2) -> Self {
        Self {
            x: self.x + FixedScalar::from_num(other.x),
            y: self.y + FixedScalar::from_num(other.y),
        }
    }
}

impl AddAssign<DVec2> for FixedVec2 {
    fn add_assign(&mut self, other: DVec2) {
        *self = *self + other;
    }
}

impl Sub<FixedVec2> for FixedVec2 {
    type Output = DVec2;

    fn sub(self, other: Self) -> DVec2 {
        DVec2 {
            x: (self.x - other.x).cast(),
            y: (self.y - other.y).cast(),
        }
    }
}

#[derive(Resource)]
struct ParentBodies(HashMap<String, Entity>);

#[derive(Component)]
struct Position(FixedVec2);

#[derive(Component)]
struct Velocity(DVec2);

#[derive(Component)]
struct SateliteOf(Entity);

#[derive(Component)]
struct Populated(u64);

#[derive(Component)]
struct Passengers(u64);

#[derive(Component, Clone, Copy)]
struct OrbitalParams {
    semi_major_axis_m: f64,
    eccentricity: f64,
    mass_kg: f64,
    radius_m: f64,
    periapsis_rotation_r: f64,
    initial_offset_r: f64,
}

#[derive(Resource)]
struct GameRng(rand::rngs::SmallRng);

struct Context<'a> {
    bodies_map: HashMap<String, Entity>,
    bodies_vec: Vec<(Entity, u64)>,
    commands: Commands<'a, 'a>,
    meshes: &'a mut Assets<Mesh>,
    materials: &'a mut Assets<ColorMaterial>,
    rng: rand::rngs::SmallRng,
}

fn spawn_belts(belts: &[AsteroidBelt], satelites_of: Option<Entity>, context: &mut Context) {
    for belt in belts {
        let periapsis_rotation_r = context.rng.gen_range(0.0..TAU);

        let semi_major_axis_dist =
            rand_distr::Normal::new(belt.semi_major_axis_mean_km, belt.semi_major_axis_stddev)
                .unwrap();
        let eccentricity_dist =
            rand_distr::Normal::new(belt.eccentricity_mean, belt.eccentricity_stddev).unwrap();

        for i in 0..belt.num.get() {
            let name = format!("Asteroid {}", i);

            let radius = context.rng.gen_range(0.1..belt.radius_mean_km);

            let mass = 1.0E17 * radius;

            let body = &SerializedBody {
                semi_major_axis_km: context.rng.sample(semi_major_axis_dist),
                eccentricity: context.rng.sample(eccentricity_dist),
                mass_kg: mass,
                radius_km: radius,
                population: (radius * radius) as u64,
                ice_kg: context.rng.gen_range(0..500_000),
                satelites: HashMap::default(),
                periapsis_rotation_r: Some(periapsis_rotation_r),
                asteroid_belts: Vec::new(),
            };

            spawn_body(&name, body, satelites_of, context);
        }
    }
}

fn spawn_body(
    name: &String,
    body: &SerializedBody,
    satelite_of: Option<Entity>,
    context: &mut Context,
) {
    let circle_mesh = context.meshes.add(shape::Circle::new(1.0).into());

    let mut entity_commands = context.commands.spawn((
        MaterialMesh2dBundle {
            mesh: circle_mesh.clone().into(),
            material: context
                .materials
                .add(ColorMaterial::from(if satelite_of.is_none() {
                    Color::RED
                } else {
                    Color::ORANGE
                })),
            transform: Transform::from_scale(Vec3::splat(1.0)),
            ..default()
        },
        OrbitalParams {
            semi_major_axis_m: body.semi_major_axis_km * 1000.0,
            eccentricity: body.eccentricity,
            periapsis_rotation_r: body
                .periapsis_rotation_r
                .unwrap_or_else(|| context.rng.gen_range(0.0..TAU)),
            initial_offset_r: context.rng.gen_range(0.0..TAU),
            mass_kg: body.mass_kg,
            radius_m: body.radius_km * 1000.0,
        },
        Name::new(name.clone()),
    ));

    if body.population > 0 {
        entity_commands.insert(Populated(body.population));
    }

    if let Some(parent) = satelite_of {
        entity_commands.insert(SateliteOf(parent));
    }

    let entity = entity_commands.id();

    context.bodies_map.insert(name.clone(), entity);
    context.bodies_vec.push((entity, body.population));

    for (name, body) in body.satelites.iter() {
        spawn_body(name, body, Some(entity), context);
    }

    spawn_belts(&body.asteroid_belts, Some(entity), context);
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let mut rng = rand::rngs::SmallRng::from_seed([0; 32]);

    let mut camera = Camera2dBundle::default();
    camera.projection.scale = 200_000_000.0;
    camera.camera_2d.clear_color =
        bevy::core_pipeline::clear_color::ClearColorConfig::Custom(Color::BLACK);
    commands.spawn(camera);

    let x: BTreeMap<String, System> =
        toml::from_str(&std::fs::read_to_string("planets.toml").unwrap())
            .unwrap_or_else(|err| panic!("{}", err));

    let circle_mesh = meshes.add(shape::Circle::new(1.0).into());

    let mut bodies_map: HashMap<String, Entity> = HashMap::new();
    let mut bodies_vec: Vec<(Entity, u64)> = Vec::new();

    let mut context = Context {
        bodies_map: bodies_map,
        bodies_vec: bodies_vec,
        commands: commands,
        meshes: &mut meshes,
        materials: &mut materials,
        rng: rng,
    };

    for (name, system) in x {
        for (name, body) in system.bodies.iter() {
            spawn_body(name, body, None, &mut context);
        }

        spawn_belts(&system.asteroid_belts, None, &mut context);
    }

    context
        .commands
        .insert_resource(ParentBodies(context.bodies_map));
    context.commands.insert_resource(GameRng(context.rng));
}

fn lorentz_factor(velocity: f64) -> f64 {
    (1.0 - ((velocity * velocity) / (C * C))).sqrt()
}

const C: f64 = 299_792_458.0;
const G: f64 = 9.80665;
const LY: f64 = 9.4607E15;

fn rapidity(velocity: f64) -> f64 {
    (velocity / C).atanh()
}

// from http://gregsspacecalculations.blogspot.com/p/blog-page.html?a=9.80665&b=31555514.67800526&c=0.
fn stationary_time_to_velocity(velocity: f64, acceleration: f64) -> f64 {
    rapidity(velocity).sinh() * C / acceleration
}

fn ship_time_to_velocity(velocity: f64, acceleration: f64) -> f64 {
    rapidity(velocity) * C / acceleration
}

fn ship_delta_v_required(velocity: f64, acceleration: f64) -> f64 {
    ship_time_to_velocity(velocity, acceleration) * 2.0 * acceleration
}

fn stationary_distance_covered_while_accelerating(velocity: f64, acceleration: f64) -> f64 {
    (rapidity(velocity).cosh() - 1.0) * C * C / acceleration
}

fn fix_time(t: f64) -> f64 {
    t / 60.0 / 60.0 / 24.0 / 365.25
}

fn fix_distance(d: f64) -> f64 {
    d / 9.4607E15
}

fn stationary_travel_time(distance: f64, max_vel: f64, acceleration: f64) -> f64 {
    let burn_time = stationary_time_to_velocity(max_vel, acceleration) * 2.0;
    let burn_distance = stationary_distance_covered_while_accelerating(max_vel, acceleration) * 2.0;

    if burn_distance > distance {
        panic!()
    }

    burn_time + (distance - burn_distance) / max_vel
}

fn ship_travel_time(distance: f64, max_vel: f64, acceleration: f64) -> f64 {
    let burn_time = ship_time_to_velocity(max_vel, acceleration) * 2.0;
    let burn_distance = stationary_distance_covered_while_accelerating(max_vel, acceleration) * 2.0;

    if burn_distance > distance {
        panic!()
    }

    let time_dilation = lorentz_factor(max_vel);
    burn_time + (distance - burn_distance) / max_vel * time_dilation
}

fn handle_camera(
    keyboard: Res<Input<KeyCode>>,
    time: Res<Time>,
    mut proj: Query<&mut bevy::render::camera::OrthographicProjection>,
    mut camera_center: ResMut<CameraCenter>,
) {
    let mut proj = proj.single_mut();

    if keyboard.pressed(KeyCode::T) {
        proj.scale /= (1.0 + (1.0 * time.delta_seconds()));
    }

    if keyboard.pressed(KeyCode::G) {
        proj.scale *= (1.0 + (1.0 * time.delta_seconds()));
    }

    let movement_speed = FixedScalar::from_num(proj.scale * 200.0 * time.delta_seconds());

    if keyboard.pressed(KeyCode::L) {
        camera_center.0.x += movement_speed;
    }

    if keyboard.pressed(KeyCode::J) {
        camera_center.0.x -= movement_speed;
    }

    if keyboard.pressed(KeyCode::I) {
        camera_center.0.y += movement_speed;
    }

    if keyboard.pressed(KeyCode::K) {
        camera_center.0.y -= movement_speed;
    }
}

fn update_body_scales(
    mut query: Query<(&mut Transform, &OrbitalParams)>,
    proj: Query<&bevy::render::camera::OrthographicProjection>,
) {
    let proj = proj.single();

    for (mut transform, params) in query.iter_mut() {
        transform.scale = Vec3::splat((params.radius_m as f32).max(proj.scale * 1.5));
    }
}

fn update_ship_scales(
    mut query: Query<&mut Transform, With<Position>>,
    proj: Query<&bevy::render::camera::OrthographicProjection>,
) {
    let proj = proj.single();

    for mut transform in query.iter_mut() {
        transform.scale = Vec3::splat(proj.scale * 3.0);
    }
}

fn travel_time(distance: f64, accel: f64) -> f64 {
    // Calculus.
    // Accel = 9.8 m/s^2
    // Vel = 9.8t
    // Distance = 9.8 t^2 / 2
    // Distance / 9.8 * 2 = t^2
    // t = sqrt(Distance / 9.8 * 2)
    // Except that we're going halfway then decelerating again.
    // so we have
    // t = sqrt(Distance / 2 / 9.8 * 2) * 2
    (distance / accel).sqrt() * 2.0
}

fn travel_time_no_decel(distance: f64, accel: f64) -> f64 {
    // Calculus.
    // Accel = 9.8 m/s^2
    // Vel = 9.8t
    // Distance = 9.8 t^2 / 2
    // Distance / 9.8 * 2 = t^2
    // t = sqrt(Distance / 9.8 * 2)
    // Except that we're going halfway then decelerating again.
    // so we have
    // t = sqrt(Distance / 2 / 9.8 * 2) * 2
    (distance / accel * 2.0).sqrt()
}

fn update_time(mut time_since_start: ResMut<TimeSinceStartMs>, timestep: Res<Timestep>) {
    time_since_start.0 += timestep.0;
}

fn update_ship_pos(
    mut query: Query<(&mut Transform, &Position)>,
    camera_center: Res<CameraCenter>,
) {
    for (mut transform, position) in query.iter_mut() {
        transform.translation = (position.0 - camera_center.0).as_vec2().extend(0.2);
    }
}

fn update_pos(
    mut query: Query<(&OrbitalParams, Option<&SateliteOf>, &mut Transform)>,
    param_query: Query<&OrbitalParams>,
    time_since_start: Res<TimeSinceStartMs>,
    camera_center: Res<CameraCenter>,
) {
    let sol_mass_kg = 1.9885E30 * 0.0898;

    for (params, satelite_of, mut transform) in query.iter_mut() {
        let pos = calc_pos_with_parent(
            *params,
            satelite_of,
            &param_query,
            time_since_start.as_secs_f64(),
        );
        let pos_z = if satelite_of.is_some() { 0.1 } else { 0.0 };

        transform.translation = (pos - camera_center.0).as_vec2().extend(pos_z);
    }
}

fn calc_pos(params: OrbitalParams, parent_mass_kg: f64, time: f64) -> FixedVec2 {
    if params.semi_major_axis_m == 0.0 {
        return FixedVec2::default();
    }

    let dvec2: DVec2 = orbit_math::calculate_position_at_time(
        params.semi_major_axis_m as f64,
        params.eccentricity as f64,
        params.periapsis_rotation_r as f64,
        params.initial_offset_r as f64,
        parent_mass_kg as f64,
        time,
    )
    .into();

    FixedVec2::new(
        FixedScalar::from_num(dvec2.x),
        FixedScalar::from_num(dvec2.y),
    )
}

fn calc_pos_with_parent(
    params: OrbitalParams,
    satelite_of: Option<&SateliteOf>,
    param_query: &Query<&OrbitalParams>,
    time: f64,
) -> FixedVec2 {
    let sol_mass_kg = 1.9885E30 * 0.0898;

    match satelite_of {
        Some(parent) => {
            let parent_params = param_query.get(parent.0).unwrap();

            let parent_pos = calc_pos(*parent_params, sol_mass_kg, time);

            let pos = calc_pos(params, parent_params.mass_kg, time);

            parent_pos.offset_by_pos(pos)
        }
        None => calc_pos(params, sol_mass_kg, time),
    }
}

fn draw_ui(mut contexts: bevy_egui::EguiContexts, time_since_start: Res<TimeSinceStartMs>) {
    let ctx = contexts.ctx_mut();

    bevy_egui::egui::Window::new("Ships").show(&*ctx, |ui| {
        //for (name, state) in query.iter() {
        ui.label(&format!(
            "Days: {:.2}",
            time_since_start.as_secs_f64() / 60.0 / 60.0 / 24.0
        ));
        //}
    });
}
