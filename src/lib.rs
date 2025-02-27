#![feature(portable_simd)]

pub mod simd_orbits;

use simd_orbits::Vec3;

//mod gptorbits;
//mod orbit_math;
/*
use bevy::math::DVec2;
use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use bevy_egui::EguiPlugin;
use rand::{Rng, SeedableRng};
use std::collections::{BTreeMap, HashMap};
use std::f64::consts::TAU;
use std::ops::{Add, AddAssign, Sub};

fn main() {
    let updates_per_sec = 1000;

    App::new()
        .insert_resource(TimeSinceStartMs(0))
        .insert_resource(Timestep(
            /*(60 * 60 * 24 * 30) * */ 500_000 * (1000 / updates_per_sec),
        ))
        //.insert_resource(FixedTime::new_from_secs(1.0 / updates_per_sec as f32))
        .insert_resource(CameraCenter(Default::default()))
        .add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, update_pos)
        .add_systems(Update, update_ship_pos)
        .add_systems(Update, handle_camera)
        .add_systems(Update, update_body_scales)
        .add_systems(Update, update_ship_scales)
        .add_systems(Update, draw_ui)
        .add_systems(Update, update_time)
        .run();
}

#[derive(Resource)]
struct CameraCenter(UniversalPos);

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
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    satelites: HashMap<String, SerializedBody>,
    periapsis_rotation_r: Option<f64>,
}

#[derive(serde::Serialize, serde::Deserialize, Clone, Debug)]
struct System {
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    bodies: BTreeMap<String, SerializedBody>,
    x: u64,
    y: u64,
}
*/
type UniversalScalar = fixed::types::I108F20;

#[derive(Clone, Copy, Default, Debug)]
pub struct UniversalPos {
    pub x: UniversalScalar,
    pub y: UniversalScalar,
    pub z: UniversalScalar,
}

use az::Cast;

use std::ops::{Add, AddAssign, Div, Mul, Sub};

impl UniversalPos {
    pub fn new_3(x: f64, y: f64, z: f64) -> Self {
        Self::new(
            UniversalScalar::from_num(x),
            UniversalScalar::from_num(y),
            UniversalScalar::from_num(z),
        )
    }

    pub fn new(x: UniversalScalar, y: UniversalScalar, z: UniversalScalar) -> Self {
        Self { x, y, z }
    }

    pub fn distance_squared(self, other: Self) -> f64 {
        (self - other).length_squared()
    }

    pub fn distance(self, other: Self) -> f64 {
        self.distance_squared(other).sqrt()
    }
}

impl Add<Vec3<f64>> for UniversalPos {
    type Output = Self;

    fn add(self, other: Vec3<f64>) -> Self {
        Self {
            x: self.x + UniversalScalar::from_num(other.x),
            y: self.y + UniversalScalar::from_num(other.y),
            z: self.z + UniversalScalar::from_num(other.z),
        }
    }
}

impl AddAssign<Vec3<f64>> for UniversalPos {
    fn add_assign(&mut self, other: Vec3<f64>) {
        *self = *self + other;
    }
}

impl Sub<UniversalPos> for UniversalPos {
    type Output = Vec3<f64>;

    fn sub(self, other: Self) -> Vec3<f64> {
        Vec3 {
            x: (self.x - other.x).cast(),
            y: (self.y - other.y).cast(),
            z: (self.z - other.z).cast(),
        }
    }
}
/*
// In meters.
#[derive(Component)]
struct Radius(pub f64);

#[derive(Component)]
struct Position(UniversalPos);

#[derive(Component)]
struct Velocity(DVec2);

#[derive(Component)]
struct SateliteOf(Entity);

#[derive(Component, Clone, Copy)]
struct OrbitalParams {
    semi_major_axis_m: f32,
    eccentricity: f32,
    mass_kg: f32,
    periapsis_rotation_r: f32,
    initial_offset_r: f32,
}

struct Context<'a> {
    commands: Commands<'a, 'a>,
    meshes: &'a mut Assets<Mesh>,
    materials: &'a mut Assets<ColorMaterial>,
    rng: rand::rngs::SmallRng,
    positions_and_mus: Vec<(OrbitalParams, Option<OrbitalParams>, f64)>,
}

fn spawn_body(
    name: &str,
    body: &SerializedBody,
    satelite_of: Option<(Entity, OrbitalParams)>,
    context: &mut Context,
) {
    let circle_mesh = context.meshes.add(Circle::new(1.0));

    let params = OrbitalParams {
        semi_major_axis_m: (body.semi_major_axis_km * 1000.0) as f32,
        eccentricity: body.eccentricity as _,
        periapsis_rotation_r: body
            .periapsis_rotation_r
            .unwrap_or_else(|| context.rng.gen_range(0.0..TAU)) as _,
        initial_offset_r: context.rng.gen_range(0.0..TAU) as _,
        mass_kg: body.mass_kg as _,
    };

    context.positions_and_mus.push((
        params,
        satelite_of.as_ref().map(|&(_, b)| b),
        params.mass_kg as f64 * (G as f64),
    ));

    let mut entity_commands = context.commands.spawn((
        Mesh2d(circle_mesh.clone()),
        MeshMaterial2d(context
            .materials
            .add(bevy::prelude::ColorMaterial::from(Color::from(if satelite_of.is_none() {
                bevy::color::palettes::basic::RED
            } else {
                bevy::color::palettes::basic::PURPLE
            })))),
        params,
        Radius(body.radius_km * 1000.0),
        Name::new(name.to_string()),
    ));

    if let Some((parent, _)) = satelite_of {
        entity_commands.insert(SateliteOf(parent));
    }

    let entity = entity_commands.id();

    for (name, body) in body.satelites.iter() {
        spawn_body(name, body, Some((entity, params)), context);
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let rng = rand::rngs::SmallRng::from_seed([0; 32]);

    let mut camera = Camera2dBundle::default();
    camera.projection.scale = 200_000_000.0;
    //camera.camera_2d.clear_color =
    //    bevy::core_pipeline::clear_color::ClearColorConfig::Custom(Color::BLACK);
    commands.spawn(camera);

    let x: BTreeMap<String, System> =
        toml::from_str(&std::fs::read_to_string("planets.toml").unwrap())
            .unwrap_or_else(|err| panic!("{}", err));

    let mut context = Context {
        commands,
        meshes: &mut meshes,
        materials: &mut materials,
        rng,
        positions_and_mus: Default::default(),
    };

    for (_name, system) in x {
        for (name, body) in system.bodies.iter() {
            spawn_body(name, body, None, &mut context);
        }
    }

    let mut pos = UniversalPos::new_2(149_598_016.0 * 1000.0, 0.0);
    let mut velocity = DVec2::new(0.0, 100.0);
    for d in 0..10_u32 {
        for t in 0..60 * 60 * 24 {
            let t = (60 * 60 * 24 * d) + t;
            let acceleration = get_acceleration(&context.positions_and_mus, pos.as_dvec2(), t as _);
            velocity += acceleration;
            pos += velocity;
        }
        dbg!(d);
    }

    dbg!(pos, velocity);
    //panic!();
}

const C: f64 = 299_792_458.0;
const G: f64 = 9.80665;
const LY: f64 = 9.4607E15;

fn fix_time(t: f64) -> f64 {
    t / 60.0 / 60.0 / 24.0 / 365.25
}

fn fix_distance(d: f64) -> f64 {
    d / 9.4607E15
}

fn handle_camera(
    keyboard: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut proj: Query<&mut bevy::render::camera::OrthographicProjection>,
    mut camera_center: ResMut<CameraCenter>,
) {
    let mut proj = proj.single_mut();

    if keyboard.pressed(KeyCode::KeyT) {
        proj.scale /= 1.0 + (1.0 * time.delta_secs());
    }

    if keyboard.pressed(KeyCode::KeyG) {
        proj.scale *= 1.0 + (1.0 * time.delta_secs());
    }

    let movement_speed = UniversalScalar::from_num(proj.scale * 200.0 * time.delta_secs());

    if keyboard.pressed(KeyCode::KeyL) {
        camera_center.0.x += movement_speed;
    }

    if keyboard.pressed(KeyCode::KeyJ) {
        camera_center.0.x -= movement_speed;
    }

    if keyboard.pressed(KeyCode::KeyI) {
        camera_center.0.y += movement_speed;
    }

    if keyboard.pressed(KeyCode::KeyK) {
        camera_center.0.y -= movement_speed;
    }
}

fn update_body_scales(
    mut query: Query<(&mut Transform, &Radius)>,
    proj: Query<&bevy::render::camera::OrthographicProjection>,
) {
    let proj = proj.single();

    for (mut transform, radius) in query.iter_mut() {
        transform.scale = Vec3::splat((radius.0 as f32).max(proj.scale * 1.5));
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

fn update_time(time_since_start: ResMut<TimeSinceStartMs>, timestep: Res<Timestep>) {
    //time_since_start.0 += timestep.0;
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
    for (params, satelite_of, mut transform) in query.iter_mut() {
        let pos = calc_pos_with_parent(
            *params,
            satelite_of.map(|parent| *param_query.get(parent.0).unwrap()),
            time_since_start.as_secs(),
        );
        let pos_z = if satelite_of.is_some() { 0.1 } else { 0.0 };

        transform.translation = (pos - camera_center.0.as_dvec2()).as_vec2().extend(pos_z);
    }
}

#[inline(never)]
fn calc_pos(params: OrbitalParams, parent_mass_kg: f32, time: f32) -> DVec2 {
    if params.semi_major_axis_m == 0.0 {
        return Default::default();
    }

    DVec2::from(gptorbits::orbital_position(
        params.semi_major_axis_m as _,
        params.eccentricity as _,
        params.periapsis_rotation_r as _,
        params.initial_offset_r as _,
        parent_mass_kg as _,
        time as _,
        5e-10 as _,
    ))
    //.as_dvec2()
}

#[inline(never)]
fn calc_pos_with_parent(
    params: OrbitalParams,
    satelite_of: Option<OrbitalParams>,
    time: f32,
) -> DVec2 {
    let sol_mass_kg = 1.9885E30 * 0.0898;

    match satelite_of {
        Some(parent_params) => {
            let parent_pos = calc_pos(parent_params, sol_mass_kg, time);

            let pos = calc_pos(params, parent_params.mass_kg, time);

            parent_pos + pos
        }
        None => calc_pos(params, sol_mass_kg, time),
    }
}

pub fn get_acceleration(
    positions_and_mus: &[(OrbitalParams, Option<OrbitalParams>, f64)],
    position: DVec2,
    time: f32,
) -> DVec2 {
    positions_and_mus
        .iter()
        .map(|&(orbit, parent_orbit, mu)| {
            let body_position = calc_pos_with_parent(orbit, parent_orbit, time);
            let dir = body_position - position;
            let mag_2 = dir.length_squared();
            let force = dir / (mag_2 * mag_2.sqrt());
            force * mu
        })
        .sum()
}

fn draw_ui(mut contexts: bevy_egui::EguiContexts, time_since_start: Res<TimeSinceStartMs>) {
    let ctx = contexts.ctx_mut();

    bevy_egui::egui::Window::new("Ships").show(&*ctx, |ui| {
        ui.label(format!(
            "Days: {:.2}",
            time_since_start.as_secs_f64() / 60.0 / 60.0 / 24.0
        ));
    });
}

// in km^3 s^-2 or some shit.
fn calculate_mu(mass_in_kg: f64) -> f64 {
    mass_in_kg * G / 1_000_000_000.0
}
 */
