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
        .add_system(move_ships.in_schedule(CoreSchedule::FixedUpdate))
        .add_system(move_ships_simple.in_schedule(CoreSchedule::FixedUpdate))
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

#[derive(Component)]
struct ShipCommands(VecDeque<Command>);

#[derive(Debug, Clone, Copy)]
enum ShippableResource {
    Ice,
}

// in kg
#[derive(Component, Default)]
struct ShippableResources {
    ice: u64,
}

impl ShippableResources {
    fn sum(&self) -> u64 {
        self.ice
    }

    fn get(&self, id: ShippableResource) -> u64 {
        match id {
            ShippableResource::Ice => self.ice,
        }
    }

    fn get_mut(&mut self, id: ShippableResource) -> &mut u64 {
        match id {
            ShippableResource::Ice => &mut self.ice,
        }
    }
}

#[derive(Clone)]
enum Command {
    DeliverResourceTo {
        to: Entity,
        resource: ShippableResource,
    },
    MoveToObjectNoDecel(Entity),
    MoveToNoDecel(FixedVec2),
    MoveToBody(Entity),
    MoveTo(FixedVec2),
    RepeatCommands(Vec<Command>),
    WaitMillis {
        millis: u64,
        at: Entity,
    },
    WaitUntil {
        time: u64,
        at: Entity,
        offset: DVec2,
    },
    WaitIndefinitely {
        at: Entity,
        offset: DVec2,
    },
    LoadResource {
        at: Entity,
        resource: ShippableResource,
    },
    UnloadResource {
        at: Entity,
        resource: ShippableResource,
    },
}

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

fn spawn_belts(
    belts: &[AsteroidBelt],
    satelites_of: Option<Entity>,
    context: &mut Context
) {
    for belt in belts {
     let periapsis_rotation_r = context.rng.gen_range(0.0..TAU);

        let semi_major_axis_dist = rand_distr::Normal::new(belt.semi_major_axis_mean_km, belt.semi_major_axis_stddev).unwrap();
        let eccentricity_dist = rand_distr::Normal::new(belt.eccentricity_mean, belt.eccentricity_stddev).unwrap();

        for i in 0 .. belt.num.get() {
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
                asteroid_belts: Vec::new()
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
            material: context.materials.add(ColorMaterial::from(if satelite_of.is_none() {
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
            periapsis_rotation_r: body.periapsis_rotation_r.unwrap_or_else(|| context.rng.gen_range(0.0..TAU)),
            initial_offset_r: context.rng.gen_range(0.0..TAU),
            mass_kg: body.mass_kg,
            radius_m: body.radius_km * 1000.0,
        },
        Name::new(name.clone()),
        ShippableResources { ice: body.ice_kg },
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
        toml::from_str(&std::fs::read_to_string("planets.toml").unwrap()).unwrap_or_else(|err| {
            panic!("{}", err)
        });

    /*let mut systems: BTreeMap<String, System> = BTreeMap::new();

    systems.insert("Sol".to_string(), System {
        bodies: x.clone(),
        x: 112,
        y: 556
    });*/

    //std::fs::write("out.toml", &toml::to_string_pretty(&systems).unwrap()).unwrap();

    let circle_mesh = meshes.add(shape::Circle::new(1.0).into());

    let mut bodies_map: HashMap<String, Entity> = HashMap::new();
    let mut bodies_vec: Vec<(Entity, u64)> = Vec::new();

    /*let mut orbital_params =
        |body: &SerializedBody, rng: &mut rand::rngs::SmallRng| OrbitalParams {
            semi_major_axis_m: body.semi_major_axis_km * 1000.0,
            eccentricity: body.eccentricity,
            periapsis_rotation_r: rng.gen_range(0.0..TAU),
            initial_offset_r: rng.gen_range(0.0..TAU),
            mass_kg: body.mass_kg,
            radius_m: body.radius_km * 1000.0,
        };

    let mut circle_from_colour = |colour| MaterialMesh2dBundle {
        mesh: circle_mesh.clone().into(),
        material: materials.add(ColorMaterial::from(colour)),
        transform: Transform::from_scale(Vec3::splat(1.0)),
        ..default()
    };*/

    /*let mut spawn_body = |name: &String, body, colour, satelite_of, bodies_map: &mut HashMap<String, Entity>| {
        let mut entity_commands = commands.spawn((
            circle_from_colour(colour),
            orbital_params(body, &mut rng),
            Name::new(name.clone()),
            ShippableResources {
                ice: body.ice_kg
            }
        ));

        if body.population > 0 {
            entity_commands.insert(Populated(body.population));
        }

        if let Some(parent) = satelite_of {
            entity_commands.insert(SateliteOf(parent));
        }

        let entity = entity_commands.id();

        bodies_map.insert(name.clone(), entity);
        bodies_vec.push((entity, body.population));

        for (name, body) in body.satelites.iter() {
            spawn_body(name, body, Color::ORANGE, Some(entity), bodies_map)
        }
    };*/

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

    let mut pick_2 = |rng: &mut rand::rngs::SmallRng| {
        let sample = rand::seq::index::sample_weighted(
            rng,
            context.bodies_vec.len(),
            |index| (context.bodies_vec[index].1 as f64).cbrt(),
            2,
        )
        .unwrap();

        [
            context.bodies_vec[sample.index(0)].0,
            context.bodies_vec[sample.index(1)].0,
        ]
    };

    let a = context.commands.spawn((
        MaterialMesh2dBundle {
            mesh: circle_mesh.clone().into(),
            material: context.materials.add(ColorMaterial::from(Color::GREEN)),
            transform: Transform::from_scale(Vec3::splat(1.0)),
            ..default()
        },
        Position(FixedVec2::new_2(-10_000_000_000.0, 0.0)),
        Velocity(DVec2::new(-1500_000.0, -500_000.0)),
        //ShipCommands(VecDeque::from([])),
        Name::new("OPA Freighter"),
        StandardAcceleration(9.8 / 3.0),
    )).id();

    let b = context.commands.spawn((
        MaterialMesh2dBundle {
            mesh: circle_mesh.clone().into(),
            material: context.materials.add(ColorMaterial::from(Color::GREEN)),
            transform: Transform::from_scale(Vec3::splat(1.0)),
            ..default()
        },Position(FixedVec2::new_2(-10_000_000_000.0, -10_000_000_000.0)),
        Velocity(DVec2::new(0.0, -1000_000.0)),
        //ShipCommands(VecDeque::from([])),
        Name::new("OPA Freighter"),
        StandardAcceleration(9.8 / 3.0),
    )).id();
    
    let _ = context.commands.spawn((
        MaterialMesh2dBundle {
            mesh: circle_mesh.clone().into(),
            material: context.materials.add(ColorMaterial::from(Color::PURPLE)),
            transform: Transform::from_scale(Vec3::splat(1.0)),
            ..default()
        },
        Position(FixedVec2::default()),
        Velocity(DVec2::new(300_000.0, 1000_000.0)),
        ShipCommands(VecDeque::from([Command::RepeatCommands(vec![
            Command::MoveToObjectNoDecel(a),
            Command::MoveToObjectNoDecel(b),
            Command::MoveToNoDecel(FixedVec2::new_2(0.0, -10_000_000_000.0)),
            Command::MoveToNoDecel(FixedVec2::new_2(0.0, 0.0)),
        ])])),
        ShippableResources::default(),
        Name::new("OPA Freighter"),
        StandardAcceleration(9.8 / 3.0),
    ));


    let _ = context.commands.spawn((
        MaterialMesh2dBundle {
            mesh: circle_mesh.clone().into(),
            material: context.materials.add(ColorMaterial::from(Color::GREEN)),
            transform: Transform::from_scale(Vec3::splat(1.0)),
            ..default()
        },Position(FixedVec2::new_2(0.0, -10_000_000_000.0)),
        Velocity(DVec2::ZERO),
        ShipCommands(VecDeque::from([])),
        Name::new("OPA Freighter"),
        StandardAcceleration(9.8 / 3.0),
    ));

    let _ = context.commands.spawn((
        MaterialMesh2dBundle {
            mesh: circle_mesh.clone().into(),
            material: context.materials.add(ColorMaterial::from(Color::GREEN)),
            transform: Transform::from_scale(Vec3::splat(1.0)),
            ..default()
        },Position(FixedVec2::new_2(0.0, 0.0)),
        Velocity(DVec2::ZERO),
        ShipCommands(VecDeque::from([])),
        Name::new("OPA Freighter"),
        StandardAcceleration(9.8 / 3.0),
    ));

    /*for i in 0..50 {
        let [first, second] = pick_2(&mut rng);

        let spawn_distance = 1_000_000_000_000.0;

        let _ = commands.spawn((
            circle_from_colour(Color::PURPLE),
            Position(FixedVec2::new(
                FixedScalar::from_num(rng.gen_range(-spawn_distance..spawn_distance)),
                FixedScalar::from_num(rng.gen_range(-spawn_distance..spawn_distance)),
            )),
            Velocity(DVec2::ZERO),
            ShipCommands(VecDeque::from([Command::RepeatCommands(vec![
                Command::DeliverResourceTo {
                    to: bodies_map["Ceres"],
                    resource: ShippableResource::Ice,
                },
                //Command::LoadResource { at: bodies_map["Saturn"], resource: ShippableResource::Ice},
                //Command::MoveToBody(bodies_map["Ceres"]),
                //Command::UnloadResource { at: bodies_map["Ceres"], resource: ShippableResource::Ice},

                /*Command::WaitMillis {
                    millis: 1000 * 60 * 60 * 24,
                    at: first,
                },*/
            ])])),
            Name::new("OPA Freighter"),
            StandardAcceleration(9.8 / 3.0),
            ShippableResources::default(),
        ));
    }*/

    /*for i in 0..10 {
        let [first, second] = pick_2(&mut rng);

        let _ = commands.spawn((
            circle_from_colour(Color::GREEN),
            Position(DVec2::new(
                rng.gen_range(-100_000_000_000.0..100_000_000_000.0),
                rng.gen_range(-100_000_000_000.0..100_000_000_000.0),
            )),
            Velocity(DVec2::ZERO),
            ShipCommands(VecDeque::from([Command::RepeatCommands(vec![
                Command::MoveToBody(first),
                Command::WaitMillis {
                    millis: 1000 * 60 * 60 * 24,
                    at: first,
                },
                Command::MoveToBody(second),
                Command::WaitMillis {
                    millis: 1000 * 60 * 60 * 24,
                    at: second,
                },
            ])])),
            Name::new("MCRN Patrol"),
            StandardAcceleration(9.8),
            ShippableResources::default()
        ));
    }*/

    /*let _ = commands.spawn((
        circle_from_colour(Color::GREEN),
        Position(DVec2::new(100_000_000_000.0, -10_000_000_000.0)),
        Velocity(DVec2::ZERO),
        ShipCommands(ship_commands.clone()),
        Name::new("MCRN Patrol"),
        StandardAcceleration(9.8)
    ));

    let _ = commands.spawn((
        circle_from_colour(Color::GREEN),
        Position(DVec2::new(100_000_000_000.0, -10_000_000_000.0)),
        Velocity(DVec2::ZERO),
        ShipCommands(VecDeque::from([Command::RepeatCommands(vec![
            Command::MoveToBody(bodies_map["Mars"]),
            Command::WaitMillis {
                millis: 1000 * 60 * 60 * 24,
                at: bodies_map["Mars"],
            },
            Command::MoveToBody(bodies_map["Ceres"]),
            Command::WaitMillis {
                millis: 1000 * 60 * 60 * 24,
                at: bodies_map["Ceres"],
            },
        ])])),
        Name::new("MCRN Patrol"),
        StandardAcceleration(9.8)
    ));

    let _ = commands.spawn((
        circle_from_colour(Color::GREEN),
        Position(DVec2::new(100_000_000_000.0, -10_000_000_000.0)),
        Velocity(DVec2::ZERO),
        ShipCommands(VecDeque::from([Command::RepeatCommands(vec![
            Command::MoveToBody(bodies_map["Mercury"]),
            Command::MoveToBody(bodies_map["Venus"]),
            Command::MoveToBody(bodies_map["Earth"]),
            Command::MoveToBody(bodies_map["Mars"]),
            Command::MoveToBody(bodies_map["Jupiter"]),
            Command::MoveToBody(bodies_map["Saturn"]),
        ])])),
        Name::new("Racer"),
        StandardAcceleration(9.8 * 2.0)
    ));

    let _ = commands.spawn((
        circle_from_colour(Color::PURPLE),
        Position(DVec2::new(100_000_000_000.0, -10_000_000_000.0)),
        Velocity(DVec2::ZERO),
        ShipCommands(VecDeque::from([Command::RepeatCommands(vec![
            Command::MoveToBody(bodies_map["Tycho Station"]),
            Command::WaitMillis {
                millis: 1000 * 60 * 60 * 24,
                at: bodies_map["Tycho Station"],
            },
            Command::MoveToBody(bodies_map["Ceres"]),
            Command::WaitMillis {
                millis: 1000 * 60 * 60 * 24,
                at: bodies_map["Ceres"],
            },
        ])])),
        Name::new("OPA Vessel"),
        StandardAcceleration(9.8 / 3.0)
    ));*/

    //let mut commands = context.commands;

    context
        .commands
        .insert_resource(ParentBodies(context.bodies_map));
    context.commands.insert_resource(GameRng(context.rng));
}

fn aim_ship_at_body(
    ship_position: FixedVec2,
    max_accel: f64,
    params: OrbitalParams,
    satelite_of: Option<&SateliteOf>,
    param_query: &Query<&OrbitalParams>,
    time: f64,
) -> (FixedVec2, f64) {
    let mut body_pos = calc_pos_with_parent(params, satelite_of, param_query, time);
    let mut travel_time_guess = travel_time(body_pos.distance(ship_position), max_accel);

    // Perform an integration where we're trying to aim correctly at the planet.
    // First aim at the planet's current position and calculate the travel time.
    // Then project where the planet will be at that point in the future, and if
    // we'll still hit the planet then stop. Otherwise keep iterating.
    for _ in 0..10 {
        let projected_body_pos =
            calc_pos_with_parent(params, satelite_of, param_query, time + travel_time_guess);
        travel_time_guess = travel_time(projected_body_pos.distance(ship_position), max_accel);

        let distance_from_center = body_pos.distance(projected_body_pos);
        //dbg!(travel_time_guess, distance_from_center);

        body_pos = projected_body_pos;

        //if params.radius_m > distance_from_center {
        //break;
        //}
    }

    (body_pos, travel_time_guess)
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

/*
fn main() {
    dbg!(fix_time(stationary_time_to_velocity(0.99 * C, G)));
    dbg!(fix_time(ship_time_to_velocity(0.9999999999999 * C, G)));


    dbg!(stationary_travel_time(LY * 100.0, 0.999 * C, G) / 60.0 / 60.0 / 24.0 / 365.25);
    dbg!(ship_travel_time(LY * 100.0, 0.99975 * C, G) / 60.0 / 60.0 / 24.0 / 365.25);
}
*/
/*

fn command_ships(
    mut ships: Query<(&mut ShipCommands, &mut Velocity)>,
    buttons: Res<Input<MouseButton>>,
    windows_query: Query<&Window>,
    camera_query: Query<(&Camera, &GlobalTransform)>,
) {
    if !buttons.just_pressed(MouseButton::Left) {
        return
    }

    let window = windows_query.single();

    let mouse_position = match window.cursor_position() {
        Some(position) => position,
        None => return
    };

    let (camera, camera_transform) = camera_query.single();

    let world_pos = match camera.viewport_to_world_2d(camera_transform, mouse_position) {
        Some(pos) => pos,
        None => return
    };

    let (mut commands, mut velocity) = ships.single_mut();
    commands.0.push_back(Command::MoveTo(world_pos.as_dvec2()));
    velocity.0 = DVec2::ZERO;
}
*/

fn extend_front<T: Clone>(queue: &mut VecDeque<T>, items: &[T]) {
    queue.reserve(items.len());
    for item in items.iter().rev() {
        queue.push_front(item.clone());
    }
}

fn move_ships(
    mut ships: Query<
        (
            &mut Position,
            &mut Velocity,
            &mut ShipCommands,
            &mut ShippableResources,
            &StandardAcceleration,
        ),
        Without<OrbitalParams>,
    >,
    orbits: Query<(&OrbitalParams, Option<&SateliteOf>)>,
    target_objects: Query<(&Position, &Velocity), Without<ShipCommands>>,
    param_query: Query<&OrbitalParams>,
    mut resources_query: Query<(Entity, &mut ShippableResources), With<OrbitalParams>>,
    mut rng: ResMut<GameRng>,

    timestep: Res<Timestep>,
    time_since_start: Res<TimeSinceStartMs>,
) {
    let calc_pos = |station: Entity| {
        let (params, satelite_of) = orbits.get(station).unwrap();
        calc_pos_with_parent(
            *params,
            satelite_of,
            &param_query,
            time_since_start.as_secs_f64(),
        )
    };

    for (mut position, mut velocity, mut commands, mut resources, standard_accel) in
        ships.iter_mut()
    {
        let max_accel = standard_accel.0;

        let mut finished = false;

        while !finished {
            match commands.0.front().cloned() {
                Some(Command::DeliverResourceTo { resource, to }) => {
                    let place_with_most = resources_query
                        .iter()
                        .filter(|&(entity, resources)| entity != to && resources.get(resource) > 0)
                        .max_by_key(|(_, resources)| {
                            resources.get(resource) * rng.0.gen_range(950..1050)
                        })
                        .map(|(entity, _)| entity);

                    match place_with_most {
                        Some(from) => extend_front(
                            &mut commands.0,
                            &[
                                Command::MoveToBody(from),
                                Command::LoadResource { at: from, resource },
                                Command::MoveToBody(to),
                                Command::UnloadResource { at: to, resource },
                            ],
                        ),
                        None => {
                            log::error!("No bodies with ice");
                            commands.0.clear();
                            commands.0.push_front(Command::WaitIndefinitely {
                                at: to,
                                offset: position.0 - calc_pos(to),
                            });
                        }
                    }
                }

                Some(Command::MoveToObjectNoDecel(object)) => {
                    let (target_pos, target_vel) = target_objects.get(object).unwrap();

                    let mut intercept_pos = target_pos.0;
                    let mut travel_time = travel_time_no_decel(intercept_pos.distance(position.0), max_accel);

                    for _ in 0 .. 100 {
                        let relative_velocity = target_vel.0 - velocity.0;
                        intercept_pos = target_pos.0 + relative_velocity * travel_time;
                        travel_time = travel_time_no_decel(intercept_pos.distance(position.0), max_accel);
                        //dbg!(travel_time);
                    }

                    let scalar_velocity = velocity.0.length();

                    let burn_vector = (intercept_pos - position.0).normalize();

                    let velocity_increase = burn_vector * max_accel * timestep.as_secs_f64();

                    let velocity_over_timestep =
                        (velocity.0 + velocity_increase / 2.0) * timestep.as_secs_f64();

                    velocity.0 += velocity_increase;

                    if position.0.distance_squared(target_pos.0) < velocity_over_timestep.length_squared()
                    {
                        dbg!(velocity.0);
                        position.0 = target_pos.0;
                        commands.0.pop_front();
                    } else {
                        position.0 += velocity_over_timestep;
                    }

                    finished = true;
                }

                Some(Command::MoveToNoDecel(target)) => {
                    let mut target_pos = target;
                    let mut travel_time = travel_time_no_decel(target_pos.distance(position.0), max_accel);

                    for _ in 0 .. 100 {
                        let relaitve_velocity = -velocity.0;
                        target_pos = target + relaitve_velocity * travel_time;
                        travel_time = travel_time_no_decel(target_pos.distance(position.0), max_accel);
                        //dbg!(travel_time);
                    }

                    let scalar_velocity = velocity.0.length();

                    let burn_vector = (target_pos - position.0).normalize();

                    let velocity_increase = burn_vector * max_accel * timestep.as_secs_f64();

                    let velocity_over_timestep =
                        (velocity.0 + velocity_increase / 2.0) * timestep.as_secs_f64();

                    velocity.0 += velocity_increase;

                    if position.0.distance_squared(target) < velocity_over_timestep.length_squared()
                    {
                        position.0 = target;
                        commands.0.pop_front();
                    } else {
                        position.0 += velocity_over_timestep;
                    }

                    finished = true;
                }
                Some(Command::MoveTo(target)) => {
                    let stopping_distance = velocity.0.length_squared() / 2.0 / max_accel;
                    let stopping_distance_sq = stopping_distance * stopping_distance;

                    let burn_vector = if position.0.distance_squared(target) > stopping_distance_sq
                    {
                        (target - position.0).normalize()
                    } else {
                        -(target - position.0).normalize()
                    };

                    let velocity_increase = burn_vector * max_accel * timestep.as_secs_f64();

                    let velocity_over_timestep =
                        (velocity.0 + velocity_increase / 2.0) * timestep.as_secs_f64();

                    velocity.0 += velocity_increase;

                    if position.0.distance_squared(target) < velocity_over_timestep.length_squared()
                    {
                        //dbg!(velocity.0.length());
                        position.0 = target;
                        commands.0.pop_front();
                        velocity.0 = DVec2::ZERO;
                    } else {
                        position.0 += velocity_over_timestep;
                    }

                    finished = true;
                }
                Some(Command::MoveToBody(body)) => {
                    let (params, satelite_of) = orbits.get(body).unwrap();
                    let (pos, travel_time) = aim_ship_at_body(
                        position.0,
                        max_accel,
                        *params,
                        satelite_of,
                        &param_query,
                        time_since_start.as_secs_f64(),
                    );
                    commands.0.pop_front();
                    commands.0.push_front(Command::MoveTo(pos));
                }
                Some(Command::RepeatCommands(to_repeat)) => {
                    extend_front(&mut commands.0, &to_repeat);
                }
                Some(Command::LoadResource { at, resource }) => {
                    commands.0.pop_front();

                    let (_, mut target_resources) = resources_query.get_mut(at).unwrap();
                    let target_resource = target_resources.get_mut(resource);

                    let max = 100_000;
                    let current = resources.sum();
                    //dbg!(current, max, *target_resource);
                    if current >= max {
                        log::warn!("Ship already at max when trying to load {:?}.", resource);
                        commands.0.clear();
                        commands.0.push_front(Command::WaitIndefinitely {
                            at,
                            offset: position.0 - calc_pos(at),
                        });
                    } else if *target_resource == 0 {
                        log::warn!("Target out of {:?}", resource);
                        while !matches!(commands.0.front(), Some(&Command::RepeatCommands(_))) {
                            commands.0.pop_front();
                        } /*
                          commands.0.clear();
                          commands.0.push_front(Command::WaitIndefinitely {dbg
                              at,
                              offset: position.0 - calc_pos(at)
                          });*/
                    } else {
                        let can_load = (max - current).min(*target_resource);

                        *target_resource -= can_load;

                        *resources.get_mut(resource) += can_load;

                        commands.0.push_front(Command::WaitMillis {
                            millis: 60 * 60 * can_load,
                            at,
                        })
                    }
                }
                Some(Command::UnloadResource { at, resource }) => {
                    commands.0.pop_front();

                    let (_, mut target_resources) = resources_query.get_mut(at).unwrap();
                    let target_resource = target_resources.get_mut(resource);

                    let ship_resource = resources.get_mut(resource);

                    if *ship_resource == 0 {
                        log::warn!("Ship out of {:?} when trying to unload", resource);
                        commands.0.clear();
                        commands.0.push_front(Command::WaitIndefinitely {
                            at,
                            offset: position.0 - calc_pos(at),
                        });
                    } else {
                        let can_unload = *ship_resource;

                        *target_resource += can_unload;

                        *ship_resource = 0;

                        commands.0.push_front(Command::WaitMillis {
                            // Half the time of loading.
                            millis: 60 * 30 * can_unload,
                            at,
                        })
                    }
                }
                Some(Command::WaitMillis { millis, at }) => {
                    commands.0.pop_front();

                    commands.0.push_front(Command::WaitUntil {
                        time: time_since_start.0 + millis,
                        at,
                        offset: position.0 - calc_pos(at),
                    });
                }
                Some(Command::WaitIndefinitely { at, offset }) => {
                    position.0 = calc_pos(at) + offset;
                    finished = true;
                }
                Some(Command::WaitUntil { time, at, offset }) => {
                    position.0 = calc_pos(at) + offset;

                    if time_since_start.0 >= time {
                        commands.0.pop_front();
                    } else {
                        finished = true;
                    }
                }
                None => {
                    finished = true;
                }
            }
        }
    }
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
        transform.scale = Vec3::splat((params.radius_m as f32).max(proj.scale * 0.5));
    }
}

fn update_ship_scales(
    mut query: Query<&mut Transform, With<Position>>,
    proj: Query<&bevy::render::camera::OrthographicProjection>,
) {
    let proj = proj.single();

    for mut transform in query.iter_mut() {
        transform.scale = Vec3::splat(proj.scale * 1.5);
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
        return FixedVec2::default()
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
        ui.label(&format!("Days: {}", time_since_start.as_secs_f64() / 60.0 / 60.0 / 24.0));
        //}
    });
}

fn aim_from_moving(pos: DVec2, vel: DVec2, target: DVec2, max_accel: f64) {
    let vector = target - pos;
    let distance = vector.length();
    let vector_scaled = vector / distance;

    let travel_time = travel_time(distance, max_accel);
    let max_vel = travel_time / 2.0 * max_accel;

    let scalar_vel = vel.length();

    if scalar_vel / 2.0 > max_vel {
        todo!()
    }

    let target_vel_vector = vector_scaled * (max_vel - scalar_vel);

    dbg!(target_vel_vector - vel);
}

#[test]
fn vector_adjustment() {
    let vector = Vec2::new(-500.0, 0.0);
    let target = Vec2::new(-1.0, 1.0).normalize() * 1000.0;

    aim_from_moving(
        DVec2::ZERO,
        DVec2::new(-500.0, 0.0),
        DVec2::new(1_000_000.0, 1_000_000.0),
        9.8,
    );

    //dbg!(target - vector);
    panic!();
}

fn move_ships_simple(
    mut query: Query<(&mut Position, &Velocity), Without<ShipCommands>>,
    timestep: Res<Timestep>,
) {
    query.for_each_mut(|(mut position, velocity)| {
        position.0 += velocity.0 * timestep.as_secs_f64();
    })
}
