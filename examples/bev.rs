#![feature(portable_simd)]

use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::math::DVec3;
use bevy::prelude::*;
use bevy_polyline::prelude::*;
use nbody_simd::{OrbitParams, UniversalPos};

#[derive(Component)]
struct MainCamera;

#[derive(Resource)]
struct UniversalCamera {
    center: UniversalPos,
    position: UniversalPos,
    distance: f64,
}

impl UniversalCamera {
    fn view_dir(&self) -> DVec3 {
        DVec3::new(1.0, 1.0, 1.0)
    }

    fn compute_position(&mut self) {
        let vector = self.view_dir() * self.distance;
        self.position = self.center + nbody_simd::Vec3::new(vector.x, vector.y, vector.z);
    }
}

use std::simd::Simd;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PolylinePlugin)
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                get_state,
                set_positions.after(get_state),
                update_camera,
                //handle_mouse_scroll,
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

/*fn set_camera_pos(
    mut camera: ResMut<UniversalCamera>,
    mut orbit_cam: Query<&mut PanOrbitCamera>,
) {
    let mut c = orbit_cam.single_mut();
    let t = c.focus.as_dvec3();
    camera.center += nbody_simd::Vec3::new(t.x, t.y, t.z);
    let focus = c.focus;
    dbg!(c.target_focus, focus);
    c.target_focus = Default::default();
    c.focus = Default::default();
    c.force_update = true;
}*/

fn get_state(mut system: ResMut<System>, time: Res<Time>) {
    system.state = system
        .system
        .state_at(time.elapsed_secs_f64() * 60.0 * 60.0 * 24.0 * 1.0);
}

fn set_positions(system: Res<System>, mut query: Query<(&SystemBody, &mut Transform)>) {
    let positions = system.state.planet_and_moon_positions;

    let get_pos = |i| {
        (bevy::math::DVec3::new(positions.x[i], positions.y[i], positions.z[i]) / (190717.305466))
            .as_vec3()
            .xzy()
    };

    for (body, mut transform) in query.iter_mut() {
        *transform = transform.with_translation(get_pos(body.0));
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
        distance: 400_000.0,
        position: Default::default(),
    });

    commands.insert_resource(System {
        system,
        state: system.state_at(0.0),
    });

    let mut sphere_mesh = {
        let mut sphere_mesh = Sphere::new(5000.0).mesh().build();
        sphere_mesh
            .generate_tangents()
            .expect("Failed to generate tangents");
        meshes.add(sphere_mesh)
    };

    let get_pos = |positions: nbody_simd::Vec3<Simd<f64, 64>>, i| {
        (bevy::math::DVec3::new(positions.x[i], positions.y[i], positions.z[i]) / (190717.305466))
            .as_vec3()
            .xzy()
    };

    for i in 0..8 {
        let positions = nbody_simd::orbital_position_from_mean_anomaly(
            OrbitParams::from_array([system.planets.get(i); 64]),
            Simd::from_array(std::array::from_fn(|i| {
                i as f64 / 63.0 * std::f64::consts::TAU
            })),
            sleef::f64x::sincos_u35,
        );

        commands.spawn((PolylineBundle {
            polyline: PolylineHandle(polylines.add(Polyline {
                vertices: std::array::from_fn::<_, 64, _>(|i| get_pos(positions, i)).to_vec(),
            })),
            material: PolylineMaterialHandle(polyline_materials.add(PolylineMaterial {
                width: 1.0,
                color: Color::hsl(55.0, 1.0, 1.5).to_linear(),
                perspective: false,
                ..Default::default()
            })),
            ..Default::default()
        },));

        let mut c = commands.spawn((
            Mesh3d(sphere_mesh.clone()),
            MeshMaterial3d(materials.add(StandardMaterial { ..default() })),
            Transform::IDENTITY,
            SystemBody(i),
        ));

        for j in 0..8 {
            if system.moon_parent_swizzles[j] != i {
                continue;
            }

            let moon_pos = nbody_simd::orbital_position_from_mean_anomaly(
                OrbitParams::from_array([system.moons.get(j); 64]),
                Simd::from_array(std::array::from_fn(|i| {
                    i as f64 / 63.0 * std::f64::consts::TAU
                })),
                sleef::f64x::sincos_u35,
            );

            c.with_child(PolylineBundle {
                polyline: PolylineHandle(polylines.add(Polyline {
                    vertices: std::array::from_fn::<_, 64, _>(|i| get_pos(moon_pos, i)).to_vec(),
                })),
                material: PolylineMaterialHandle(polyline_materials.add(PolylineMaterial {
                    width: 1.0,
                    color: Color::hsl(55.0, 1.0, 1.5).to_linear(),
                    perspective: false,
                    ..Default::default()
                })),
                ..Default::default()
            });
        }
    }

    for i in 8..16 {
        commands.spawn((
            Mesh3d(sphere_mesh.clone()),
            MeshMaterial3d(materials.add(StandardMaterial { ..default() })),
            Transform::IDENTITY,
            SystemBody(i),
        ));
    }

    /*for i in 0..8 {
        let parent_position = nbody_simd::orbital_position_from_mean_anomaly(
            OrbitParams::from_array([system.planets.get(system.moon_parent_swizzles[i]); 64]),
            Simd::from_array(std::array::from_fn(|i| {
                i as f64 / 63.0 * std::f64::consts::TAU
            })),
            sleef::f64x::sincos_u35,
        );

        let positions = parent_position
            + nbody_simd::orbital_position_from_mean_anomaly(
                OrbitParams::from_array([system.moons.get(i); 64]),
                Simd::from_array(std::array::from_fn(|i| {
                    i as f64 / 63.0 * std::f64::consts::TAU
                })),
                sleef::f64x::sincos_u35,
            );

        commands.spawn((PolylineBundle {
            polyline: PolylineHandle(polylines.add(Polyline {
                vertices: std::array::from_fn::<_, 64, _>(|i| get_pos(positions, i)).to_vec(),
            })),
            material: PolylineMaterialHandle(polyline_materials.add(PolylineMaterial {
                width: 1.0,
                color: Color::hsl(55.0, 1.0, 1.5).to_linear(),
                perspective: false,
                ..Default::default()
            })),
            ..Default::default()
        },));

        commands.spawn((
            Mesh3d(sphere_mesh.clone()),
            MeshMaterial3d(materials.add(StandardMaterial { ..default() })),
            Transform::IDENTITY.with_scale(Vec3::splat(50.0)),
            SystemBody(i + 8),
        ));
    }*/

    commands.spawn((
        Camera3dBundle::default(), // The camera which is related via the rig tag
    ));
}

fn update_camera(mut trans: Query<&mut Transform, With<Camera3d>>, mut x: ResMut<UniversalCamera>) {
    let mut t = trans.single_mut();
    x.compute_position();
    let v = x.view_dir();
    *t = Transform::from_translation(DVec3::new(v.x, v.y, v.z).as_vec3() * 400_000.0)
        .looking_at(Vec3::ZERO, Vec3::Y);
}

/*fn update_camera(
    mut mouse_motion_events: EventReader<MouseMotion>,
    mut rig_q: Query<&mut Rig>,
    trans: Query<&Transform, With<DollyPosCtrlMove>>,
    mut config: ResMut<DollyPosCtrlConfig>,
    buttons: Res<ButtonInput<MouseButton>>,
    //grab_config: Res<DollyCursorGrabConfig>,
) {
    dbg!(rig_q.final_transform);

    let mut rig = rig_q.single_mut();
    let camera_yp = rig.driver_mut::<YawPitch>();
    let sensitivity = Vec2::splat(2.0);

    let mut delta = Vec2::ZERO;
    for event in mouse_motion_events.read() {
        delta += event.delta;
    }

    config.transform.rotation = Quat::from_rotation_y(delta.x);

    if buttons.pressed(MouseButton::Left) {
        camera_yp.rotate_yaw_pitch(
            -0.1 * delta.x * sensitivity.x,
            -0.1 * delta.y * sensitivity.y,
        );
    }

    if buttons.pressed(MouseButton::Right) {
        camera_yp.rotate_yaw_pitch(
            -0.1 * delta.x * sensitivity.x,
            -0.1 * delta.y * sensitivity.y,
        );
    }
}

fn handle_mouse_scroll(
    mut mouse_wheel_events: EventReader<MouseWheel>,
    mut q_main: Query<&mut Projection, With<MainCamera>>,
    mut rig_q: Query<&mut Rig>,
) {
    for mouse_wheel_event in mouse_wheel_events.read() {
        if let Ok(mut rig) = rig_q.get_single_mut() {
            if let Some(arm) = rig.try_driver_mut::<Arm>() {
                //let mut xz = arm.offset;
                //xz.z = (xz.z - mouse_wheel_event.y * 0.5).abs();
                arm.offset *= (1.0 + mouse_wheel_event.y * -0.1);
            }
        }
    }
}*/
