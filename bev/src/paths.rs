use super::*;
use bevy::ecs::system::SystemParam;

pub fn set_path_positions(
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
                .push((pos / camera.distance).as_vec3());
        }
    }
}

pub fn set_ship_path_positions(
    camera: Res<UniversalCamera>,
    mut polylines: ResMut<Assets<Polyline>>,
    paths: Query<(&ShipPath, &PolylineHandle)>,
    settings: Res<PathSettings>,
    system: Res<System>,
    reference_frame: Res<ReferenceFrameBody>,
) {
    for (path, handle) in paths.iter() {
        let line = polylines.get_mut(&handle.0).unwrap();
        line.vertices.clear();

        let decimation: usize = settings.decimation;
        let system_system: nbody_simd::System = system.system;

        let reference_frame_at_time =
            reference_frame.get_offsets(path.start, &system, settings.decimation);

        for (point, ref_offset) in path
            .positions
            .iter()
            .step_by(settings.decimation)
            .zip(reference_frame_at_time)
            .take(settings.max_segments / settings.decimation)
        {
            let pos = (*point - camera.position) + ref_offset;

            line.vertices
                // Move the lines closer to the camera for better stability on opengl.
                .push(convert_vec(pos / camera.distance).as_vec3());
        }
    }
}

#[derive(SystemParam)]
pub struct UiParams<'w, 's> {
    cursor_on_ui_element: Res<'w, CursorOnUiElement>,
    contexts: EguiContexts<'w, 's>,
}

pub fn get_closest_path_point(
    mut ui_params: UiParams,
    camera: Res<UniversalCamera>,
    render_cam: Single<(&Camera, &GlobalTransform)>,
    paths: Query<&ShipPath>,
    mut time: ResMut<SystemTime>,
    primary_window: Option<Single<&Window>>,
    buttons: Res<ButtonInput<MouseButton>>,
    mut polyline_materials: ResMut<Assets<PolylineMaterial>>,
    mut polylines: ResMut<Assets<Polyline>>,
    mut commands: Commands,
    settings: Res<PathSettings>,
    reference_frame: Res<ReferenceFrameBody>,
    system: Res<System>,
    mut color_chooser: ResMut<ColorChooser>,
    sphere_mesh: Res<SphereMesh>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if ui_params.contexts.ctx_mut().is_using_pointer() || ui_params.cursor_on_ui_element.0.is_some()
    {
        return;
    }

    let mut cursor_position =
        if let Some(cursor_position) = primary_window.and_then(|window| window.cursor_position()) {
            cursor_position
        } else {
            return;
        };

    let (render_cam, render_cam_transform) = *render_cam;

    let cone_dir =
        if let Ok(ray) = render_cam.viewport_to_world(render_cam_transform, cursor_position) {
            ray
        } else {
            return;
        }
        .direction
        .as_vec3()
        .as_dvec3();

    let picked_position = paths
        .iter()
        .flat_map(|path| {
            let reference_frame_at_time =
                reference_frame.get_offsets(path.start, &system, settings.selection_decimation);

            path.positions
                .iter()
                .enumerate()
                .step_by(settings.selection_decimation)
                .map(move |(i, pos)| (i, path, pos))
                .zip(reference_frame_at_time)
                .take(settings.max_segments)
        })
        .map(|((i, path, &pos), ref_offset)| {
            let relative = convert_vec((pos - camera.position) + ref_offset);
            let dir = relative.normalize();
            let closeness = dir.dot(cone_dir);
            (i, path, closeness)
        })
        .max_by_key(|&(_, _, cosine_angle)| ordered_float::OrderedFloat(cosine_angle))
        .filter(|&(_, _, cosine_angle)| cosine_angle > 0.9995);

    if let Some((index, origin_path, x)) = picked_position {
        if buttons.just_pressed(MouseButton::Left) {
            let start = index as f64 * 10.0 + origin_path.start;
            if reference_frame.0.is_none() {
                time.0 = start;
            }
            let vel = origin_path.velocities[index];
            let pos = origin_path.positions[index];

            let (output_tx, output_rx) = sync_channel(100);
            let (burn_tx, burn_rx) = sync_channel(10);

            std::thread::spawn(move || {
                trajectory_calculator(
                    output_tx,
                    burn_rx,
                    start,
                    pos,
                    vel,
                    nbody_simd::System::sol(),
                );
            });

            let material = MeshMaterial3d(materials.add(StandardMaterial {
                perceptual_roughness: 1.0,
                base_color_texture: None,
                unlit: true,
                ..Default::default()
            }));
            let trajectory = commands
                .spawn((
                    TrajectoryReceiver {
                        expected_round: 0,
                        inner: output_rx,
                    },
                    BurnTx(burn_tx),
                    ShipPath {
                        start,
                        velocities: vec![vel],
                        positions: vec![pos],
                    },
                    Burn {
                        vector: Default::default(),
                    },
                    PolylineBundle {
                        polyline: PolylineHandle(polylines.add(Polyline::default())),
                        material: PolylineMaterialHandle(polyline_materials.add(
                            PolylineMaterial {
                                width: 1.0,
                                color: color_chooser.next(),
                                perspective: false,
                                ..Default::default()
                            },
                        )),
                        ..Default::default()
                    },
                ))
                .id();

            let x_pos = commands.spawn((
                AssociatedTrajectory(trajectory),
                Mesh3d(sphere_mesh.0.clone()),
                material.clone(),
                Transform::IDENTITY,
                UniversalObjectPos {
                    pos,
                    offset: DVec3::new(5.0, 0.0, 0.0),
                },
            ));
        }
    }
}

#[derive(Component)]
pub struct AssociatedTrajectory(Entity);

pub fn trajectory_handles(
    render_cam: Single<(&Camera, &GlobalTransform)>,
    query: Query<(&Transform, &AssociatedTrajectory)>,
    primary_window: Option<Single<&Window>>,
    buttons: Res<ButtonInput<MouseButton>>,
    mut cursor_on_ui_element: ResMut<CursorOnUiElement>,
) {
    if buttons.pressed(MouseButton::Left) {
        return;
    }

    let mut cursor_position =
        if let Some(cursor_position) = primary_window.and_then(|window| window.cursor_position()) {
            cursor_position
        } else {
            return;
        };

    let (render_cam, render_cam_transform) = *render_cam;

    let ray = if let Ok(ray) = render_cam.viewport_to_world(render_cam_transform, cursor_position) {
        ray
    } else {
        return;
    };

    cursor_on_ui_element.0 = None;

    for (transform, assoc) in query.iter() {
        if bevy::math::bounding::RayCast3d::from_ray(ray, 1000.0)
            .sphere_intersection_at(&bevy::math::bounding::BoundingSphere {
                center: transform.translation.into(),
                sphere: Sphere {
                    radius: transform.scale.x,
                },
            })
            .is_some()
        {
            cursor_on_ui_element.0 = Some(assoc.0);
            dbg!(());
        }
    }
}

pub fn adjust_trajectory(
    render_cam: Single<(&Camera, &GlobalTransform)>,
    query: Query<(&Transform, &AssociatedTrajectory)>,
    primary_window: Option<Single<&Window>>,
    buttons: Res<ButtonInput<MouseButton>>,
    mut cursor_on_ui_element: Res<CursorOnUiElement>,
    mut burns: Query<(&mut ShipPath, &mut Burn, &BurnTx, &mut TrajectoryReceiver)>,
    mut mouse_motion_events: EventReader<MouseMotion>,
) {
    if !buttons.pressed(MouseButton::Left) {
        return;
    }

    let associated_trajectory = if let Some(assoc) = cursor_on_ui_element.0 {
        assoc
    } else {
        return;
    };

    let (mut path, mut burn, burn_tx, mut trajec) = burns.get_mut(associated_trajectory).unwrap();

    let mut delta = Vec2::ZERO;
    for event in mouse_motion_events.read() {
        delta += event.delta;
    }

    if delta == Vec2::ZERO {
        return;
    }

    burn.vector.x += delta.x as f64 * 1000.0;
    let new_round = trajec.expected_round + 1;
    trajec.expected_round = new_round;
    burn_tx.0.send((burn.vector, new_round)).unwrap();
    path.positions.clear();
    path.velocities.clear();
}
