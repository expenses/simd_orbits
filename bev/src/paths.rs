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
    let offset = if let Some(body) = reference_frame.0 {
        system.state.planet_and_moon_positions.get(body)
    } else {
        Default::default()
    };

    for (path, handle) in paths.iter() {
        let line = polylines.get_mut(&handle.0).unwrap();
        line.vertices.clear();

        let decimation: usize = settings.decimation;
        let system_system: nbody_simd::System = system.system;

        for point in path
            .batches
            .iter()
            .flat_map(|batch| &batch.render_positions)
        //.step_by(settings.decimation)
        //.take(settings.max_segments / settings.decimation)
        {
            let pos = (*point - camera.position) + offset;

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
    let mut cursor_position =
        if let Some(cursor_position) = primary_window.and_then(|window| window.cursor_position()) {
            cursor_position
        } else {
            return;
        };

    // Have this after the primary window check.
    if ui_params.contexts.ctx_mut().is_using_pointer() || ui_params.cursor_on_ui_element.0.is_some()
    {
        return;
    }

    let offset = if let Some(body) = reference_frame.0 {
        system.state.planet_and_moon_positions.get(body)
    } else {
        Default::default()
    };

    let (render_cam, render_cam_transform) = *render_cam;

    let ray = if let Ok(ray) = render_cam.viewport_to_world(render_cam_transform, cursor_position) {
        ray
    } else {
        return;
    };

    let cone_dir = ray.direction.as_vec3().as_dvec3();

    let reference_frame: &ReferenceFrameBody = &reference_frame;
    let decimation = settings.selection_decimation;
    let system: &System = &system;
    let camera: &UniversalCamera = &camera;

    let picked_position = paths
        .iter()
        .flat_map(|path| {
            path.batches
                .iter()
                .enumerate()
                .flat_map(move |(batch_id, batch)| {
                    let min = convert_vec(batch.min - camera.position);
                    let max = convert_vec(batch.max - camera.position);

                    let center = (min + max) / 2.0;
                    let radius = (max - min).length();

                    /*let aabb = bevy::math::bounding::Aabb3d {
                        min: (convert_vec(batch.min - camera.position) / camera.distance)
                            .as_vec3()
                            .into(),
                        max: (convert_vec(batch.max - camera.position) / camera.distance)
                            .as_vec3()
                            .into(),
                    };

                    if bevy::math::bounding::RayCast3d::from_ray(ray, 1000.0)
                        .aabb_intersection_at(&aabb)
                        .is_none()
                    {
                        return None;
                    }*/

                    Some(
                        batch
                            .adapted_positions
                            .iter()
                            .enumerate()
                            .map(move |(i, pos)| (batch_id, i, path, pos))
                            .step_by(decimation),
                    )
                })
                .flatten()
                .take(settings.max_segments)
        })
        .map(|((batch_id, i, path, &pos))| {
            let relative = convert_vec((pos - camera.position) + offset);
            let dir = relative.normalize();
            let closeness = dir.dot(cone_dir);
            (batch_id, i, path, closeness)
        })
        .max_by_key(|&(.., cosine_angle)| ordered_float::OrderedFloat(cosine_angle))
        .filter(|&(.., cosine_angle)| cosine_angle > 0.9995);

    if let Some((batch_id, index, origin_path, x)) = picked_position {
        if buttons.just_pressed(MouseButton::Left) {
            let start = origin_path.start
                + origin_path
                    .batches
                    .iter()
                    .take(batch_id)
                    .map(|batch| batch.step * batch.positions.len() as f64)
                    .sum::<f64>();
            if reference_frame.0.is_none() {
                time.0 = start;
            }

            let batch = &origin_path.batches[batch_id];

            let vel = batch.velocities[index];
            let pos = batch.positions[index];

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
                        start_pos: pos,
                        start_vel: vel,
                        batches: Vec::new(),
                        total_duration: 0.0
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

            /*let x_pos = commands.spawn((
                AssociatedTrajectory(trajectory),
                Mesh3d(sphere_mesh.0.clone()),
                material.clone(),
                Transform::IDENTITY,
                UniversalObjectPos {
                    pos,
                    offset: DVec3::new(5.0, 0.0, 0.0),
                },
            ));*/
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
    path.batches.clear();
}

pub fn adapt_batch_to_reference_frame(
    time: f64,
    batch: &mut PathBatch,
    reference_frame: &ReferenceFrameBody,
    system: &System,
) {
    if let Some(body) = reference_frame.0 {
        let offsets = get_reference_frame_offset(time, system, body, batch.step);
        batch.adapted_positions.clear();

        batch.adapted_positions.extend(
            batch
                .positions
                .iter()
                .zip(offsets)
                .map(|(pos, offset)| *pos + offset),
        );
    } else {
        batch.adapted_positions.clone_from(&batch.positions);
    }

    for &position in &batch.adapted_positions {
        batch.min = batch.min.min(position);
        batch.max = batch.max.max(position);
    }

    decimate(&batch.adapted_positions, &mut batch.render_positions);
}

pub fn adapt_paths_to_new_reference_frame(
    reference_frame: Res<ReferenceFrameBody>,
    system: Res<System>,
    mut query: Query<&mut ShipPath>,
) {
    if !reference_frame.is_changed() {
        return;
    }

    for mut path in query.iter_mut() {
        let mut time = path.start;

        for batch in &mut path.batches {
            adapt_batch_to_reference_frame(time, batch, &reference_frame, &system);
            time += batch.positions.len() as f64 * batch.step;
        }
    }
}
fn decimate(points: &[UniversalPos], output: &mut Vec<UniversalPos>) {
    output.clear();
    output.push(points[0]);
    let mut vector = convert_vec(points[1] - points[0]).normalize();
    let mut last = points[1];

    for &point in &points[2..] {
        let new_vector = convert_vec(point - *output.last().unwrap()).normalize();
        if new_vector.dot(vector) <= 0.5_f64.to_radians().cos() {
            output.push(last);
            vector = convert_vec(point - last).normalize();
        }

        last = point;
    }

    output.push(last);
}
