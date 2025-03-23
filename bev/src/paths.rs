use super::*;
use bevy::ecs::system::SystemParam;
use bevy::tasks::AsyncComputeTaskPool;

#[derive(SystemParam)]
pub struct UiParams<'w, 's> {
    cursor_on_ui_element: Res<'w, CursorOnUiElement>,
    contexts: EguiContexts<'w, 's>,
}

#[allow(clippy::too_many_arguments)]
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
    reference_frame: Res<ReferenceFrameBody>,
    system: Res<System>,
    mut color_chooser: ResMut<ColorChooser>,
    sphere_mesh: Res<SphereMesh>,
    mut burn_preview_position: Single<&mut Transform, With<BurnPreviewPosition>>,
    mut selected_burn: ResMut<SelectedBurn>,
) {
    let cursor_position =
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
    let camera: &UniversalCamera = &camera;

    let cone_angle = 0.2_f64.to_radians();

    let picked_position = paths
        .iter()
        .flat_map(|path| {
            path.batches
                .iter()
                .enumerate()
                .flat_map(move |(batch_id, batch)| {
                    let min = convert_vec((batch.min - camera.position) + offset);
                    let max = convert_vec((batch.max - camera.position) + offset);

                    let aabb = bevy::math::bounding::Aabb3d {
                        min: (min / camera.distance).as_vec3().into(),
                        max: (max / camera.distance).as_vec3().into(),
                    };

                    if bevy::math::bounding::RayCast3d::from_ray(ray, 1000.0)
                        .aabb_intersection_at(&aabb)
                        .is_none()
                    {
                        return None;
                    }

                    Some(
                        batch
                            .adapted_positions
                            .iter()
                            .enumerate()
                            .map(move |(i, pos)| (batch_id, i, path, pos)),
                    )
                })
                .flatten()
        })
        .map(|(batch_id, i, path, &pos)| {
            let relative = convert_vec((pos - camera.position) + offset);
            let dir = relative.normalize();
            let cosine_angle = dir.dot(cone_dir);
            (batch_id, i, path, cosine_angle)
        })
        .max_by_key(|&(.., cosine_angle)| ordered_float::OrderedFloat(cosine_angle))
        .filter(|&(.., cosine_angle)| cosine_angle > cone_angle.cos());

    let (batch_id, index, origin_path, _) = if let Some(path) = picked_position {
        path
    } else {
        **burn_preview_position = Transform::from_scale(Default::default());

        return;
    };

    let batch = &origin_path.batches[batch_id];

    **burn_preview_position = create_transform(
        batch.adapted_positions[index] + offset,
        &camera,
        |distance| distance / 100.0,
    );

    let start = origin_path.start
        + origin_path
            .batches
            .iter()
            .take(batch_id)
            .map(|batch| batch.duration())
            .sum::<f64>()
        + index as f64 * batch.step;

    if buttons.just_pressed(MouseButton::Right) {
        time.0 = start;
        return;
    }

    let vel = batch.velocities[index];
    let pos = batch.positions[index];

    if !buttons.just_pressed(MouseButton::Left) {
        return;
    }

    let task_pool = AsyncComputeTaskPool::get();

    let (output_tx, output_rx) = bounded(100);
    let (burn_tx, burn_rx) = bounded(10);

    let handle = task_pool.spawn(trajectory_calculator(
        output_tx,
        burn_rx,
        start,
        pos,
        vel,
        nbody_simd::System::sol(),
    ));

    let trajectory = commands
        .spawn((
            TrajectoryReceiver {
                expected_round: 0,
                inner: output_rx,
                burn_tx,
                handle,
            },
            ShipPath {
                start,
                start_pos: pos,
                start_vel: vel,
                batches: Vec::new(),
                total_duration: 0.0,
            },
            Burn {
                vector: Default::default(),
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
        ))
        .id();

    selected_burn.0 = Some(trajectory);
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

    let cursor_position =
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
    buttons: Res<ButtonInput<MouseButton>>,
    cursor_on_ui_element: Res<CursorOnUiElement>,
    mut burns: Query<(&mut ShipPath, &mut Burn, &mut TrajectoryReceiver)>,
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

    let (mut path, mut burn, mut trajec) = burns.get_mut(associated_trajectory).unwrap();

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
    trajec.burn_tx.try_send((burn.vector, new_round)).unwrap();
    path.clear();
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
        path.total_duration = 0.0;

        let ShipPath {
            batches,
            total_duration,
            start,
            ..
        } = &mut *path;

        for batch in batches {
            adapt_batch_to_reference_frame(
                *start + *total_duration,
                batch,
                &reference_frame,
                &system,
            );
            *total_duration += batch.duration();
        }
    }
}
fn decimate(points: &[UniversalPos], output: &mut Vec<UniversalPos>) {
    output.clear();
    output.push(points[0]);
    let mut vector = convert_vec(points[1] - points[0]).normalize();
    let mut last = points[1];

    for &point in &points[2..] {
        let new_vector = convert_vec(point - last).normalize();
        if new_vector.dot(vector) <= 0.1_f64.to_radians().cos() {
            output.push(last);
            vector = new_vector;
        }

        last = point;
    }

    output.push(last);
}
