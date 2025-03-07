use super::*;

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
                .push((pos / SCALE).as_vec3());
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

        let reference_frame_at_time = reference_frame
            .0
            .into_iter()
            .flat_map(|body| {
                get_reference_frame_offset(path.start, &system, body, settings.decimation)
            })
            .chain(std::iter::repeat(Default::default()));

        let mut time = path.start;

        for (point, ref_offset) in path
            .positions
            .iter()
            .step_by(settings.decimation)
            .zip(reference_frame_at_time)
            .take(settings.max_segments / settings.decimation)
        {
            let mut pos = *point - camera.position;
            pos += ref_offset;

            line.vertices
                // Move the lines closer to the camera for better stability on opengl.
                .push(convert_vec(pos / SCALE).as_vec3());

            time += 10.0 * settings.decimation as f64;
        }
    }
}

pub fn get_closest_path_point(
    mut contexts: EguiContexts,
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

    let cone_dir =
        if let Ok(ray) = render_cam.viewport_to_world(render_cam_transform, cursor_position) {
            ray
        } else {
            return;
        }
        .direction
        .as_vec3()
        .as_dvec3();

    let compare_dist = 20.0;
    let step = 10;

    let picked_position = paths
        .iter()
        .flat_map(|path| {
            let reference_frame_at_time = reference_frame
                .0
                .into_iter()
                .flat_map(|body| get_reference_frame_offset(path.start, &system, body, step))
                .chain(std::iter::repeat(Default::default()));

            path.positions
                .iter()
                .enumerate()
                .step_by(step)
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

            commands.spawn((
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
