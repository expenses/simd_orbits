use super::*;

pub fn set_body_path_positions(
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

pub fn set_body_positions(
    system: Res<System>,
    mut query: Query<(&SystemBody, &mut Transform, &BodyRadius)>,
    camera: Res<UniversalCamera>,
) {
    let positions = system.state.planet_and_moon_positions;

    for (body, mut transform, radius) in query.iter_mut() {
        *transform = create_transform(positions.get(body.0).into(), &camera, |distance| {
            (radius.0 / camera.distance).max(distance / 200.0)
        });
    }
}

pub fn set_ship_path_positions(
    camera: Res<UniversalCamera>,
    mut polylines: ResMut<Assets<Polyline>>,
    paths: Query<(&ShipPath, &PolylineHandle)>,
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

        for point in path
            .batches
            .iter()
            .flat_map(|batch| &batch.render_positions)
        {
            let pos = (*point - camera.position) + offset;

            line.vertices
                .push(convert_vec(pos / camera.distance).as_vec3());
        }
    }
}

pub fn set_star_positions(
    mut query: Query<(&mut Transform, &BodyRadius), With<SystemStar>>,
    camera: Res<UniversalCamera>,
) {
    for (mut transform, radius) in query.iter_mut() {
        *transform = create_transform(UniversalPos::default(), &camera, |distance| {
            (radius.0 / camera.distance).max(distance / 75.0)
        });
    }
}

pub fn update_camera(
    mut trans: Query<&mut Transform, With<Camera3d>>,
    camera: Res<UniversalCamera>,
) {
    let mut t = trans.single_mut();
    *t = Transform::from_translation(Vec3::ZERO).looking_at(-camera.view_dir().as_vec3(), Vec3::Y);
}
