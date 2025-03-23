use super::*;

#[allow(clippy::needless_range_loop)]
pub fn setup(
    mut commands: Commands,
    mut polyline_materials: ResMut<Assets<PolylineMaterial>>,
    mut polylines: ResMut<Assets<Polyline>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    mut color_chooser: ResMut<ColorChooser>,
) {
    let planet_info = [
        ("Mercury", "2k_mercury.jpg"),
        ("Venus", "2k_venus_atmosphere.jpg"),
        ("Earth", "2k_earth_daymap.jpg"),
        ("Mars", "2k_mars.jpg"),
        ("Jupiter", "2k_jupiter.jpg"),
        ("Saturn", "2k_saturn.jpg"),
        ("Uranus", "2k_uranus.jpg"),
        ("Neptune", "2k_neptune.jpg"),
    ];

    let sphere_mesh = asset_server.load("planet.glb#Mesh0/Primitive0");
    let saturn_rings = asset_server.load("saturn_rings.glb#Mesh0/Primitive0");

    commands.insert_resource(SphereMesh(sphere_mesh.clone()));

    commands.spawn((
        BurnPreviewLocation,
        Mesh3d(sphere_mesh.clone()),
        MeshMaterial3d(materials.add(StandardMaterial {
            perceptual_roughness: 1.0,
            base_color_texture: None,
            unlit: true,
            ..Default::default()
        })),
        Transform::IDENTITY,
    ));

    let system = nbody_simd::System::sol();

    let colour = Color::hsl(55.0, 0.75, 1.5).to_linear();

    {
        let mut pos = UniversalPos::new_3(57_909_048_000.0 / 2.0, 0.0, 0.0);
        let mut vel = nbody_simd::Vec3::new(0.0, 0.0, 100000.0);
        let start_pos = pos;
        let start_vel = vel;
        let mut positions = vec![pos];
        let mut velocities = vec![vel];
        let timestep = 10.0;
        for i in 0..1_000_000 {
            vel += system
                .state_at(i as f64 * timestep)
                .acceleration_at(pos.as_vec3())
                * timestep;
            pos += vel * timestep;
            positions.push(pos);
            velocities.push(vel);
        }
        let batch = PathBatch {
            positions,
            velocities,
            adapted_positions: Default::default(),
            render_positions: Default::default(),
            min: Default::default(),
            max: Default::default(),
            step: timestep,
        };
        commands.spawn((
            ShipPath {
                start: 0.0,
                start_pos,
                start_vel,
                total_duration: batch.duration(),
                batches: vec![batch],
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
        let (name, image_filename) = planet_info[i];
        let radius = nbody_simd::System::SOL_PLANETS[i].1;

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

    let moon_info = [
        ("Luna", "2k_moon.jpg"),
        ("Phobos", "phobos.jpg"),
        ("Deimos", "deimos.jpg"),
        ("Io", "io.jpg"),
        ("Europa", "europa.png"),
        ("Ganymede", "ganymede_2k_downscaled.png"),
        ("Callisto", "callisto.jpg"),
        ("Titan", "2k_titan.png"),
    ];

    for i in 0..8 {
        let (name, image_filename) = moon_info[i];
        let radius = nbody_simd::System::SOL_MOONS[i].1;

        commands.spawn((
            Mesh3d(sphere_mesh.clone()),
            MeshMaterial3d(materials.add(StandardMaterial {
                perceptual_roughness: 1.0,
                base_color_texture: Some(asset_server.load(image_filename)),
                unlit: true,
                ..Default::default()
            })),
            Transform::IDENTITY,
            SystemBody(i + 8),
            Name::new(name),
            BodyRadius(radius),
        ));
    }

    commands.spawn((Camera3d::default(),));
}
