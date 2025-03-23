use super::*;

#[allow(clippy::too_many_arguments)]
pub fn update_ui(
    mut contexts: EguiContexts,
    mut followed: ResMut<FollowedBody>,
    mut reference_frame: ResMut<ReferenceFrameBody>,
    mut time: ResMut<SystemTime>,
    mut burns: Query<(Entity, &mut ShipPath, &mut Burn, &mut TrajectoryReceiver)>,
    mut commands: Commands,
    mut path_settings: ResMut<PathSettings>,
    mut selected_burn: ResMut<SelectedBurn>,
    system_bodies: Query<(&Name, &SystemBody)>,
) {
    egui::Window::new("Hello").show(contexts.ctx_mut(), |ui| {
        ui.add(egui::Slider::new(
            &mut path_settings.max_segments,
            1..=10_000_000,
        ));
        ui.add(egui::Slider::new(&mut time.0, 0.0..=100_000_000.0));

        ui.horizontal(|ui| {
            if ui.button("follow none").clicked() {
                followed.0 = None;
            }
            if ui.button("No Ref").clicked() {
                reference_frame.0 = None;
            }
            if ui.button("no burn select").clicked() {
                selected_burn.0 = None;
            }
        });

        for (entity, ..) in burns.iter_mut() {
            ui.horizontal(|ui| {
                ui.label("Burn");
                if ui.button("select").clicked() {
                    selected_burn.0 = Some(entity);
                }
                if ui.button("delete").clicked() {
                    commands.entity(entity).despawn();
                    if selected_burn.0 == Some(entity) {
                        selected_burn.0 = None;
                    }
                }
            });
        }

        if let Some(burn) = selected_burn.0 {
            let (_, mut path, mut burn, mut trajec) = burns.get_mut(burn).unwrap();

            let mut updated = ui
                .add(egui::Slider::new(
                    &mut burn.vector.x,
                    -100_000.0..=100_000.0_f64,
                ))
                .changed();
            updated |= ui
                .add(egui::Slider::new(
                    &mut burn.vector.y,
                    -100_000.0..=100_000.0_f64,
                ))
                .changed();
            updated |= ui
                .add(egui::Slider::new(
                    &mut burn.vector.z,
                    -100_000.0..=100_000.0_f64,
                ))
                .changed();
            if updated {
                let new_round = trajec.expected_round + 1;
                trajec.expected_round = new_round;
                trajec.burn_tx.try_send((burn.vector, new_round)).unwrap();
                path.clear();
            }
        }

        for (name, body) in system_bodies.iter() {
            ui.horizontal(|ui| {
                if ui.button(&**name).clicked() {
                    followed.0 = Some(body.0);
                }
                if ui.button("Set ref").clicked() {
                    reference_frame.0 = Some(body.0);
                }
            });
        }
    });
}
