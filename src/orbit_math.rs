// Modified from https://github.com/atbentley/bevy_mod_orbits
// MIT licensed by atbentley <Andrew Bentley>

use std::f64::consts::{PI, TAU};

const G: f64 = 6.67384e-11;

#[inline]
pub fn calculate_position_at_time(
    semi_major_axis: f64,
    eccentricity: f64,
    argument_of_periapsis: f64,
    initial_mean_anomaly: f64,
    parent_mass: f64,
    time: f64,
) -> (f64, f64) {
    let period = calculate_period(semi_major_axis, parent_mass);
    let mean_motion = calculate_mean_motion(period);
    let mean_anomaly = calculate_mean_anomaly(mean_motion, initial_mean_anomaly, time);
    let eccentric_anomaly = calculate_eccentric_anomaly(eccentricity, mean_anomaly);
    let true_anomaly = calculate_true_anomaly(eccentricity, eccentric_anomaly);
    let heliocentric_distance =
        calculate_heliocentric_distance(semi_major_axis, eccentricity, true_anomaly);
    calculate_position(
        true_anomaly,
        heliocentric_distance,
        argument_of_periapsis,
        mean_anomaly,
    )
}

#[inline]
pub fn calculate_period(semi_major_axis: f64, parent_mass: f64) -> f64 {
    TAU * (semi_major_axis.powi(3) / (G * parent_mass)).sqrt()
}

#[inline]
pub fn calculate_mean_motion(period: f64) -> f64 {
    TAU / period
}

#[inline]
pub fn calculate_mean_anomaly(mean_motion: f64, initial_mean_anomaly: f64, time: f64) -> f64 {
    (initial_mean_anomaly + mean_motion * time).rem_euclid(TAU)
}

#[inline]
pub fn calculate_initial_mean_anomaly(mean_anomaly: f64, period: f64, time: f64) -> f64 {
    let mean_motion = calculate_mean_motion(period);
    (mean_anomaly - mean_motion * time).rem_euclid(TAU)
}

#[inline]
pub fn calculate_eccentric_anomaly(eccentricity: f64, mean_anomaly: f64) -> f64 {
    let e = eccentricity;
    let ma = mean_anomaly;
    let mut ea = ma;
    // using Newton's method
    for _i in 0..5 {
        ea = ea - (ea - e * ea.sin() - ma) / (1.0 - e * ea.cos());
    }
    ea
}

#[inline]
pub fn calculate_true_anomaly(eccentricity: f64, eccentric_anomaly: f64) -> f64 {
    let e = eccentricity;
    let e_a = eccentric_anomaly;
    2.0 * (((1.0 + e) / (1.0 - e) * ((e_a / 2.0).tan()).powi(2)).sqrt()).atan()
}

#[inline]
pub fn calculate_heliocentric_distance(
    semi_major_axis: f64,
    eccentricity: f64,
    true_anomaly: f64,
) -> f64 {
    let semilatus_rectum = semi_major_axis * (1.0 - eccentricity.powi(2));
    semilatus_rectum / (1.0 + eccentricity * true_anomaly.cos())
}

#[inline]
pub fn calculate_position(
    true_anomaly: f64,
    heliocentric_distance: f64,
    argument_of_periapsis: f64,
    mean_anomaly: f64,
) -> (f64, f64) {
    let zmod = if (mean_anomaly % TAU) < PI { -1.0 } else { 1.0 };

    let x = heliocentric_distance * true_anomaly.cos();
    let z = heliocentric_distance * true_anomaly.sin() * zmod;

    let rotated_x = x * argument_of_periapsis.cos() - z * argument_of_periapsis.sin();
    let rotated_z = x * argument_of_periapsis.sin() + z * argument_of_periapsis.cos();

    (rotated_x, rotated_z)
}
