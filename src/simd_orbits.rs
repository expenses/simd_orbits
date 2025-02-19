use std::simd::{self, LaneCount, Simd, SimdElement, SupportedLaneCount};

// all units in meters
// all masses in kg

fn calculate_mean_motion<T: num_traits::Float>(
    combined_mass_div_gravitational_constant: T,
    semi_major_axis: T,
) -> T {
    (combined_mass_div_gravitational_constant
        / (semi_major_axis * semi_major_axis * semi_major_axis))
        .sqrt()
}

fn calculate_semi_minor_axis<T: num_traits::Float>(semi_major_axis: T, eccentricity: T) -> T {
    semi_major_axis * (T::one() - (eccentricity * eccentricity)).sqrt()
}

// https://en.wikipedia.org/wiki/Gravitational_constant
// in m^3 kg^-1 s^-2
const G: f64 = 6.6743015e-11;

#[derive(Clone, Copy)]
pub struct OrbitParams<T> {
    pub mean_motion: T,
    pub eccentricity: T,
    pub initial_mean_anomaly: T,
    pub semi_major_axis: T,
    pub semi_minor_axis: T,
    pub argument_of_periapsis_sin: T,
    pub argument_of_periapsis_cos: T,
    pub mass_div_gravitational_constant: T,
}

impl<T: num_traits::Float + From<f64>> OrbitParams<T> {
    #[inline]
    pub fn new(
        parent_mass: T,
        mass: T,
        eccentricity: T,
        initial_mean_anomaly: T,
        semi_major_axis: T,
        argument_of_periapsis: T,
    ) -> Self {
        Self {
            eccentricity,
            initial_mean_anomaly,
            mean_motion: calculate_mean_motion((parent_mass + mass) * G.into(), semi_major_axis),
            semi_major_axis,
            semi_minor_axis: calculate_semi_minor_axis(semi_major_axis, eccentricity),
            argument_of_periapsis_sin: argument_of_periapsis.sin(),
            argument_of_periapsis_cos: argument_of_periapsis.cos(),
            mass_div_gravitational_constant: mass * G.into(),
        }
    }
}

impl<T: SimdElement, const N: usize> OrbitParams<Simd<T, N>>
where
    LaneCount<N>: SupportedLaneCount,
{
    fn from_array(params: [OrbitParams<T>; N]) -> Self {
        Self {
            mean_motion: Simd::from_array(params.map(|param| param.mean_motion)),
            eccentricity: Simd::from_array(params.map(|param| param.eccentricity)),
            initial_mean_anomaly: Simd::from_array(params.map(|param| param.initial_mean_anomaly)),
            semi_major_axis: Simd::from_array(params.map(|param| param.semi_major_axis)),
            semi_minor_axis: Simd::from_array(params.map(|param| param.semi_minor_axis)),
            argument_of_periapsis_sin: Simd::from_array(
                params.map(|param| param.argument_of_periapsis_sin),
            ),
            argument_of_periapsis_cos: Simd::from_array(
                params.map(|param| param.argument_of_periapsis_cos),
            ),
            mass_div_gravitational_constant: Simd::from_array(
                params.map(|param| param.mass_div_gravitational_constant),
            ),
        }
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct Vec3<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T> Vec3<T> {
    #[inline]
    pub fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }
}

impl<T: SimdElement, const N: usize> Vec3<Simd<T, N>>
where
    LaneCount<N>: SupportedLaneCount,
{
    fn splat(vec: Vec3<T>) -> Self {
        Self::new(Simd::splat(vec.x), Simd::splat(vec.y), Simd::splat(vec.z))
    }
}

impl<T: FloatSimd> Vec3<T> {
    fn reduce_sum(self) -> Vec3<T::Scalar> {
        Vec3::new(
            self.x.reduce_sum(),
            self.y.reduce_sum(),
            self.z.reduce_sum(),
        )
    }
}

impl<T: std::ops::Add<Output = T> + std::ops::Mul<Output = T> + Copy> Vec3<T> {
    #[inline]
    pub fn length_squared(&self) -> T {
        self.x * self.x + self.y * self.y + self.z * self.z
    }
}

impl<T: std::ops::Add<Output = T>> std::ops::Add<Self> for Vec3<T> {
    type Output = Self;

    #[inline]
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl<T: std::ops::Sub<Output = T>> std::ops::Sub<Self> for Vec3<T> {
    type Output = Self;

    #[inline]
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl<T: std::ops::Div<Output = T> + Copy> std::ops::Div<T> for Vec3<T> {
    type Output = Self;

    #[inline]
    fn div(self, scalar: T) -> Self {
        Self {
            x: self.x / scalar,
            y: self.y / scalar,
            z: self.z / scalar,
        }
    }
}

impl<T: std::ops::Mul<Output = T> + Copy> std::ops::Mul<T> for Vec3<T> {
    type Output = Self;

    #[inline]
    fn mul(self, scalar: T) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

#[inline]
pub fn get_acceleration<T: FloatSimd>(body_position: Vec3<T>, mu: T, point: Vec3<T>) -> Vec3<T> {
    let dir = body_position - point;
    let mag_2 = dir.length_squared();
    let force = dir / (mag_2 * mag_2.sqrt());
    let accel = force * mu;
    accel
}

#[inline]
pub fn orbital_position_simd<T: FloatSimd, SC: Fn(T) -> (T, T)>(
    params: OrbitParams<T>,
    time: T,
    sincos: SC,
) -> Vec3<T> {
    let mean_anomaly = params.initial_mean_anomaly + params.mean_motion * time;
    orbital_position_from_mean_anomaly(params, mean_anomaly, sincos)
}

#[inline]
pub fn orbital_position_from_mean_anomaly<T: FloatSimd, SC: Fn(T) -> (T, T)>(
    params: OrbitParams<T>,
    mean_anomaly: T,
    sincos: SC,
) -> Vec3<T> {
    let (_eccentric_anomaly, eccentric_anomaly_sin, eccentric_anomaly_cos) =
        newton_raphson_kepler_simd(mean_anomaly, params.eccentricity, sincos);

    let x_prime = params.semi_major_axis * (eccentric_anomaly_cos - params.eccentricity);
    let y_prime = params.semi_minor_axis * eccentric_anomaly_sin;

    let x = x_prime * params.argument_of_periapsis_cos - y_prime * params.argument_of_periapsis_sin;
    let y = x_prime * params.argument_of_periapsis_sin + y_prime * params.argument_of_periapsis_cos;

    Vec3::new(x, y, Default::default())
}

pub trait FloatSimd:
    std::ops::Add<Output = Self>
    + std::ops::Mul<Output = Self>
    + std::ops::Sub<Output = Self>
    + std::ops::Div<Output = Self>
    + std::ops::SubAssign
    + Sized
    + Copy
    + simd::num::SimdFloat
    + simd::StdFloat
    + Default
{
    fn one() -> Self;

    fn splat_f64(v: f64) -> Self;
}

impl<const N: usize> FloatSimd for Simd<f32, N>
where
    LaneCount<N>: SupportedLaneCount,
{
    #[inline]
    fn one() -> Self {
        Simd::splat(1.0)
    }

    #[inline]
    fn splat_f64(v: f64) -> Self {
        Self::splat(v as f32)
    }
}
impl<const N: usize> FloatSimd for Simd<f64, N>
where
    LaneCount<N>: SupportedLaneCount,
{
    #[inline]
    fn one() -> Self {
        Simd::splat(1.0)
    }

    #[inline]
    fn splat_f64(v: f64) -> Self {
        Self::splat(v)
    }
}

#[inline]
pub fn newton_raphson_kepler_simd<T: FloatSimd, SC: Fn(T) -> (T, T)>(
    mean_anomaly: T,
    eccentricity: T,
    sincos: SC,
) -> (T, T, T) {
    let mut eccentric_anomaly = mean_anomaly;
    let (eccentric_anomaly_sin, eccentric_anomaly_cos) = sincos(eccentric_anomaly);

    let f = eccentric_anomaly - eccentricity * eccentric_anomaly_sin - mean_anomaly;
    let f_prime = T::one() - eccentricity * eccentric_anomaly_cos;
    let delta = f / f_prime;
    eccentric_anomaly -= delta;
    let (eccentric_anomaly_sin, eccentric_anomaly_cos) = sincos(eccentric_anomaly);

    let f = eccentric_anomaly - eccentricity * eccentric_anomaly_sin - mean_anomaly;
    let f_prime = T::one() - eccentricity * eccentric_anomaly_cos;
    let delta = f / f_prime;
    eccentric_anomaly -= delta;
    let (eccentric_anomaly_sin, eccentric_anomaly_cos) = sincos(eccentric_anomaly);

    let f = eccentric_anomaly - eccentricity * eccentric_anomaly_sin - mean_anomaly;
    let f_prime = T::one() - eccentricity * eccentric_anomaly_cos;
    let delta = f / f_prime;
    eccentric_anomaly = eccentric_anomaly - delta;
    let (eccentric_anomaly_sin, eccentric_anomaly_cos) = sincos(eccentric_anomaly);

    (
        eccentric_anomaly,
        eccentric_anomaly_sin,
        eccentric_anomaly_cos,
    )
}

pub fn calculate_acceleration_for_system(point: Vec3<f64>, time: f64) -> Vec3<f64> {
    let sol_mass_kg = 1.9885E30;

    let planets = OrbitParams::from_array(
        [OrbitParams::new(
            sol_mass_kg,
            5972167981508606000000000.0,
            0.016708599403500557,
            0.0,
            149598016_000.0,
            0.0,
        ); 8],
    );

    let sun_accel = get_acceleration::<Simd<f64, 1>>(
        Vec3::default(),
        Simd::splat_f64(sol_mass_kg * G),
        Vec3::splat(point),
    );

    let planet_positions =
        orbital_position_simd(planets, Simd::splat(time), sleef::f64x::sincos_u35);

    sun_accel.reduce_sum()
        + get_acceleration(
            planet_positions,
            planets.mass_div_gravitational_constant,
            Vec3::splat(point),
        ).reduce_sum()
}

#[test]
fn testing() {
    let accel = calculate_acceleration_for_system(Vec3::new(0.0, 0.0, 149598016_000.0), 100.0);

    dbg!(accel);

    /*dbg!(orbital_position_simd(
        params,
        Simd::splat(0.0),
        sleef::f64x::sincos_u35
    ));

    dbg!(orbital_position_simd(
        params,
        Simd::splat(1.0),
        sleef::f64x::sincos_u35
    ));

    dbg!(orbital_position_simd(
        params,
        Simd::splat(60.0 * 60.0 * 24.0 * 365.25),
        sleef::f64x::sincos_u35
    ));*/

    panic!()
}
