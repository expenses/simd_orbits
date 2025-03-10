#![feature(portable_simd)]

use std::{
    ops::Add,
    simd::{self, LaneCount, Mask, MaskElement, Simd, SimdElement, SupportedLaneCount},
};

pub use sleef;

// all units in meters
// all masses in kg

#[inline]
fn calculate_mean_motion<T: num_traits::Float>(
    combined_mass_div_gravitational_constant: T,
    semi_major_axis: T,
) -> T {
    (combined_mass_div_gravitational_constant
        / (semi_major_axis * semi_major_axis * semi_major_axis))
        .sqrt()
}

#[inline]
fn calculate_semi_minor_axis<T: num_traits::Float>(semi_major_axis: T, eccentricity: T) -> T {
    semi_major_axis * (T::one() - (eccentricity * eccentricity)).sqrt()
}

// https://en.wikipedia.org/wiki/Gravitational_constant
// in m^3 kg^-1 s^-2
const G: f64 = 6.6743015e-11;

#[derive(Clone, Copy)]
pub struct OrbitParamsInput<T> {
    parent_mass: T,
    mass: T,
    eccentricity: T,
    initial_mean_anomaly: T,
    semi_major_axis: T,
    argument_of_periapsis: T,
}

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

impl<T: num_traits::Float> OrbitParams<T> {
    #[inline]
    pub fn new(input: OrbitParamsInput<T>) -> Self {
        Self {
            eccentricity: input.eccentricity,
            initial_mean_anomaly: input.initial_mean_anomaly,
            mean_motion: calculate_mean_motion(
                (input.parent_mass + input.mass) * T::from(G).unwrap(),
                input.semi_major_axis,
            ),
            semi_major_axis: input.semi_major_axis,
            semi_minor_axis: calculate_semi_minor_axis(input.semi_major_axis, input.eccentricity),
            argument_of_periapsis_sin: input.argument_of_periapsis.sin(),
            argument_of_periapsis_cos: input.argument_of_periapsis.cos(),
            mass_div_gravitational_constant: input.mass * T::from(G).unwrap(),
        }
    }
}

impl<T: SimdElement, const N: usize> OrbitParams<Simd<T, N>>
where
    LaneCount<N>: SupportedLaneCount,
{
    #[inline]
    pub fn from_array(params: [OrbitParams<T>; N]) -> Self {
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

    #[inline]
    pub fn get(self, index: usize) -> OrbitParams<T> {
        OrbitParams {
            mean_motion: self.mean_motion[index],
            eccentricity: self.eccentricity[index],
            initial_mean_anomaly: self.initial_mean_anomaly[index],
            semi_major_axis: self.semi_major_axis[index],
            semi_minor_axis: self.semi_minor_axis[index],
            argument_of_periapsis_sin: self.argument_of_periapsis_sin[index],
            argument_of_periapsis_cos: self.argument_of_periapsis_cos[index],
            mass_div_gravitational_constant: self.mass_div_gravitational_constant[index],
        }
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct Vec3<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<const N: usize> Vec3<UniversalSimd<N>> {
    fn universal_splat(value: UniversalPos) -> Self {
        Self {
            x: UniversalSimd([value.x; N]),
            y: UniversalSimd([value.y; N]),
            z: UniversalSimd([value.z; N]),
        }
    }
}

impl<const N: usize> From<Vec3<Simd<f64, N>>> for Vec3<UniversalSimd<N>>
where
    LaneCount<N>: SupportedLaneCount,
{
    #[inline]
    fn from(value: Vec3<Simd<f64, N>>) -> Self {
        Self {
            x: value.x.into(),
            y: value.y.into(),
            z: value.z.into(),
        }
    }
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
    #[inline]
    pub fn get(self, index: usize) -> Vec3<T> {
        Vec3::new(self.x[index], self.y[index], self.z[index])
    }

    #[inline]
    pub fn splat(vec: Vec3<T>) -> Self {
        Self::new(Simd::splat(vec.x), Simd::splat(vec.y), Simd::splat(vec.z))
    }

    #[inline]
    pub fn as_array(self) -> [Vec3<T>; N] {
        std::array::from_fn(|i| self.get(i))
    }
}

impl<T: SimdElement> Vec3<Simd<T, 8>> {
    #[inline]
    fn swizzle_8(self, indices: [usize; 8]) -> Self {
        Self::new(
            swizzle_8(self.x, indices),
            swizzle_8(self.y, indices),
            swizzle_8(self.z, indices),
        )
    }
}

impl Vec3<UniversalSimd<8>> {
    #[inline]
    fn universal_swizzle_8(self, indices: [usize; 8]) -> Self {
        Self::new(
            universal_swizzle_8(self.x, indices),
            universal_swizzle_8(self.y, indices),
            universal_swizzle_8(self.z, indices),
        )
    }
}

impl<T: simd::num::SimdFloat> Vec3<T> {
    #[inline]
    fn reduce_sum(self) -> Vec3<T::Scalar> {
        Vec3::new(
            self.x.reduce_sum(),
            self.y.reduce_sum(),
            self.z.reduce_sum(),
        )
    }
}

impl<T: std::ops::Neg<Output = T>> std::ops::Neg for Vec3<T> {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self::Output {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
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

impl<T: std::ops::Add<Output = T> + Copy> std::ops::AddAssign for Vec3<T> {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl<T: std::ops::Sub> std::ops::Sub<Self> for Vec3<T> {
    type Output = Vec3<T::Output>;

    #[inline]
    fn sub(self, other: Self) -> Vec3<T::Output> {
        Vec3 {
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
pub fn get_acceleration_from_dir<T: FloatSimd>(dir: Vec3<T>, mu: T) -> Vec3<T::Scalar> {
    // Needs a square root as we're trying to get the normalized direction / distance^2.
    let mag_2 = dir.length_squared();
    let normalized_dir_div_dist_squared = dir / (mag_2 * mag_2.sqrt());
    let accel = normalized_dir_div_dist_squared * mu;
    accel.reduce_sum()
}

#[inline]
pub fn collides_at_distance<T: FloatSimd>(distance: Vec3<T>, body_radius_squared: T) -> bool {
    (distance.length_squared())
        .simd_le(body_radius_squared)
        .any()
}

#[inline]
pub fn collides<T: FloatSimd>(
    body_position: Vec3<T>,
    body_radius_squared: T,
    point: Vec3<T>,
) -> bool {
    collides_at_distance(body_position - point, body_radius_squared)
}

#[inline]
pub fn get_acceleration<T: FloatSimd>(
    body_position: Vec3<T>,
    mu: T,
    point: Vec3<T>,
) -> Vec3<T::Scalar> {
    get_acceleration_from_dir(body_position - point, mu)
}

#[inline]
pub fn get_universal_acceleration<const N: usize>(
    body_position: Vec3<UniversalSimd<N>>,
    mu: Simd<f64, N>,
    point: Vec3<UniversalSimd<N>>,
) -> Vec3<f64>
where
    LaneCount<N>: SupportedLaneCount,
    Simd<f64, N>: simd::num::SimdFloat<Scalar = f64> + simd::StdFloat,
{
    get_acceleration_from_dir(body_position - point, mu)
}

#[inline]
pub fn orbital_position<T: FloatSimd, SC: Fn(T) -> (T, T)>(
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

    Vec3::new(x, Default::default(), y)
}

pub trait SimdMask {
    fn any(self) -> bool;
}

impl<T: MaskElement, const N: usize> SimdMask for Mask<T, N>
where
    LaneCount<N>: SupportedLaneCount,
{
    fn any(self) -> bool {
        Mask::any(self)
    }
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
    + std::simd::cmp::SimdPartialEq<Mask = <Self as FloatSimd>::Mask>
    + std::simd::cmp::SimdPartialOrd
    + Default
{
    type Mask: SimdMask;

    fn one() -> Self;

    fn splat_f64(v: f64) -> Self;
}

impl<const N: usize> FloatSimd for Simd<f32, N>
where
    LaneCount<N>: SupportedLaneCount,
{
    type Mask = Mask<<f32 as SimdElement>::Mask, N>;

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
    type Mask = Mask<<f64 as SimdElement>::Mask, N>;

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
    eccentric_anomaly -= delta;
    let (eccentric_anomaly_sin, eccentric_anomaly_cos) = sincos(eccentric_anomaly);

    (
        eccentric_anomaly,
        eccentric_anomaly_sin,
        eccentric_anomaly_cos,
    )
}

#[inline]
fn join_universal_lanes(a: UniversalSimd<8>, b: UniversalSimd<8>) -> UniversalSimd<16> {
    let mut c = [Default::default(); 16];
    c[..8].copy_from_slice(&a[..]);
    c[8..].copy_from_slice(&b[..]);
    UniversalSimd(c)
}

#[inline]
fn join_universal_vec3s(
    a: Vec3<UniversalSimd<8>>,
    b: Vec3<UniversalSimd<8>>,
) -> Vec3<UniversalSimd<16>> {
    Vec3::new(
        join_universal_lanes(a.x, b.x),
        join_universal_lanes(a.y, b.y),
        join_universal_lanes(a.z, b.z),
    )
}

#[inline]
fn join_lanes(a: Simd<f64, 8>, b: Simd<f64, 8>) -> Simd<f64, 16> {
    let mut c = Simd::default();
    c.as_mut_array()[..8].copy_from_slice(a.as_array());
    c.as_mut_array()[8..].copy_from_slice(b.as_array());
    c
}

#[inline]
fn join_vec3s(a: Vec3<Simd<f64, 8>>, b: Vec3<Simd<f64, 8>>) -> Vec3<Simd<f64, 16>> {
    Vec3::new(
        join_lanes(a.x, b.x),
        join_lanes(a.y, b.y),
        join_lanes(a.z, b.z),
    )
}

#[derive(Clone, Copy)]
pub struct UniversalSimd<const N: usize>([UniversalScalar; N]);

impl<const N: usize> Default for UniversalSimd<N> {
    fn default() -> Self {
        Self([Default::default(); N])
    }
}

impl<const N: usize> std::ops::Add<Self> for UniversalSimd<N> {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        UniversalSimd(std::array::from_fn(|i| self[i] + rhs[i]))
    }
}

impl<const N: usize> std::ops::Sub<Self> for UniversalSimd<N>
where
    LaneCount<N>: SupportedLaneCount,
{
    type Output = Simd<f64, N>;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Simd::from_array(std::array::from_fn(|i| (self[i] - rhs[i]).cast()))
    }
}

impl<const N: usize> std::ops::Deref for UniversalSimd<N> {
    type Target = [UniversalScalar; N];
    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<const N: usize> From<Simd<f64, N>> for UniversalSimd<N>
where
    LaneCount<N>: SupportedLaneCount,
{
    #[inline]
    fn from(value: Simd<f64, N>) -> Self {
        Self(std::array::from_fn(|i| UniversalScalar::from_num(value[i])))
    }
}

#[derive(Clone, Copy)]
pub struct SystemState {
    pub star_mass_div_gravitational_constant: f64,
    pub planet_and_moon_positions: Vec3<Simd<f64, 16>>,
    pub planet_and_moon_gravitation: Simd<f64, 16>,
    pub star_radius_sq: Simd<f64, 1>,
    pub planet_and_moon_radius_sq: Simd<f64, 16>,
}

impl SystemState {
    #[inline]
    pub fn collides(&self, point: Vec3<f64>) -> bool {
        collides_at_distance::<Simd<f64, 1>>(Vec3::splat(point), self.star_radius_sq)
            || collides(
                self.planet_and_moon_positions,
                self.planet_and_moon_radius_sq,
                Vec3::splat(point),
            )
    }

    #[inline]
    pub fn acceleration_at(&self, point: Vec3<f64>) -> Vec3<f64> {
        get_acceleration::<Simd<f64, 1>>(
            Vec3::default(),
            Simd::splat_f64(self.star_mass_div_gravitational_constant),
            Vec3::splat(point),
        ) + get_acceleration(
            self.planet_and_moon_positions,
            self.planet_and_moon_gravitation,
            Vec3::splat(point),
        )
    }
}

#[derive(Clone, Copy)]
pub struct UniversalSystemState {
    star_mass_div_gravitational_constant: f64,
    planet_and_moon_positions: Vec3<UniversalSimd<16>>,
    planet_and_moon_gravitation: Simd<f64, 16>,
}

impl UniversalSystemState {
    #[inline]
    pub fn acceleration_at(&self, point: UniversalPos) -> Vec3<f64> {
        get_universal_acceleration::<1>(
            Vec3::default(),
            Simd::splat_f64(self.star_mass_div_gravitational_constant),
            Vec3::universal_splat(point),
        ) + get_universal_acceleration(
            self.planet_and_moon_positions,
            self.planet_and_moon_gravitation,
            Vec3::universal_splat(point),
        )
    }
}

#[derive(Clone, Copy)]
pub struct System {
    pub star_mass_div_gravitational_constant: f64,
    pub planets: OrbitParams<Simd<f64, 8>>,
    pub moons: OrbitParams<Simd<f64, 8>>,
    pub moon_parent_swizzles: [usize; 8],
    pub star_radius_sq: Simd<f64, 1>,
    pub planet_and_moon_radius_sq: Simd<f64, 16>,
}

impl System {
    pub const SOL_MASS_KG: f64 = 1.9885E30;
    pub const SOL_RADIUS: f64 = 6.957E8;

    pub const SOL_PLANETS: [(OrbitParamsInput<f64>, f64); 8] = [
        // Mercury
        (
            OrbitParamsInput {
                parent_mass: Self::SOL_MASS_KG,
                mass: 330109997604976300000000.0,
                eccentricity: 0.20563000440597534,
                semi_major_axis: 57_909_048_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            },
            2_439_700.0,
        ),
        // Venus
        (
            OrbitParamsInput {
                parent_mass: Self::SOL_MASS_KG,
                mass: 48674999312183700000000000.0,
                eccentricity: 0.006771999876946211,
                semi_major_axis: 108_208_000_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            },
            6_051_800.0,
        ),
        // Earth
        (
            OrbitParamsInput {
                parent_mass: Self::SOL_MASS_KG,
                mass: 5972167981508606000000000.0,
                eccentricity: 0.016708599403500557,
                initial_mean_anomaly: 0.0,
                semi_major_axis: 149_598_016_000.0,
                argument_of_periapsis: 0.0,
            },
            6_371_000.0,
        ),
        // Mars
        (
            OrbitParamsInput {
                parent_mass: Self::SOL_MASS_KG,
                mass: 641709984568677800000000.0,
                eccentricity: 0.0934000015258789,
                semi_major_axis: 227_939_360_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            },
            3_389_500.0,
        ),
        // Jupiter
        (
            OrbitParamsInput {
                parent_mass: Self::SOL_MASS_KG,
                mass: 1898200070271041200000000000.0,
                eccentricity: 0.048900000751018524,
                semi_major_axis: 778_000_000_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            },
            69_911_000.0,
        ),
        // Saturn
        (
            OrbitParamsInput {
                parent_mass: Self::SOL_MASS_KG,
                mass: 568340015946830600000000000.0,
                eccentricity: 0.05649999901652336,
                semi_major_axis: 1_433_529_984_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            },
            69_911_000.0,
        ),
        // Uranus
        (
            OrbitParamsInput {
                parent_mass: Self::SOL_MASS_KG,
                mass: 86809999452623640000000000.0,
                eccentricity: 0.0471699982881546,
                semi_major_axis: 2_870_971_904_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            },
            69_911_000.0,
        ),
        // Neptune
        (
            OrbitParamsInput {
                parent_mass: Self::SOL_MASS_KG,
                mass: 102413002683302160000000000.0,
                eccentricity: 0.008678000420331955,
                semi_major_axis: 4_499_999_744_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            },
            69_911_000.0,
        ),
    ];

    pub const SOL_MOONS: [(OrbitParamsInput<f64>, f64); 8] = [
        // Luna
        (
            OrbitParamsInput {
                parent_mass: Self::SOL_PLANETS[2].0.mass,
                mass: 73420000479201920000000.0,
                eccentricity: 0.05490000173449516,
                semi_major_axis: 384_399_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            },
            1_737_000.4,
        ),
        // Phobos
        (
            OrbitParamsInput {
                parent_mass: Self::SOL_PLANETS[3].0.mass,
                mass: 1.060E16,
                eccentricity: 0.0151,
                semi_major_axis: 9_376_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            },
            11_080.0,
        ),
        // Deimos
        (
            OrbitParamsInput {
                parent_mass: Self::SOL_PLANETS[3].0.mass,
                mass: 1.51E15,
                eccentricity: 0.00033,
                semi_major_axis: 23_463_200.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            },
            6_270.0,
        ),
        // Io
        (
            OrbitParamsInput {
                parent_mass: Self::SOL_PLANETS[4].0.mass,
                mass: 8.931938E22,
                eccentricity: 0.0040313019,
                semi_major_axis: 421_700_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            },
            1_821_600.0,
        ),
        // Europa
        (
            OrbitParamsInput {
                parent_mass: Self::SOL_PLANETS[4].0.mass,
                mass: 4.79984E22,
                eccentricity: 0.009,
                semi_major_axis: 670_900_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            },
            1_560_800.0,
        ),
        // Ganymede
        (
            OrbitParamsInput {
                parent_mass: Self::SOL_PLANETS[4].0.mass,
                mass: 1.4819E23,
                eccentricity: 0.0013,
                semi_major_axis: 1_070_400_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            },
            2_634_100.0,
        ),
        // Callisto
        (
            OrbitParamsInput {
                parent_mass: Self::SOL_PLANETS[4].0.mass,
                mass: 1.075938E23,
                eccentricity: 0.0074,
                semi_major_axis: 1_882_700_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            },
            2_410_300.0,
        ),
        // Titan
        (
            OrbitParamsInput {
                parent_mass: Self::SOL_PLANETS[5].0.mass,
                mass: 1.34518E23,
                eccentricity: 0.0288,
                semi_major_axis: 1_221_870_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            },
            2_574_730.0,
        ),
    ];

    #[inline]
    pub fn sol() -> Self {
        let planets =
            OrbitParams::from_array(Self::SOL_PLANETS.map(|(input, _)| OrbitParams::new(input)));

        let moons =
            OrbitParams::from_array(Self::SOL_MOONS.map(|(moon, _)| OrbitParams::new(moon)));

        Self {
            star_mass_div_gravitational_constant: Self::SOL_MASS_KG * G,
            planets,
            moons,
            moon_parent_swizzles: [2, 3, 3, 4, 4, 4, 4, 5],
            star_radius_sq: Simd::splat_f64(Self::SOL_RADIUS * Self::SOL_RADIUS),
            planet_and_moon_radius_sq: join_lanes(
                Simd::from_array(Self::SOL_PLANETS.map(|(_, radius)| radius * radius)),
                Simd::from_array(Self::SOL_MOONS.map(|(_, radius)| radius * radius)),
            ),
        }
    }

    #[inline]
    pub fn position_for_single_body(
        &self,
        index: usize,
        time: Simd<f64, 64>,
    ) -> Vec3<Simd<f64, 64>> {
        if index < 8 {
            let body = OrbitParams::from_array([self.planets.get(index); 64]);
            orbital_position::<Simd<f64, 64>, _>(body, time, sleef::f64x::sincos_u35)
        } else {
            let body = OrbitParams::from_array([self.moons.get(index - 8); 64]);

            let parent = self.moon_parent_swizzles[index - 8];

            orbital_position::<Simd<f64, 64>, _>(body, time, sleef::f64x::sincos_u35)
                + orbital_position::<Simd<f64, 64>, _>(
                    OrbitParams::from_array([self.planets.get(parent); 64]),
                    time,
                    sleef::f64x::sincos_u35,
                )
        }
    }

    #[inline]
    pub fn state_at(&self, time: f64) -> SystemState {
        let planet_positions =
            orbital_position(self.planets, Simd::splat(time), sleef::f64x::sincos_u35);

        let mut moon_positions =
            orbital_position(self.moons, Simd::splat(time), sleef::f64x::sincos_u35);

        moon_positions += planet_positions.swizzle_8(self.moon_parent_swizzles);

        SystemState {
            star_mass_div_gravitational_constant: self.star_mass_div_gravitational_constant,
            planet_and_moon_positions: join_vec3s(planet_positions, moon_positions),
            planet_and_moon_gravitation: join_lanes(
                self.planets.mass_div_gravitational_constant,
                self.moons.mass_div_gravitational_constant,
            ),
            star_radius_sq: self.star_radius_sq,
            planet_and_moon_radius_sq: self.planet_and_moon_radius_sq,
        }
    }

    #[inline]
    pub fn universal_state_at(&self, time: f64) -> UniversalSystemState {
        let planet_positions = Vec3::<UniversalSimd<8>>::from(orbital_position(
            self.planets,
            Simd::splat(time),
            sleef::f64x::sincos_u35,
        ));

        let mut moon_positions = Vec3::<UniversalSimd<8>>::from(orbital_position(
            self.moons,
            Simd::splat(time),
            sleef::f64x::sincos_u35,
        ));

        moon_positions += planet_positions.universal_swizzle_8(self.moon_parent_swizzles);

        UniversalSystemState {
            star_mass_div_gravitational_constant: self.star_mass_div_gravitational_constant,
            planet_and_moon_positions: join_universal_vec3s(planet_positions, moon_positions),
            planet_and_moon_gravitation: join_lanes(
                self.planets.mass_div_gravitational_constant,
                self.moons.mass_div_gravitational_constant,
            ),
        }
    }
}

/*fn erydanos_sin_cos(v: Simd<f64, 8>) -> (Simd<f64, 8>, Simd<f64, 8>) {
    use erydanos::{_mm256_cos_pd, _mm256_sin_pd};
    use std::arch::x86_64::__m256d;

    let arr = v.to_array();

    unsafe {
        let low = std::mem::transmute::<[f64; 4], __m256d>([arr[0], arr[1], arr[2], arr[3]]);
        let high = std::mem::transmute::<[f64; 4], __m256d>([arr[4], arr[5], arr[6], arr[7]]);

        let sin_low = _mm256_sin_pd(low);
        let sin_high = _mm256_sin_pd(high);

        let cos_low = _mm256_cos_pd(low);
        let cos_high = _mm256_cos_pd(high);

        (
            Simd::from_array(std::mem::transmute([sin_low, sin_high])),
            Simd::from_array(std::mem::transmute([cos_low, cos_high])),
        )
    }
}*/

#[inline]
fn swizzle_8<T: SimdElement>(a: Simd<T, 8>, indices: [usize; 8]) -> Simd<T, 8> {
    Simd::from_array(std::array::from_fn(|i| a[indices[i]]))
}

#[inline]
fn universal_swizzle_8(a: UniversalSimd<8>, indices: [usize; 8]) -> UniversalSimd<8> {
    UniversalSimd(std::array::from_fn(|i| a[indices[i]]))
}

pub type UniversalScalar = fixed::types::I108F20;

#[derive(Clone, Copy, Default, Debug)]
pub struct UniversalPos {
    pub x: UniversalScalar,
    pub y: UniversalScalar,
    pub z: UniversalScalar,
}

use az::Cast;

use std::ops::{AddAssign, Sub};

impl From<Vec3<f64>> for UniversalPos {
    #[inline]
    fn from(vec: Vec3<f64>) -> Self {
        Self::new_3(vec.x, vec.y, vec.z)
    }
}

impl UniversalPos {
    #[inline]
    pub fn splat(v: UniversalScalar) -> Self {
        Self::new(v, v, v)
    }

    #[inline]
    pub fn min(self, other: Self) -> Self {
        Self::new(
            self.x.min(other.x),
            self.y.min(other.y),
            self.z.min(other.z),
        )
    }

    #[inline]
    pub fn max(self, other: Self) -> Self {
        Self::new(
            self.x.max(other.x),
            self.y.max(other.y),
            self.z.max(other.z),
        )
    }

    #[inline]
    pub fn new_3(x: f64, y: f64, z: f64) -> Self {
        Self::new(
            UniversalScalar::from_num(x),
            UniversalScalar::from_num(y),
            UniversalScalar::from_num(z),
        )
    }

    #[inline]
    pub fn new(x: UniversalScalar, y: UniversalScalar, z: UniversalScalar) -> Self {
        Self { x, y, z }
    }

    #[inline]
    pub fn distance_squared(self, other: Self) -> f64 {
        (self - other).length_squared()
    }

    #[inline]
    pub fn distance(self, other: Self) -> f64 {
        self.distance_squared(other).sqrt()
    }

    #[inline]
    pub fn relative_to(self, other: Self) -> Self {
        Self {
            x: (self.x - other.x),
            y: (self.y - other.y),
            z: (self.z - other.z),
        }
    }

    #[inline]
    pub fn as_vec3(self) -> Vec3<f64> {
        Vec3::new(self.x.cast(), self.y.cast(), self.z.cast())
    }
}

impl Add<Vec3<f64>> for UniversalPos {
    type Output = Self;

    #[inline]
    fn add(self, other: Vec3<f64>) -> Self {
        Self {
            x: self.x + UniversalScalar::from_num(other.x),
            y: self.y + UniversalScalar::from_num(other.y),
            z: self.z + UniversalScalar::from_num(other.z),
        }
    }
}

impl AddAssign<Vec3<f64>> for UniversalPos {
    #[inline]
    fn add_assign(&mut self, other: Vec3<f64>) {
        *self = *self + other;
    }
}

impl Sub<UniversalPos> for UniversalPos {
    type Output = Vec3<f64>;

    #[inline]
    fn sub(self, other: Self) -> Vec3<f64> {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
        .as_vec3()
    }
}
