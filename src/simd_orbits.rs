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

    #[inline]
    fn get(self, i: usize) -> OrbitParams<T> {
        OrbitParams {
            eccentricity: self.eccentricity[i],
            initial_mean_anomaly: self.initial_mean_anomaly[i],
            mean_motion: self.mean_motion[i],
            semi_major_axis: self.semi_major_axis[i],
            semi_minor_axis: self.semi_minor_axis[i],
            argument_of_periapsis_sin: self.argument_of_periapsis_sin[i],
            argument_of_periapsis_cos: self.argument_of_periapsis_cos[i],
            mass_div_gravitational_constant: self.mass_div_gravitational_constant[i],
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

impl<T: SimdElement> Vec3<Simd<T, 8>> {
    fn swizzle_8(self, indices: [usize; 8]) -> Self {
        Self::new(
            swizzle_8(self.x, indices),
            swizzle_8(self.y, indices),
            swizzle_8(self.z, indices),
        )
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

impl<T: std::ops::Add<Output = T> + Copy> std::ops::AddAssign for Vec3<T> {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        *self = (*self + rhs);
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
pub fn get_acceleration<T: FloatSimd>(
    body_position: Vec3<T>,
    mu: T,
    point: Vec3<T>,
) -> Vec3<T::Scalar> {
    let dir = body_position - point;
    let mag_2 = dir.length_squared();
    let force = dir / (mag_2 * mag_2.sqrt());
    let accel = force * mu;
    accel.reduce_sum()
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

#[inline]
fn join_params(
    a: OrbitParams<Simd<f64, 8>>,
    b: OrbitParams<Simd<f64, 8>>,
) -> OrbitParams<Simd<f64, 16>> {
    OrbitParams {
        mean_motion: join_lanes(a.mean_motion, b.mean_motion),
        eccentricity: join_lanes(a.eccentricity, b.eccentricity),
        initial_mean_anomaly: join_lanes(a.initial_mean_anomaly, b.initial_mean_anomaly),
        argument_of_periapsis_sin: join_lanes(
            a.argument_of_periapsis_sin,
            b.argument_of_periapsis_sin,
        ),
        argument_of_periapsis_cos: join_lanes(
            a.argument_of_periapsis_cos,
            b.argument_of_periapsis_cos,
        ),
        semi_minor_axis: join_lanes(a.semi_minor_axis, b.semi_minor_axis),
        semi_major_axis: join_lanes(a.semi_major_axis, b.semi_major_axis),
        mass_div_gravitational_constant: join_lanes(
            a.mass_div_gravitational_constant,
            b.mass_div_gravitational_constant,
        ),
    }
}

#[derive(Clone, Copy)]
pub struct SystemState {
    star_mass_div_gravitational_constant: f64,
    planet_and_moon_positions: Vec3<Simd<f64, 16>>,
    planet_and_moon_gravitation: Simd<f64, 16>,
}

impl SystemState {
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
pub struct System {
    star_mass_div_gravitational_constant: f64,
    planets: OrbitParams<Simd<f64, 8>>,
    moons: OrbitParams<Simd<f64, 8>>,
    moon_parent_swizzles: [usize; 8],
}

impl System {
    pub fn new() -> Self {
        let sol_mass_kg = 1.9885E30;
        let earth_mass = 5972167981508606000000000.0;
        let jupiter_mass = 1898200070271041200000000000.0;

        let planets = OrbitParams::from_array([
            // Mercury
            OrbitParams::new(OrbitParamsInput {
                parent_mass: sol_mass_kg,
                mass: 330109997604976300000000.0,
                eccentricity: 0.20563000440597534,
                semi_major_axis: 57909048_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            }),
            // Venus
            OrbitParams::new(OrbitParamsInput {
                parent_mass: sol_mass_kg,
                mass: 48674999312183700000000000.0,
                eccentricity: 0.006771999876946211,
                semi_major_axis: 108208000_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            }),
            // Earth
            OrbitParams::new(OrbitParamsInput {
                parent_mass: sol_mass_kg,
                mass: earth_mass,
                eccentricity: 0.016708599403500557,
                initial_mean_anomaly: 0.0,
                semi_major_axis: 149598016_000.0,
                argument_of_periapsis: 0.0,
            }),
            // Mars
            OrbitParams::new(OrbitParamsInput {
                parent_mass: sol_mass_kg,
                mass: 641709984568677800000000.0,
                eccentricity: 0.0934000015258789,
                semi_major_axis: 227939360_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            }),
            // Jupiter
            OrbitParams::new(OrbitParamsInput {
                parent_mass: sol_mass_kg,
                mass: jupiter_mass,
                eccentricity: 0.048900000751018524,
                semi_major_axis: 778000000_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            }),
            // Saturn
            OrbitParams::new(OrbitParamsInput {
                parent_mass: sol_mass_kg,
                mass: 568340015946830600000000000.0,
                eccentricity: 0.05649999901652336,
                semi_major_axis: 1433529984_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            }),
            // Uranus
            OrbitParams::new(OrbitParamsInput {
                parent_mass: sol_mass_kg,
                mass: 86809999452623640000000000.0,
                eccentricity: 0.0471699982881546,
                semi_major_axis: 2870971904_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            }),
            // Neptune
            OrbitParams::new(OrbitParamsInput {
                parent_mass: sol_mass_kg,
                mass: 102413002683302160000000000.0,
                eccentricity: 0.008678000420331955,
                semi_major_axis: 4499999744_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            }),
        ]);

        let moons = OrbitParams::from_array([
            // Luna
            OrbitParams::new(OrbitParamsInput {
                parent_mass: earth_mass,
                mass: 73420000479201920000000.0,
                eccentricity: 0.05490000173449516,
                semi_major_axis: 384399_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            }),
            // Luna
            OrbitParams::new(OrbitParamsInput {
                parent_mass: earth_mass,
                mass: 73420000479201920000000.0,
                eccentricity: 0.05490000173449516,
                semi_major_axis: 384399_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            }),
            // Luna
            OrbitParams::new(OrbitParamsInput {
                parent_mass: earth_mass,
                mass: 73420000479201920000000.0,
                eccentricity: 0.05490000173449516,
                semi_major_axis: 384399_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            }),
            // Callisto
            OrbitParams::new(OrbitParamsInput {
                parent_mass: jupiter_mass,
                mass: 107593796336849370000000.0,
                eccentricity: 0.007400000002235174,
                semi_major_axis: 1882700_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            }),
            // Luna
            OrbitParams::new(OrbitParamsInput {
                parent_mass: earth_mass,
                mass: 73420000479201920000000.0,
                eccentricity: 0.05490000173449516,
                semi_major_axis: 384399_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            }),
            // Luna
            OrbitParams::new(OrbitParamsInput {
                parent_mass: earth_mass,
                mass: 73420000479201920000000.0,
                eccentricity: 0.05490000173449516,
                semi_major_axis: 384399_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            }),
            // Luna
            OrbitParams::new(OrbitParamsInput {
                parent_mass: earth_mass,
                mass: 73420000479201920000000.0,
                eccentricity: 0.05490000173449516,
                semi_major_axis: 384399_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            }),
            // Luna
            OrbitParams::new(OrbitParamsInput {
                parent_mass: earth_mass,
                mass: 73420000479201920000000.0,
                eccentricity: 0.05490000173449516,
                semi_major_axis: 384399_000.0,
                initial_mean_anomaly: 0.0,
                argument_of_periapsis: 0.0,
            }),
        ]);

        Self {
            star_mass_div_gravitational_constant: sol_mass_kg * G,
            planets,
            moons,
            moon_parent_swizzles: [2, 2, 2, 4, 2, 2, 2, 2],
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
            planet_and_moon_gravitation: join_lanes(self.planets.mass_div_gravitational_constant, self.moons.mass_div_gravitational_constant)
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
    let array = a.as_array();

    Simd::from_array([
        a[indices[0]],
        a[indices[1]],
        a[indices[2]],
        a[indices[3]],
        a[indices[4]],
        a[indices[5]],
        a[indices[6]],
        a[indices[7]],
    ])
}

pub fn calculate_acceleration_for_system(system: System, point: Vec3<f64>, time: f64) -> Vec3<f64> {
    let sun_accel = get_acceleration::<Simd<f64, 1>>(
        Vec3::default(),
        Simd::splat_f64(system.star_mass_div_gravitational_constant),
        Vec3::splat(point),
    );

    let planet_positions =
        orbital_position(system.planets, Simd::splat(time), sleef::f64x::sincos_u35);

    let mut moon_positions =
        orbital_position(system.moons, Simd::splat(time), sleef::f64x::sincos_u35);

    moon_positions += planet_positions.swizzle_8(system.moon_parent_swizzles);

    //let combined_positions = Vec3::new(
    //    join_lanes(planet_positions.x, moon_positions.x),
    //    join_lanes(planet_positions.y, moon_positions.y),
    //    join_lanes(planet_positions.z, moon_positions.z),
    //);

    //let combined_gravities = join_lanes(
    //    system.planets.mass_div_gravitational_constant,
    //    system.moons.mass_div_gravitational_constant,
    //);

    sun_accel
    //+ get_acceleration(
    //    combined_positions,
    //    combined_gravities,
    //    Vec3::splat(point),
    //).reduce_sum()
    + get_acceleration(
        planet_positions,
        system.planets.mass_div_gravitational_constant,
        Vec3::splat(point),
    )

    + get_acceleration(
        moon_positions,
        system.moons.mass_div_gravitational_constant,
        Vec3::splat(point),
    )
}

pub fn calculate_acceleration_for_system_64(
    system: System,
    point: &mut Vec3<f64>,
    vel: &mut Vec3<f64>,
    initial_time: f64,
) {
    let times = Simd::splat(initial_time) + Simd::from_array(std::array::from_fn(|i| i as f64));

    let planet_pos = std::array::from_fn::<_, 8, _>(|i| {
        let planet = system.planets.get(i);
        orbital_position(
            OrbitParams::from_array([planet; 64]),
            times,
            sleef::f64x::sincos_u35,
        )
    });

    let moon_pos = std::array::from_fn::<_, 8, _>(|i| {
        let moon = system.moons.get(i);
        orbital_position(
            OrbitParams::from_array([moon; 64]),
            times,
            sleef::f64x::sincos_u35,
        ) + planet_pos[system.moon_parent_swizzles[i]]
    });

    for i in 0..64 {
        let sun_accel = get_acceleration::<Simd<f64, 1>>(
            Vec3::default(),
            Simd::splat_f64(system.star_mass_div_gravitational_constant),
            Vec3::splat(*point),
        );

        let accel = sun_accel
            + get_acceleration(
                Vec3::new(
                    Simd::from_array(std::array::from_fn(|p_i| planet_pos[p_i].x.as_array()[i])),
                    Simd::from_array(std::array::from_fn(|p_i| planet_pos[p_i].y.as_array()[i])),
                    Simd::from_array(std::array::from_fn(|p_i| planet_pos[p_i].z.as_array()[i])),
                ),
                system.planets.mass_div_gravitational_constant,
                Vec3::splat(*point),
            )
            + get_acceleration(
                Vec3::new(
                    Simd::from_array(std::array::from_fn(|p_i| moon_pos[p_i].x.as_array()[i])),
                    Simd::from_array(std::array::from_fn(|p_i| moon_pos[p_i].y.as_array()[i])),
                    Simd::from_array(std::array::from_fn(|p_i| moon_pos[p_i].z.as_array()[i])),
                ),
                system.moons.mass_div_gravitational_constant,
                Vec3::splat(*point),
            );

        *vel += accel;
        *point += *vel;
    }
}

#[test]
fn testing() {
    let accel = calculate_acceleration_for_system(
        System::new(),
        Vec3::new(0.0, 0.0, 149598016_000.0),
        100.0,
    );

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
