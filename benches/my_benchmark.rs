#![feature(portable_simd)]

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};

use nbody::*;
use std::simd::{LaneCount, Simd, SimdElement, StdFloat, SupportedLaneCount};

use criterion::BenchmarkId;

pub fn bench_group<T, const N: usize, SC>(
    c: &mut Criterion,
    sincos: SC,
    mean_anomaly: T,
    eccentricity: T,
) where
    T: SimdElement,
    LaneCount<N>: SupportedLaneCount,
    Simd<T, N>: FloatSimd,
    SC: Fn(Simd<T, N>) -> (Simd<T, N>, Simd<T, N>) + Copy,
{
    let mut group = c.benchmark_group(format!(
        "{}/{}/{}",
        std::any::type_name::<T>(),
        N,
        std::any::type_name::<SC>(),
    ));
    group.throughput(Throughput::Elements(N as _));

    group.bench_function("newton_raphson_kepler_simd", |b| {
        b.iter(|| {
            newton_raphson_kepler_simd::<Simd<T, N>, _>(
                black_box(Simd::splat(mean_anomaly)),
                black_box(Simd::splat(eccentricity)),
                sincos,
            )
        })
    });

    group.finish();
}

pub fn sin_cos_sequential<T: FloatSimd>(v: T) -> (T, T) {
    (v.sin(), v.cos())
}

pub fn criterion_benchmark(c: &mut Criterion) {
    let mut group = c.benchmark_group("f64/1");
    group.throughput(Throughput::Elements(1));
    //group.bench_function("newton_raphson_kepler_scalar", |b| {
    //    b.iter(|| newton_raphson_kepler(black_box(0.1), black_box(0.2)))
    //});
    group.finish();

    bench_group::<f32, 01, _>(c, sleef::f32x::sincos_u35, 0.1, 0.2);
    bench_group::<f32, 02, _>(c, sleef::f32x::sincos_u35, 0.1, 0.2);
    bench_group::<f32, 04, _>(c, sleef::f32x::sincos_u35, 0.1, 0.2);
    bench_group::<f32, 08, _>(c, sleef::f32x::sincos_u35, 0.1, 0.2);
    bench_group::<f32, 16, _>(c, sleef::f32x::sincos_u35, 0.1, 0.2);
    bench_group::<f32, 32, _>(c, sleef::f32x::sincos_u35, 0.1, 0.2);
    bench_group::<f32, 64, _>(c, sleef::f32x::sincos_u35, 0.1, 0.2);

    bench_group::<f32, 01, _>(c, sin_cos_sequential, 0.1, 0.2);
    bench_group::<f32, 02, _>(c, sin_cos_sequential, 0.1, 0.2);
    bench_group::<f32, 04, _>(c, sin_cos_sequential, 0.1, 0.2);
    bench_group::<f32, 08, _>(c, sin_cos_sequential, 0.1, 0.2);
    bench_group::<f32, 16, _>(c, sin_cos_sequential, 0.1, 0.2);
    bench_group::<f32, 32, _>(c, sin_cos_sequential, 0.1, 0.2);
    bench_group::<f32, 64, _>(c, sin_cos_sequential, 0.1, 0.2);

    bench_group::<f64, 01, _>(c, sleef::f64x::sincos_u35, 0.1, 0.2);
    bench_group::<f64, 02, _>(c, sleef::f64x::sincos_u35, 0.1, 0.2);
    bench_group::<f64, 04, _>(c, sleef::f64x::sincos_u35, 0.1, 0.2);
    bench_group::<f64, 08, _>(c, sleef::f64x::sincos_u35, 0.1, 0.2);
    bench_group::<f64, 16, _>(c, sleef::f64x::sincos_u35, 0.1, 0.2);
    bench_group::<f64, 32, _>(c, sleef::f64x::sincos_u35, 0.1, 0.2);
    bench_group::<f64, 64, _>(c, sleef::f64x::sincos_u35, 0.1, 0.2);

    bench_group::<f64, 01, _>(c, sin_cos_sequential, 0.1, 0.2);
    bench_group::<f64, 02, _>(c, sin_cos_sequential, 0.1, 0.2);
    bench_group::<f64, 04, _>(c, sin_cos_sequential, 0.1, 0.2);
    bench_group::<f64, 08, _>(c, sin_cos_sequential, 0.1, 0.2);
    bench_group::<f64, 16, _>(c, sin_cos_sequential, 0.1, 0.2);
    bench_group::<f64, 32, _>(c, sin_cos_sequential, 0.1, 0.2);
    bench_group::<f64, 64, _>(c, sin_cos_sequential, 0.1, 0.2);

    let system = System::sol();

    c.bench_function("1_000_000", |b| {
        b.iter(|| {
            let mut position = black_box(Vec3::new(100000.0, 100000.0, 100000.0));
            let mut velocity = black_box(Vec3::new(1000.0, 0.0, 0.0));
            let initial_time = black_box(36666.3);

            for i in 0..1_000_000 {
                let state = system.state_at(initial_time + i as f64);
                velocity += state.acceleration_at(position);
                position += velocity;
            }
        })
    });

    c.bench_function("1_000_000_128-bit", |b| {
        b.iter(|| {
            let mut position = black_box(UniversalPos::new_3(100000.0, 100000.0, 100000.0));
            let mut velocity = black_box(Vec3::new(1000.0, 0.0, 0.0));
            let initial_time = black_box(36666.3);
            let star_position = black_box(UniversalPos::default());

            for i in 0..1_000_000 {
                let state = system.state_at(initial_time + i as f64);
                velocity += state.acceleration_at(position - star_position);
                position += velocity;
            }
        })
    });

    c.bench_function("1_000_000_univ", |b| {
        b.iter(|| {
            let mut position = black_box(UniversalPos::new_3(100000.0, 100000.0, 100000.0));
            let mut velocity = black_box(Vec3::new(1000.0, 0.0, 0.0));
            let initial_time = black_box(36666.3);
            let star_position = black_box(UniversalPos::default());

            for i in 0..1_000_000 {
                let state = system.universal_state_at(initial_time + i as f64);
                velocity += state.acceleration_at(position.relative_to(star_position));
                position += velocity;
            }
        })
    });

    /*c.bench_function("1_day_64", |b| {
        b.iter(|| {
            let mut position = black_box(Vec3::new(100000.0, 100000.0, 100000.0));
            let mut velocity = black_box(Vec3::new(1000.0, 0.0, 0.0));
            let mut initial_time = black_box(36666.3);

            for i in 0..(60 * 60 * 24) / 64 {
                calculate_acceleration_for_system_64(
                    system,
                    &mut position,
                    &mut velocity,
                    initial_time + (i * 64) as f64,
                );
            }
        })
    });*/

    /*c.bench_function("abc", |b| {
        b.iter(|| {
            orbital_position_simd::<Simd<f64, 16>, _>(
                OrbitParams {
                    mean_motion: black_box(Simd::splat(0.1)),
                    eccentricity: black_box(Simd::splat(0.2)),
                    initial_mean_anomaly: black_box(Simd::splat(0.1)),
                    semi_major_axis: black_box(Simd::splat(0.2)),
                    semi_minor_axis: black_box(Simd::splat(0.2)),
                    argument_of_periapsis_sin: black_box(Simd::splat(0.1)),
                    argument_of_periapsis_cos: black_box(Simd::splat(0.2)),
                },
                black_box(Simd::splat(1023.666)),
                sleef::f64x::sincos_u35,
            )
        });
    });

    c.bench_function("get_acceleration_simd_f64x8", |b| {
        b.iter(|| {
            let position = orbital_position_simd::<Simd<f64, 16>, _>(
                OrbitParams {
                    mean_motion: black_box(Simd::splat(0.1)),
                    eccentricity: black_box(Simd::splat(0.2)),
                    initial_mean_anomaly: black_box(Simd::splat(0.1)),
                    semi_major_axis: black_box(Simd::splat(0.2)),
                    semi_minor_axis: black_box(Simd::splat(0.2)),
                    argument_of_periapsis_sin: black_box(Simd::splat(0.1)),
                    argument_of_periapsis_cos: black_box(Simd::splat(0.2)),
                },
                black_box(Simd::splat(1023.666)),
                sleef::f64x::sincos_u35,
            );

            get_acceleration(
                position,
                black_box(Simd::splat(0.2)),
                black_box(Vec3::new(
                    Simd::splat(1.0),
                    Simd::splat(2.0),
                    Simd::splat(3.0),
                )),
            )
        });
    });*/
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
