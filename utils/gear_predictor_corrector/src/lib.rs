use num_traits::Float;
use std::{ops::{Add, Div, Mul, Sub}, fmt::Debug};

const fn fac(n: u64) -> u64 {
    match n {
        0u64 | 1u64 => 1,
        2u64..=20u64 => fac(n - 1u64) * n,
        _ => 0,
    }
}

fn fac_f<F: Float>(n: u64) -> F {
    F::from(fac(n)).unwrap()
}

pub struct GearPredictor<T> {
    pub rs: [T; 6],
}

pub struct GearCorrector<T> {
    pub predictions: [T; 6],
}

impl<T: Copy + Sub<T, Output = T> + Add<T, Output = T>> GearPredictor<T> {
    pub fn predict<F: Float + Debug>(self, dt: F) -> GearCorrector<T>
    where
        T: Div<F, Output = T> + Mul<F, Output = T> + Debug,
    {
        let [r, r1, r2, r3, r4, r5] = self.rs;

        GearCorrector {
            predictions: [
                r + r1 * dt
                    + r2 * dt.powi(2) / fac_f(2)
                    + r3 * dt.powi(3) / fac_f(3)
                    + r4 * dt.powi(4) / fac_f(4)
                    + r5 * dt.powi(5) / fac_f(5),
                r1 + r2 * dt
                    + r3 * dt.powi(2) / fac_f(2)
                    + r4 * dt.powi(3) / fac_f(3)
                    + r5 * dt.powi(4) / fac_f(4),
                r2 + r3 * dt + r4 * dt.powi(2) / fac_f(2) + r5 * dt.powi(3) / fac_f(3),
                r3 + r4 * dt + r5 * dt.powi(2) / fac_f(2),
                r4 + r5 * dt,
                r5,
            ],
        }
    }
}

impl<T: Copy + Sub<T, Output = T> + Add<T, Output = T> + Debug> GearCorrector<T> {
    pub fn correct<F: Float + Debug>(&self, r2: T, dt: F) -> [T; 6]
    where
        T: Div<F, Output = T> + Mul<F, Output = T>,
        F: Mul<T, Output = T>,
    {
        let [rp, r1p, r2p, r3p, r4p, r5p] = self.predictions;

        // Calculate the deltas
        let da = r2 - r2p;
        let dr2 = da * dt.powi(2) / fac_f(2); // delta r2

        // Calculate the corrections
        [
            rp + F::from(3.0 / 16.0).unwrap() * dr2,
            r1p + F::from(251.0 / 360.0).unwrap() * dr2 / dt,
            r2p + F::from(1.0).unwrap() * dr2 * fac_f(2) / dt.powi(2),
            r3p + F::from(11.0 / 18.0).unwrap() * dr2 * fac_f(3) / dt.powi(3),
            r4p + F::from(1.0 / 6.0).unwrap() * dr2 * fac_f(4) / dt.powi(4),
            r5p + F::from(1.0 / 60.0).unwrap() * dr2 * fac_f(5) / dt.powi(5),
        ]
    }
}
