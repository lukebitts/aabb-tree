#![deny(missing_docs)]

//! A mod that defines some common AABB operations, so any AABB type can be used with
//! the AABB Tree.

use num::{Num, Zero, One};
//use std::ops::{Add, Sub, Mul};

/*fn vec3_add<T: Add<Output=T>>(v1: (T, T, T), v2: (T, T, T)) -> (T, T, T) {
	(v1.0 + v2.0, v1.1 + v2.1, v1.2 + v2.2)
}

fn vec3_sub<T: Sub<Output=T>>(v1: (T, T, T), v2: (T, T, T)) -> (T, T, T) {
	(v1.0 - v2.0, v1.1 - v2.1, v1.2 - v2.2)
}

fn vec3_num_mul<T: Copy + Mul<Output=T>>(v: (T, T, T), n: T) -> (T, T, T) {
	(v.0 * n, v.1 * n, v.2 * n)
}*/

/// A trait that AABB types should implement to be used within the `AabbTree`. Has some
/// convenience methods already defined.
pub trait Aabb: ::std::fmt::Debug {
    /// Defines the precision of the AABB (and consequently of the `AabbTree`). While
    /// any numerical type can be used, the tree might behave weirdly when given non
    /// float types.
    type Precision: Zero + One + Num + PartialOrd + Copy;

    /// Returns a new AABB from two tuples.
    fn with_params(
        (Self::Precision, Self::Precision, Self::Precision),
        (Self::Precision, Self::Precision, Self::Precision),
    ) -> Self
    where
        Self: Sized;
    /// Returns the min value of the AABB.
    fn min(&self) -> (Self::Precision, Self::Precision, Self::Precision);
    /// Returns the max value of the AABB.
    fn max(&self) -> (Self::Precision, Self::Precision, Self::Precision);

    /// Returns `true` if `other` is fully contained in `self`.
    ///
    /// # Panics
    ///
    /// Panics in debug mode unless both AABBs are valid.
    fn contains<T: Aabb<Precision = Self::Precision>>(&self, other: &T) -> bool {
        //debug_assert!(self.is_valid() && other.is_valid(), "AABB is not valid");
        debug_assert!(self.is_valid() && other.is_valid(), "({:?}, {:?}) AABB is not valid", self, other);

        //self.is_valid() && other.is_valid() &&
        self.min().0 <= other.min().0 && self.min().1 <= other.min().1 &&
            self.min().2 <= other.min().2 && other.max().0 <= self.max().0 &&
            other.max().1 <= self.max().1 && other.max().2 <= self.max().2
    }

    /// Returns the perimeter of the AABB.
    ///
    /// # Panics
    ///
    /// Panics in debug mode if the AABB is not valid.
    fn perimeter(&self) -> Self::Precision {
        //debug_assert!(self.is_valid(), "AABB is not valid");
        debug_assert!(self.is_valid(), "({:?}) AABB is not valid", self);

        let wx = self.max().0 - self.min().0;
        let wy = self.max().1 - self.min().1;
        let wz = self.max().2 - self.min().2;

        let four = Self::Precision::one() + Self::Precision::one() + Self::Precision::one() +
            Self::Precision::one();
        four * (wx + wy + wz)
    }

    /// Returns `true` if the two AABBs overlap.
    ///
    /// # Panics
    ///
    /// Panics in debug mode unless both AABBs are valid.
    fn overlaps<T: Aabb<Precision = Self::Precision>>(&self, other: &T) -> bool {
        debug_assert!(self.is_valid() && other.is_valid(), "({:?}, {:?}) AABB is not valid", self, other);

        //(self.is_valid() && other.is_valid()) &&
        !((other.min().0 > self.max().0 || other.max().0 < self.min().0) ||
              (other.min().1 > self.max().1 || other.max().1 < self.min().1) ||
              (other.min().2 > self.max().2 || other.max().2 < self.min().2))
    }

    /// Returns the center of the AABB, which is defined as `(min + (diagonal / 2))`.
    ///
    /// # Panics
    ///
    /// Panics in debug mode if the AABB is not valid.
    /*fn center(&self) -> (Self::Precision, Self::Precision, Self::Precision) {
		debug_assert!(self.is_valid(), "AABB is not valid");

		let d = self.diagonal();
		let half = Self::Precision::one() / (Self::Precision::one() + Self::Precision::one());
		vec3_add(self.min(), vec3_num_mul(d, half))
	}*/
    /// Returns the diagonal vector of the AABB which is defined as `(max - min)`.
    ///
    /// # Panics
    ///
    /// Panics in debug mode if the AABB is not valid.
    /*fn diagonal(&self) -> (Self::Precision, Self::Precision, Self::Precision) {
		debug_assert!(self.is_valid(), "AABB is not valid");

		vec3_sub(self.max(), self.min())
	}*/
    /// Tests wether an AABB is valid or not. AABBs are valid if their min value is smaller
    /// than their max value.
    fn is_valid(&self) -> bool {
        self.min().0 < self.max().0 && self.min().1 < self.max().1 && self.min().2 < self.max().2
    }
}

/*impl Aabb for ((f32, f32, f32), (f32, f32, f32)) {
	type Precision = f32;
	fn with_params(min: (f32, f32, f32), max: (f32, f32, f32)) -> Self {
		(min, max)
	}
	fn min(&self) -> (f32, f32, f32) {
		self.0
	}
	fn max(&self) -> (f32, f32, f32) {
		self.1
	}
}

impl Aabb for [[f32; 3]; 2] {
	type Precision = f32;
	fn with_params(min: (f32, f32, f32), max: (f32, f32, f32 )) -> Self {
		[[min.0, min.1, min.2], [max.0, max.1, max.2]]
	}
	fn min(&self) -> (f32, f32, f32) {
		(self[0][0], self[0][1], self[0][2])
	}
	fn max(&self) -> (f32, f32, f32) {
		(self[1][0], self[1][1], self[1][2])
	}
}

impl Aabb for ((f64, f64, f64), (f64, f64, f64)) {
	type Precision = f64;
	fn with_params(min: (f64, f64, f64), max: (f64, f64, f64)) -> Self {
		(min, max)
	}
	fn min(&self) -> (f64, f64, f64) {
		self.0
	}
	fn max(&self) -> (f64, f64, f64) {
		self.1
	}
}

impl Aabb for [[f64; 3]; 2] {
	type Precision = f64;
	fn with_params(min: (f64, f64, f64), max: (f64, f64, f64)) -> Self {
		[[min.0, min.1, min.2], [max.0, max.1, max.2]]
	}
	fn min(&self) -> (f64, f64, f64) {
		(self[0][0], self[0][1], self[0][2])
	}
	fn max(&self) -> (f64, f64, f64) {
		(self[1][0], self[1][1], self[1][2])
	}
}*/

#[cfg(test)]
pub mod tests {
    use std;
    use std::ops::{Add, Sub, Mul};
    use super::Aabb;
    //use quickcheck;

    pub fn vec3_add<T: Add<Output = T>>(v1: (T, T, T), v2: (T, T, T)) -> (T, T, T) {
        (v1.0 + v2.0, v1.1 + v2.1, v1.2 + v2.2)
    }

    pub fn vec3_sub<T: Sub<Output = T>>(v1: (T, T, T), v2: (T, T, T)) -> (T, T, T) {
        (v1.0 - v2.0, v1.1 - v2.1, v1.2 - v2.2)
    }

    pub fn vec3_num_mul<T: Copy + Mul<Output = T>>(v: (T, T, T), n: T) -> (T, T, T) {
        (v.0 * n, v.1 * n, v.2 * n)
    }

    //type Vec12 = (f32, f32, f32, f32, f32, f32, f32, f32, f32, f32, f32, f32);

    pub fn float_cmp(f1: f32, f2: f32) -> bool {
        (f1 - f2).abs() < std::f32::EPSILON
    }

    #[test]
    fn vec_operations() {
        let a1 = (1.0, 1.0, 1.0);
        let a2 = (2.0, 2.0, 2.0);

        assert_eq!(vec3_add(a1, a2), (3.0, 3.0, 3.0));
        assert_eq!(vec3_sub(a1, a2), (-1.0, -1.0, -1.0));
        assert_eq!(vec3_num_mul(a1, 5.0), (5.0, 5.0, 5.0));
    }

    #[test]
    fn aabb_contains() {
        let a1 = ((-1.0, -1.0, -1.0), (1.0, 1.0, 1.0));
        let a2 = ((-2.0, -2.0, -2.0), (2.0, 2.0, 2.0));

        assert!(a2.contains(&a1));
        assert!(!a1.contains(&a2));
    }

    #[test]
    fn aabb_perimeter() {
        let a1 = ((-2.5f32, -1.5, -2.0), (2.5f32, 1.5, 2.0));
        assert!(float_cmp(a1.perimeter(), 48.0));

        let a2 = ((-2.0f32, -2.0, -2.0), (2.0f32, 3.0, 2.0));
        assert!(float_cmp(a2.perimeter(), 52.0));
    }

    #[test]
    #[should_panic(expected = "AABB is not valid")]
    fn aabb_overlaps() {
        let a1 = ((-1.0, -1.0, -1.0), (1.0, 1.0, 1.0));
        let a2 = ((0.0, 0.0, 0.0), (2.0, 2.0, 2.0));

        assert!(a1.overlaps(&a2));
        assert!(a2.overlaps(&a1));

        let a5 = ((-1.0, -1.0, 2.0), (1.0, 1.0, 4.0));
        let a6 = ((-1.0, -1.0, -1.0), (1.0, 1.0, 2.5));

        assert!(a6.overlaps(&a5));
        assert!(a5.overlaps(&a6));

        let a3 = ((-1.0, -1.0, -1.0), (1.0, 1.0, 1.0));
        let a4 = ((1.0, 1.0, 1.0), (-1.0, -1.0, -1.0));

        assert!(!a3.overlaps(&a4));
        assert!(!a4.overlaps(&a3));
    }

    /*#[test]
	fn aabb_center() {
		let a1 = ((-1.0, -1.0, -1.0), (1.0, 1.0, 1.0));
		assert_eq!(a1.center(), (0.0, 0.0, 0.0));

		let a2 = ((0.0, 0.0, 0.0), (10.0, 10.0, 10.0));
		assert_eq!(a2.center(), (5.0, 5.0, 5.0));
	}*/

    /*#[test]
	fn aabb_diagonal() {
		let a1 = ((-1.0, -1.0, -1.0), (1.0, 1.0, 1.0));
		assert_eq!(a1.diagonal(), (2.0, 2.0, 2.0));

		let a2 = ((0.0, 0.0, 0.0), (10.0, 10.0, 10.0));
		assert_eq!(a2.diagonal(), (10.0, 10.0, 10.0));
	}*/

    #[test]
    fn aabb_valid() {
        let a1 = ((-1.0, -1.0, -1.0), (1.0, 1.0, 1.0));
        assert!(a1.is_valid());

        let a2 = ((1.0, 1.0, 1.0), (-1.0, -1.0, -1.0));
        assert!(!a2.is_valid());

        let a3 = ((0.0, 0.0, 1.0), (0.0, 0.0, 0.0));
        assert!(!a3.is_valid());
    }

    /*fn perimeter_is_larger_when_containing_prop(v: Vec12) -> quickcheck::TestResult {
		let a1 = ((v.0, v.1, v.2), (v.3, v.4 , v.5 ));
		let a2 = ((v.6, v.7, v.8), (v.9, v.10, v.11));

		if a1.is_valid() && a2.is_valid() {
			if a1.contains(a2) {
				return quickcheck::TestResult::from_bool(a1.perimeter() > a2.perimeter())
			}
			else if a2.contains(a1) {
				return quickcheck::TestResult::from_bool(a2.perimeter() > a1.perimeter())
			}
		}

		quickcheck::TestResult::discard()
	}*/

    /*#[test]
	fn perimeter_is_larger_when_containing() {
		quickcheck::quickcheck(perimeter_is_larger_when_containing_prop as fn(Vec12) -> quickcheck::TestResult);
	}*/

    /*fn overlaps_when_containing_prop(v: Vec12) -> quickcheck::TestResult {
		let a1 = ((v.0, v.1, v.2), (v.3, v.4 , v.5 ));
		let a2 = ((v.6, v.7, v.8), (v.9, v.10, v.11));

		if (a1.is_valid() && a2.is_valid()) && (a1.contains(a2) || a2.contains(a1)) {
			quickcheck::TestResult::from_bool(a1.overlaps(a2) && a2.overlaps(a1))
		}
		else {
			quickcheck::TestResult::discard()
		}
	}*/

    /*#[test]
	fn overlaps_when_containing() {
		quickcheck::quickcheck(overlaps_when_containing_prop as fn(Vec12) -> quickcheck::TestResult);
	}*/

    // FIXME: this doesn't really test anything
	/*fn contains_when_overlaps_prop(v: Vec12) -> quickcheck::TestResult {
		let a1 = ((v.0, v.1, v.2), (v.3, v.4 , v.5 ));
		let a2 = ((v.6, v.7, v.8), (v.9, v.10, v.11));

		if (a1.is_valid() && a2.is_valid()) && (a1.overlaps(a2) || a2.overlaps(a1)) && (a1.contains(a2) || a2.contains(a1)) {
			return quickcheck::TestResult::from_bool(true);
		}

		quickcheck::TestResult::discard()
	}*/

    /*#[test]
	fn contains_when_overlaps() {
		quickcheck::quickcheck(contains_when_overlaps_prop as fn(Vec12) -> quickcheck::TestResult);
	}*/
}
