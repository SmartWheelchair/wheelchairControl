#ifndef QUATERNION_H
#define QUATERNION_H

#include <cmath>
#include <mbed.h>
#include "tmatrix.h"
class Quaternion
{
public:
	typedef float FloatType;

private:
	FloatType mData[4];

public:

	Quaternion() {
		mData[0] = mData[1] = mData[2] = 0;
		mData[3] = 1;
	}

	Quaternion(const TVector3& v, FloatType w) {
		mData[0] = v.element(0,0);
		mData[1] = v.element(1,0);
		mData[2] = v.element(2,0);
		mData[3] = w;
	}

	Quaternion(const TVector4& v) {
		mData[0] = v.element(0,0);
		mData[1] = v.element(1,0);
		mData[2] = v.element(2,0);
		mData[3] = v.element(3,0);
	}

	Quaternion(const FloatType* array) {
		MBED_ASSERT(array != NULL);
		for (uint32_t i = 0; i < 4; i++) {
			mData[i] = array[i];
		}
	}

	Quaternion(FloatType x, FloatType y, FloatType z, FloatType w) {
		mData[0] = x;
		mData[1] = y;
		mData[2] = z;
		mData[3] = w;
	}

	FloatType x() const { return mData[0]; }
	FloatType y() const { return mData[1]; }
	FloatType z() const { return mData[2]; }
	FloatType w() const { return real(); }

	TVector3 complex() const { return TVector3(mData); }
	void complex(const TVector3& c) { mData[0] = c[0]; mData[1] = c[1];  mData[2] = c[2]; }

	FloatType real() const { return mData[3]; }
	void real(FloatType r) { mData[3] = r; }

	Quaternion conjugate(void) const {
		return Quaternion(-complex(), real());
	}

	/**
	 * @brief Computes the inverse of this quaternion.
	 *
	 * @note This is a general inverse.  If you know a priori
	 * that you're using a unit quaternion (i.e., norm() == 1),
	 * it will be significantly faster to use conjugate() instead.
	 *
	 * @return The quaternion q such that q * (*this) == (*this) * q
	 * == [ 0 0 0 1 ]<sup>T</sup>.
	 */
	Quaternion inverse(void) const {
		return conjugate() / norm();
	}


	/**
	 * @brief Computes the product of this quaternion with the
	 * quaternion 'rhs'.
	 *
	 * @param rhs The right-hand-side of the product operation.
	 *
	 * @return The quaternion product (*this) x @p rhs.
	 */
	Quaternion product(const Quaternion& rhs) const {
		return Quaternion(y()*rhs.z() - z()*rhs.y() + x()*rhs.w() + w()*rhs.x(),
						  z()*rhs.x() - x()*rhs.z() + y()*rhs.w() + w()*rhs.y(),
						  x()*rhs.y() - y()*rhs.x() + z()*rhs.w() + w()*rhs.z(),
						  w()*rhs.w() - x()*rhs.x() - y()*rhs.y() - z()*rhs.z());
	}

	/**
	 * @brief Quaternion product operator.
	 *
	 * The result is a quaternion such that:
	 *
	 * result.real() = (*this).real() * rhs.real() -
	 * (*this).complex().dot(rhs.complex());
	 *
	 * and:
	 *
	 * result.complex() = rhs.complex() * (*this).real
	 * + (*this).complex() * rhs.real()
	 * - (*this).complex().cross(rhs.complex());
	 *
	 * @return The quaternion product (*this) x rhs.
	 */
	Quaternion operator*(const Quaternion& rhs) const {
		return product(rhs);
	}

	/**
	 * @brief Quaternion scalar product operator.
	 * @param s A scalar by which to multiply all components
	 * of this quaternion.
	 * @return The quaternion (*this) * s.
	 */
	Quaternion operator*(FloatType s) const {
		return Quaternion(complex()*s, real()*s);
	}

	/**
	 * @brief Produces the sum of this quaternion and rhs.
	 */
	Quaternion operator+(const Quaternion& rhs) const {
		return Quaternion(x()+rhs.x(), y()+rhs.y(), z()+rhs.z(), w()+rhs.w());
	}

	/**
	 * @brief Produces the difference of this quaternion and rhs.
	 */
	Quaternion operator-(const Quaternion& rhs) const {
		return Quaternion(x()-rhs.x(), y()-rhs.y(), z()-rhs.z(), w()-rhs.w());
	}

	/**
	 * @brief Unary negation.
	 */
	Quaternion operator-() const {
		return Quaternion(-x(), -y(), -z(), -w());
	}

	/**
	 * @brief Quaternion scalar division operator.
	 * @param s A scalar by which to divide all components
	 * of this quaternion.
	 * @return The quaternion (*this) / s.
	 */
	Quaternion operator/(FloatType s) const {
		MBED_ASSERT(s != 0);
		return Quaternion(complex()/s, real()/s);
	}

	/**
	 * @brief Returns a matrix representation of this
	 * quaternion.
	 *
	 * Specifically this is the matrix such that:
	 *
	 * this->matrix() * q.vector() = (*this) * q for any quaternion q.
	 *
	 * Note that this is @e NOT the rotation matrix that may be
	 * represented by a unit quaternion.
	 */
	TMatrix4 matrix() const {
		FloatType m[16] = {
				w(), -z(),  y(), x(),
				z(),  w(), -x(), y(),
				-y(),  x(),  w(), z(),
				-x(), -y(), -z(), w()
		};
		return TMatrix4(m);
	}

	/**
	 * @brief Returns a matrix representation of this
	 * quaternion for right multiplication.
	 *
	 * Specifically this is the matrix such that:
	 *
	 * q.vector().transpose() * this->matrix() = (q *
	 * (*this)).vector().transpose() for any quaternion q.
	 *
	 * Note that this is @e NOT the rotation matrix that may be
	 * represented by a unit quaternion.
	 */
	TMatrix4 rightMatrix() const {
		FloatType m[16] = {
				+w(), -z(),  y(), -x(),
				+z(),  w(), -x(), -y(),
				-y(),  x(),  w(), -z(),
				+x(),  y(),  z(),  w()
		};
		return TMatrix4(m);
	}

	/**
	 * @brief Returns this quaternion as a 4-vector.
	 *
	 * This is simply the vector [x y z w]<sup>T</sup>
	 */
	TVector4 vector() const { return TVector4(mData); }

	/**
	 * @brief Returns the norm ("magnitude") of the quaternion.
	 * @return The 2-norm of [ w(), x(), y(), z() ]<sup>T</sup>.
	 */
	FloatType norm() const { return sqrt(mData[0]*mData[0]+mData[1]*mData[1]+
									  mData[2]*mData[2]+mData[3]*mData[3]); }

	/**
	 * @brief Computes the rotation matrix represented by a unit
	 * quaternion.
	 *
	 * @note This does not check that this quaternion is normalized.
	 * It formulaically returns the matrix, which will not be a
	 * rotation if the quaternion is non-unit.
	 */
	TMatrix3 rotationMatrix() const {
		FloatType m[9] = {
				1-2*y()*y()-2*z()*z(), 2*x()*y() - 2*z()*w(), 2*x()*z() + 2*y()*w(),
				2*x()*y() + 2*z()*w(), 1-2*x()*x()-2*z()*z(), 2*y()*z() - 2*x()*w(),
				2*x()*z() - 2*y()*w(), 2*y()*z() + 2*x()*w(), 1-2*x()*x()-2*y()*y()
		};
		return TMatrix3(m);
	}
	/**
	 * @brief Sets quaternion to be same as rotation by scaled axis w.
	 */
	void scaledAxis(const TVector3& w) {
		FloatType theta = w.norm();
		if (theta > 0.0001) {
			FloatType s = sin(theta / 2.0);
			TVector3 W(w / theta * s);
			mData[0] = W[0];
			mData[1] = W[1];
			mData[2] = W[2];
			mData[3] = cos(theta / 2.0);
		} else {
			mData[0]=mData[1]=mData[2]=0;
			mData[3]=1.0;
		}
	}

	/**
	 * @brief Returns a vector rotated by this quaternion.
	 *
	 * Functionally equivalent to:  (rotationMatrix() * v)
	 * or (q * Quaternion(0, v) * q.inverse()).
	 *
	 * @warning conjugate() is used instead of inverse() for better
	 * performance, when this quaternion must be normalized.
	 */
	TVector3 rotatedVector(const TVector3& v) const {
		return (((*this) * Quaternion(v, 0)) * conjugate()).complex();
	}



	/**
	 * @brief Computes the quaternion that is equivalent to a given
	 * euler angle rotation.
	 * @param euler A 3-vector in order:  roll-pitch-yaw.
	 */
	void euler(const TVector3& euler) {
		FloatType c1 = cos(euler[2] * 0.5f);
		FloatType c2 = cos(euler[1] * 0.5f);
		FloatType c3 = cos(euler[0] * 0.5f);
		FloatType s1 = sin(euler[2] * 0.5f);
		FloatType s2 = sin(euler[1] * 0.5f);
		FloatType s3 = sin(euler[0] * 0.5f);

		mData[0] = c1*c2*s3 - s1*s2*c3;
		mData[1] = c1*s2*c3 + s1*c2*s3;
		mData[2] = s1*c2*c3 - c1*s2*s3;
		mData[3] = c1*c2*c3 + s1*s2*s3;
	}

	/** @brief Returns an equivalent euler angle representation of
	 * this quaternion.
	 * @return Euler angles in roll-pitch-yaw order.
	 */
	TVector3 euler(void) const {
		TVector3 euler;
		const static FloatType PI_OVER_2 = M_PI * 0.5;
		const static FloatType EPSILON = 1e-10;
		FloatType sqw, sqx, sqy, sqz;

		// quick conversion to Euler angles to give tilt to user
		sqw = mData[3]*mData[3];
		sqx = mData[0]*mData[0];
		sqy = mData[1]*mData[1];
		sqz = mData[2]*mData[2];

		euler[1] = asin(2.0f * (mData[3]*mData[1] - mData[0]*mData[2]));
		if (PI_OVER_2 - fabs(euler[1]) > EPSILON) {
			euler[2] = atan2(2.0f * (mData[0]*mData[1] + mData[3]*mData[2]),
							 sqx - sqy - sqz + sqw);
			euler[0] = atan2(2.0f * (mData[3]*mData[0] + mData[1]*mData[2]),
							 sqw - sqx - sqy + sqz);
		} else {
			// compute heading from local 'down' vector
			euler[2] = atan2(2*mData[1]*mData[2] - 2*mData[0]*mData[3],
							 2*mData[0]*mData[2] + 2*mData[1]*mData[3]);
			euler[0] = 0.0;

			// If facing down, reverse yaw
			if (euler[1] < 0)
				euler[2] = M_PI - euler[2];
		}
		return euler;
	}

	/**
	 * @brief Returns pointer to the internal array.
	 *
	 * Array is in order x,y,z,w.
	 */
	FloatType* row(uint32_t i) { return mData + i; }
	// Const version of the above.
	const FloatType* row(uint32_t i) const { return mData + i; }
};

/**
 * @brief Global operator allowing left-multiply by scalar.
 */
Quaternion operator*(Quaternion::FloatType s, const Quaternion& q);


#endif /* QUATERNION_H */
