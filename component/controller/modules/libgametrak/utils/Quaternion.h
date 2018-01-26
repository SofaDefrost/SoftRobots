/* -*- mode: c++ -*-
 *
 * libgametrak/utils/Quaternion.h --
 *
 * Initial software
 * Authors: Géry Casiez (gery.casiez@lifl.fr)
 * Copyright University Lille 1, Inria
 *
 * https://code.google.com/p/libgametrak/
 *
 * This software may be used and distributed according to the terms of
 * the GNU General Public License version 2 or any later version.
 *
 */

#ifndef _QUATERNION_
#define _QUATERNION_

#include "vecteur3d.h"
#include <assert.h>
#include <iostream>

namespace gametrak {

class   Quaternion
{
  public:
	double _q[4];

  public:
	Quaternion();
	virtual ~Quaternion();
	Quaternion(double x, double y, double z, double w);
	Quaternion(double q[]);

	// Normalize a quaternion
	void Normalize(void);

	// Gives the nom of a quaternion
	double Norm(void);

	// Given two quaternions, add them together to get a third quaternion.
	// Adding quaternions to get a compound rotation is analagous to adding
	// translations to get a compound translation.  
	friend Quaternion operator+(Quaternion q1, Quaternion q2);

	// Given two Quaternions, multiply them together to get a third quaternion.
	friend Quaternion operator*(const Quaternion& q1, const Quaternion& q2);

	// Multiply a quaternion by a scalar
	friend Quaternion operator*(const Quaternion& q1, const double a);
	friend Quaternion operator*(const double a, const Quaternion& q1);

	// Conjuguate of a quaternion
	friend Quaternion operator~(const Quaternion& q1);

	// Power of a quaternion
	friend Quaternion operator^(Quaternion q1, const double n);

	// Computes the exponential of a quaternion
	Quaternion Exp(void);

	// Computes the ln of a quaternion
	Quaternion Ln(void);

	//Dot product
	double Dot(const Quaternion& q1, const Quaternion& q2);

	Quaternion quatVectMult(const Vecteur3D& vect);

	Quaternion vectQuatMult(const Vecteur3D& vect);

	inline double& operator[](int index) const
	{
		assert(index >= 0 && index < 4);
		return *(const_cast < double* >(&_q[index]));
	}

	// Returns the inverse of a quaternion
	Quaternion invert(void);

	// A useful function, builds a rotation matrix in Matrix based on
	// given quaternion. m is in OpenGL format
	void BuildRotationMatrix(double m[16]);

	// m is in OpenGL format
	void Matrix2QuaternionAndPosition( double m[16], Vecteur3D &T );

	void Matrix2Quaternion( double m[16]);

	// This function computes a quaternion based on an axis (defined by
	// the given vector) and an angle about which to rotate.  The angle is
	// expressed in radians.  
	void AxisToQuaternion(Vecteur3D a, double phi);

	// Given an axis and an angle in radian, computes the quaternion
	void QuaternionToAxisAndAngle(Vecteur3D& u, double& alpha);

	Vecteur3D RotateVector(Vecteur3D u);

	// Computes the angle of the smallest rotation connecting the current quaternion and q2
	double SmallestRotation(Quaternion &q2);

	Quaternion  slerp(Quaternion &q0, const Quaternion &q1, double t);

	// Print the quaternion
	friend std::ostream&	operator<<(std::ostream& out, Quaternion Q);
};

}

#endif // _QUATERNION_
