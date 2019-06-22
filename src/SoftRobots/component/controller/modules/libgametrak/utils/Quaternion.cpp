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

#include <math.h>
#include <iostream>
#include "Quaternion.h"

namespace gametrak {

#define RENORMCOUNT 50

// Constructor 
Quaternion::Quaternion()
{
	_q[0] = _q[1] = _q[2] = _q[3] = 0.0;
}

Quaternion::Quaternion(double x, double y, double z, double w)
{
	_q[0] = x;
	_q[1] = y;
	_q[2] = z;
	_q[3] = w;
}

Quaternion::Quaternion(double q[])
{
	for (int i = 0; i < 4; i++)
	{
		_q[i] = q[i];
	}
}

// Destructor
Quaternion::~Quaternion()
{
}

// Given two rotations, e1 and e2, expressed as quaternion rotations,
// figure out the equivalent single rotation and stuff it into dest.
// This routine also normalizes the result every RENORMCOUNT times it is
// called, to keep error from creeping in.
//   NOTE: This routine is written so that q1 or q2 may be the same
//  	   as dest (or each other). 
Quaternion operator+(Quaternion q1, Quaternion q2)
{
	static int	count	= 0;

	double		t1[4], t2[4], t3[4];
	double		tf[4];
	Quaternion	ret;
	ret._q[0] = q1._q[0] + q2._q[0];
	ret._q[1] = q1._q[1] + q2._q[1];
	ret._q[2] = q1._q[2] + q2._q[2];
	ret._q[3] = q1._q[3] + q2._q[3];
	ret.Normalize();

	t1[0] = q1._q[0] * q2._q[3];
	t1[1] = q1._q[1] * q2._q[3];
	t1[2] = q1._q[2] * q2._q[3];

	t2[0] = q2._q[0] * q1._q[3];
	t2[1] = q2._q[1] * q1._q[3];
	t2[2] = q2._q[2] * q1._q[3];

	// cross product t3 = q2 x q1
	t3[0] = (q2._q[1] * q1._q[2]) - (q2._q[2] * q1._q[1]);
	t3[1] = (q2._q[2] * q1._q[0]) - (q2._q[0] * q1._q[2]);
	t3[2] = (q2._q[0] * q1._q[1]) - (q2._q[1] * q1._q[0]);
	// end cross product

	tf[0] = t1[0] + t2[0] + t3[0];
	tf[1] = t1[1] + t2[1] + t3[1];
	tf[2] = t1[2] + t2[2] + t3[2];
	tf[3] = q1._q[3] * q2._q[3] -
			(q1._q[0] * q2._q[0] + q1._q[1] * q2._q[1] + q1._q[2] * q2._q[2]);

	ret._q[0] = tf[0];
	ret._q[1] = tf[1];
	ret._q[2] = tf[2];
	ret._q[3] = tf[3];

	if (++count > RENORMCOUNT)
	{
		count = 0;
		ret.Normalize();
	}

	return ret;
}

Quaternion operator*(const Quaternion& q1, const Quaternion& q2)
{
	Quaternion	ret;

	ret._q[3] = q1._q[3] * q2._q[3] -
				(q1._q[0] * q2._q[0] +
				 q1._q[1] * q2._q[1] +
				 q1._q[2] * q2._q[2]);
	ret._q[0] = q1._q[3] * q2._q[0] +
				q1._q[0] * q2._q[3] +
				q1._q[1] * q2._q[2] -
				q1._q[2] * q2._q[1];
	ret._q[1] = q1._q[3] * q2._q[1] +
				q1._q[1] * q2._q[3] +
				q1._q[2] * q2._q[0] -
				q1._q[0] * q2._q[2];
	ret._q[2] = q1._q[3] * q2._q[2] +
				q1._q[2] * q2._q[3] +
				q1._q[0] * q2._q[1] -
				q1._q[1] * q2._q[0];

	return ret;
}

Quaternion operator*(const Quaternion& q1, const double a)
{
	return q1 * Quaternion(0.0, 0.0, 0.0, a);
}

Quaternion operator*(const double a, const Quaternion& q1)
{
	return q1 * Quaternion(0.0, 0.0, 0.0, a);
}

Quaternion operator^(Quaternion q1, const double n)
{
	Quaternion ret = q1.Ln() * n;
	return ret.Exp();
}

Quaternion operator~(const Quaternion& q1)
{
	Quaternion	ret;
	ret._q[0] = -q1[0];
	ret._q[1] = -q1[1];
	ret._q[2] = -q1[2];
	ret._q[3] = q1[3];
	return ret;
}

Quaternion Quaternion::Exp(void)
{
	Quaternion	ret;

	double mult;
	double normedeU = sqrt(_q[0] * _q[0] + _q[1] * _q[1] + _q[2] * _q[2]);

	if (normedeU > 0)
	{
		mult = sin(normedeU) / normedeU * exp(_q[3]);
		ret._q[0] = _q[0] * mult;
		ret._q[1] = _q[1] * mult;
		ret._q[2] = _q[2] * mult;
	}
	else
	{
		ret._q[0] = 0.0;
		ret._q[1] = 0.0;
		ret._q[2] = 0.0;
	}
	ret._q[3] = exp(_q[3]) * cos(normedeU);

	return ret;
}

Quaternion Quaternion::Ln(void)
{
	Quaternion ret;
	double mult;
	double normedeU = sqrt(_q[0] * _q[0] + _q[1] * _q[1] + _q[2] * _q[2]);
	double arg;

	double N = Norm();
	double ratio = _q[3]/Norm();

	static Quaternion QP;
	static double ratioP;
	
	arg = acos(ratio);
 
	if (normedeU > 0.00001)
	{
		mult = arg / normedeU;
		ret._q[0] = _q[0] * mult;
		ret._q[1] = _q[1] * mult;
		ret._q[2] = _q[2] * mult;
	}
	else
	{
		ret._q[0] = 0.0;
		ret._q[1] = 0.0;
		ret._q[2] = 0.0;
	}
	ret._q[3] = log(Norm());

	// test
	//Quaternion ret2 = ret * 0.6;
	//Quaternion E = ret2.Exp();
	//if (E._q[3] < 0.0)
	//	return ret;

	//QP = (*this);
	//ratioP = ratio;
	return ret;
}

double Quaternion::Dot(const Quaternion& q1, const Quaternion& q2)
{
	return q1._q[0] * q2._q[0] + q1._q[1] * q2._q[1] + q1._q[2] * q2._q[2] + q1._q[3] * q2._q[3];
}

Quaternion Quaternion::quatVectMult(const Vecteur3D& vect)
{
	Quaternion	ret;

	ret._q[3] = -(_q[0] * vect.x + _q[1] * vect.y + _q[2] * vect.z);
	ret._q[0] = _q[3] * vect.x + _q[1] * vect.z - _q[2] * vect.y;
	ret._q[1] = _q[3] * vect.y + _q[2] * vect.x - _q[0] * vect.z;
	ret._q[2] = _q[3] * vect.z + _q[0] * vect.y - _q[1] * vect.x;

	return ret;
}

Quaternion Quaternion::vectQuatMult(const Vecteur3D& vect)
{
	Quaternion	ret;

	ret[3] = -(vect.x * _q[0] + vect.y * _q[1] + vect.z * _q[2]);
	ret[0] = vect.x * _q[3] + vect.y * _q[2] - vect.z * _q[1];
	ret[1] = vect.y * _q[3] + vect.z * _q[0] - vect.x * _q[2];
	ret[2] = vect.z * _q[3] + vect.x * _q[1] - vect.y * _q[0];

	return ret;
}

Quaternion Quaternion::invert(void)
{
	double	norm	= Norm();

	if (norm > 0) return ~(*this) * (1.0/norm);
	
	return Quaternion(0.0, 0.0, 0.0, 0.0);
}

// Quaternions always obey:  a^2 + b^2 + c^2 + d^2 = 1.0
// If they don't add up to 1.0, dividing by their magnitued will
// renormalize them.

void Quaternion::Normalize(void)
{
	int		i;
	double	mag;

	mag = (_q[0] * _q[0] + _q[1] * _q[1] + _q[2] * _q[2] + _q[3] * _q[3]);
	for (i = 0; i < 4; i++)
	{
		_q[i] /= sqrt(mag);
	}
}

// Computes the norm of a quaternion
double Quaternion::Norm(void)
{
	return sqrt(_q[0] * _q[0] + _q[1] * _q[1] + _q[2] * _q[2] + _q[3] * _q[3]);
}

// Build a rotation matrix, given a quaternion rotation.
void Quaternion::BuildRotationMatrix(double m[16])
{
	m[0] = (1.0 - 2.0 * (_q[1] * _q[1] + _q[2] * _q[2]));
	m[1] = (2.0 * (_q[0] * _q[1] - _q[2] * _q[3]));
	m[2] = (2.0 * (_q[2] * _q[0] + _q[1] * _q[3]));
	m[3] = 0.0f;

	m[4] = (2.0 * (_q[0] * _q[1] + _q[2] * _q[3]));
	m[5] = (1.0 - 2.0 * (_q[2] * _q[2] + _q[0] * _q[0]));
	m[6] = (2.0 * (_q[1] * _q[2] - _q[0] * _q[3]));
	m[7] = 0.0f;

	m[8] = (2.0 * (_q[2] * _q[0] - _q[1] * _q[3]));
	m[9] = (2.0 * (_q[1] * _q[2] + _q[0] * _q[3]));
	m[10] = (1.0 - 2.0 * (_q[1] * _q[1] + _q[0] * _q[0]));
	m[11] = 0.0f;

	m[12] = 0.0f;
	m[13] = 0.0f;
	m[14] = 0.0f;
	m[15] = 1.0f;
}

void Quaternion::Matrix2QuaternionAndPosition( double m[16], Vecteur3D &T )
{
  double w = m[0] + m[5] + m[10] + 1;

	if (w > 0.00000001)
	{
		w = sqrt( w );
		_q[0] = (m[6] - m[9]) / (w*2.0);
		_q[1] = (m[8] - m[2]) / (w*2.0);
		_q[2] = (m[1] - m[4]) / (w*2.0);
		_q[3] = w / 2.0;
	}
	else
	{
    //If the trace of the matrix is equal to zero then identify
    //which major diagonal element has the greatest value.
    //Depending on this, calculate the following:
		if ( m[0] > m[5] && m[0] > m[10] )  {	// Column 0: 
        w  = sqrt( 1.0 + m[0] - m[5] - m[10] ) * 2;
        _q[0] = 0.25 * w;
        _q[1] = (m[4] + m[1] ) / w;
        _q[2] = (m[2] + m[8] ) / w;
        _q[3] = (m[6] - m[9] ) / w;
    } else if ( m[5] > m[10] ) {			// Column 1: 
        w  = sqrt( 1.0 + m[5] - m[0] - m[10] ) * 2;
        _q[0] = (m[4] + m[1] ) / w;
        _q[1] = 0.25 * w;
        _q[2] = (m[9] + m[6] ) / w;
        _q[3] = (m[8] - m[2] ) / w;
    } else {						// Column 2:
        w  = sqrt( 1.0 + m[10] - m[0] - m[5] ) * 2;
        _q[0] = (m[2] + m[8] ) / w;
        _q[1] = (m[9] + m[6] ) / w;
        _q[2] = 0.25 * w;
        _q[3] = (m[1] - m[4] ) / w;
    }
	}

	T = Vecteur3D(m[12], m[13], m[14]);
}

void Quaternion::Matrix2Quaternion( double m[16])
{
	Vecteur3D T;
  Matrix2QuaternionAndPosition(m, T);
}

// Given an axis and angle, compute quaternion.
void Quaternion::AxisToQuaternion(Vecteur3D a, double phi)
{
	double omega = 0.5f * phi;
	double s = (double)sin(omega);

	a.normalise();
	_q[0] = s * a.x;
	_q[1] = s * a.y;
	_q[2] = s * a.z;

	_q[3] = (double)cos(omega);
}

//Given a quaternion, gives the axis and angle
void Quaternion::QuaternionToAxisAndAngle(Vecteur3D& u, double& alpha)
{
	/* Normalisation du quaternion */
	Normalize();

	/* Récupération de l'angle de rotation */
	double cos_a = _q[3];
	alpha = acos(cos_a) * 2;
	double sin_a = sqrt(1 - cos_a * cos_a);
	if ( fabs( sin_a ) < 0.0005 ) sin_a = 1;

	/* Récupération des composantes de l'axe de rotation */
	u.x = _q[0]/sin_a;
	u.y = _q[1]/sin_a;
	u.z = _q[2]/sin_a;
}

Vecteur3D Quaternion::RotateVector(Vecteur3D u)
{
	Quaternion tmp;
	tmp = (*this) * Quaternion(u.x, u.y, u.z, 0) * (~(*this));
	return Vecteur3D(tmp._q[0],tmp._q[1],tmp._q[2]);
}

double Quaternion::SmallestRotation(Quaternion &q2)
{
	//Normalize();
	//q2.Normalize();
	//double res = 2 * acos(Dot((*this), q2));
	//if (res > M_PI) return res - M_PI;
	//return res;
	Quaternion res = (*this) * q2.invert();
	Vecteur3D u;
	double a;
	res.QuaternionToAxisAndAngle(u, a);
	// returns the min between a and 2*pi-a
	if (a < 2*M_PI-a)
		return a;
	else
		return 2*M_PI-a;
}

Quaternion Quaternion::slerp( Quaternion &q0, const Quaternion &q1, double t)
{
	return ((q1 * q0.invert())^t) * q0;
}

// Output quaternion
std::ostream& operator<<(std::ostream& out, Quaternion Q)
{
	return (out << Q._q[0] << " " << Q._q[1] << " " << Q._q[2] << " "
			<< Q._q[3]);

	//return (out << "(" << Q._q[0] << "," << Q._q[1] << "," << Q._q[2] << ","
	//			<< Q._q[3] << ")");
}

}
