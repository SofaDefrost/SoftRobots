/* -*- mode: c++ -*-
 *
 * libgametrak/utils/Vecteur3D.h --
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

#ifndef __vecteur3d_h__
#define __vecteur3d_h__

#include <iostream>
#include <math.h>

namespace gametrak {

#ifdef _DEBUG

#include <stdlib.h>
#define TEST_ERROR(cond,error_msg) {if(!(cond)){ std::cerr << (error_msg) << std::endl << " in " << __FILE__ << std::endl << " line " << __LINE__ << std::endl; exit(-1); }}
#define TEST_WARNING(cond,warning_msg) {if(!(cond)) std::cerr << (warning_msg) << std::endl << " in " << __FILE__ << std::endl << " line " << __LINE__ << std::endl; }

#else

#define TEST_ERROR(cond,error_msg)
#define TEST_WARNING(cond,warning_msg)

#endif

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

typedef double real;

class Vecteur3D
{
	 friend std::ostream &operator <<(std::ostream &flux,Vecteur3D &v);
public:
  real x,y,z;

  Vecteur3D(){x=y=z=0; };
  Vecteur3D(real _x,real _y,real _z):x(_x),y(_y),z(_z){};
  
  inline void nul(void){x=y=z=0.0;};

  inline real &operator [](int i)
  {
    TEST_ERROR(i>=0 && i<3,"Vecteur3D[] : 3 components vector !!");
    return *(&x+i);
  };

	// Affectation d'un scalaire
  inline Vecteur3D operator =(const real &v) const
  { return Vecteur3D(v,v,v); };

	// Différent
	inline bool operator !=(const Vecteur3D &v) const
	{ return ((x != v.x) && (y != v.y) && (z != v.z));};

  // Addition
  inline Vecteur3D operator +(const Vecteur3D &v) const
  { return Vecteur3D(x+v.x,y+v.y,z+v.z); };
  inline Vecteur3D &operator +=(const Vecteur3D &v)
  { x+=v.x; y+=v.y; z+=v.z; return *this; };

  // Soustraction
  inline Vecteur3D operator -(const Vecteur3D &v) const
  { return Vecteur3D(x-v.x,y-v.y,z-v.z); };
  inline Vecteur3D &operator -=(const Vecteur3D &v)
  { x-=v.x; y-=v.y; z-=v.z; return *this; };
  inline Vecteur3D operator -(void) const
  { return Vecteur3D(-x,-y,-z); };

  // Multiplication par un scalaire
  inline Vecteur3D &operator *=(real f)
  { x*=f; y*=f; z*=f; return *this; };
  inline Vecteur3D operator *(real f) const
  { return Vecteur3D(x*f,y*f,z*f); };

  // Norme et Carre de la norme
  inline real norme(void) const
  { return (real)sqrt(x*x+y*y+z*z); };
  inline real carreNorme(void) const
  { return (x*x+y*y+z*z); };
	inline void normalise(void)
	{ real t=(real)1.0/(real)sqrt((double)(x*x+y*y+z*z));
		x*=t; y*=t; z*=t;
	};

  // Produit Scalaire
  inline real operator *(const Vecteur3D &a) const
  { return (x*a.x+y*a.y+z*a.z); };
  
  // Produit Vectorielle
  // Attention au priorite des operateurs !!
  inline Vecteur3D operator ^(const Vecteur3D &a) const
  { return Vecteur3D( y*a.z-z*a.y , z*a.x-x*a.z , x*a.y-y*a.x ); };
  inline Vecteur3D &operator ^=(const Vecteur3D &a)
  {
    real tempx=x,tempy=y;
    x=y*a.z-z*a.y; y=z*a.x-tempx*a.z; z=tempx*a.y-tempy*a.x;
    return *this;
  };
};

//std::ostream &operator <<(std::ostream &flux,Vecteur3D &v);
}

#endif
