/* -*- mode: c++ -*-
 *
 * libgametrak/utils/Vecteur3D.cpp --
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

#include "vecteur3d.h"

namespace gametrak {

std::ostream &operator <<(std::ostream &flux,Vecteur3D &v)
{
  return flux << "(" << v.x << "," << v.y << "," << v.z << ")";
}

}
