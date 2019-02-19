/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture                          *
*                (c) 2006-2018 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                           Plugin SoftRobots    v1.0                         *
*				                                              *
* This plugin is also distributed under the GNU LGPL (Lesser General          *
* Public License) license with the same conditions than SOFA.                 *
*                                                                             *
* Contributors: Defrost team  (INRIA, University of Lille, CNRS,              *
*               Ecole Centrale de Lille)                                      *
*                                                                             *
* Contact information: https://project.inria.fr/softrobot/contact/            *
*                                                                             *
******************************************************************************/
#ifndef _SPLIT_FLOAT_HEADER_
#define _SPLIT_FLOAT_HEADER_

#define _USE_MATH_DEFINES

/*==========  Includes  ==========*/
#include <limits.h>
#include <float.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#ifdef _MSC_VER
#define isnan(x) _isnan(x)  // VC++ uses _isnan() instead of isnan()
#define isinf(x) (!_finite(x)) 
#define INFINITY (DBL_MAX+DBL_MAX)
#define NAN (INFINITY-INFINITY)
#  define TRUE 1
#  define FALSE 0
//bool signbit(double num) { return _copysign(1.0, num) < 0; }
#define signbit(x) ((x) < 0.0)
#define truncf(x) floor(x)
#endif


/*==========  Errors  ==========*/
#if CHAR_BIT != 8
#error currently supported only CHAR_BIT = 8
#endif

#if FLT_RADIX != 2
#error currently supported only FLT_RADIX = 2
#endif

/*==========  Defines & Types  ==========*/

typedef unsigned char uint8;

#define VALUE_AND_NAME(V) { V, #V }


    
#ifdef __cplusplus
extern "C" {
#endif
/*====================================
=            Declarations            =
====================================*/

/**
 * 4-byte little-endian serialized format for float:
 *   - normalized mantissa stored as 24-bit (3-byte) signed integer:
 *       negative range: (-2^23, -2^24]
 *         -> herited from frexpf and its mantissa (-0.5, -1]
 *       zero: 0
 *       positive range: [+2^23, +2^24)
 *         -> herited from frexpf and its mantissa (0.5, 1]
 *   - 8-bit (1-byte) signed exponent:
 *         range: [-0x7E, +0x7E]
 *   
 * Represented value = mantissa * 2^(exponent - 127)
 *     
 * Special cases:
 *   - +infinity: mantissa = 0x7FFFFF, exp = 0x7F
 *   - -infinity: mantissa = 0x800000, exp = 0x7F 
 *   - NaN:       mantissa = 0x000000, exp = 0x7F
 *   - +/-0:      only one zero supported
 */
void Float2Bytes(uint8 buf[4], float f);

void Bytes2Float(float* f, const uint8 buf[4]);

//int test(float x, const char* name);

//int testInf(void);

//void fullTest(void);

/*-----  End of Declarations  ------*/
#ifdef __cplusplus
}
#endif

#endif // _SPLIT_FLOAT_HEADER_
