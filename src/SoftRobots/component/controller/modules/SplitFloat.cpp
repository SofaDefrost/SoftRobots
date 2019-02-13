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
#include "SplitFloat.h"

const struct
{
  double value;
  const char* name;
} testData[] =
{
#ifdef NAN
  VALUE_AND_NAME(NAN),
#endif
  VALUE_AND_NAME(0.0),
  VALUE_AND_NAME(+DBL_MIN),
  VALUE_AND_NAME(-DBL_MIN),
  VALUE_AND_NAME(+1.0),
  VALUE_AND_NAME(-1.0),
  VALUE_AND_NAME(+M_PI),
  VALUE_AND_NAME(-M_PI),
  VALUE_AND_NAME(+DBL_MAX),
  VALUE_AND_NAME(-DBL_MAX),
  VALUE_AND_NAME(+INFINITY),
  VALUE_AND_NAME(-INFINITY),
};


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
void Float2Bytes(uint8 buf[4], float f)
{
    // Special cases - NaN (Not a Number)
    if(isnan(f))
    {
        memcpy(buf, "\x00\x00\x00" "\x7F", 4);
    }
    // Special cases - Infinite
    else if (isinf(f))
    {
        // Negative
        if(signbit(f))
        {
            memcpy(buf, "\x80\x00\x00" "\x7F", 4);
        }
        // Positive
        else
        {
            memcpy(buf, "\x7F\xFF\xFF" "\x7F", 4);
        }
    }
    // Normal cases and zero
    else
    {
        // Get sign
        int e = 0;
        char sign = (signbit(f))?1:0;

        // Get mantissa (significand) and exponent
        float m = frexpf(f, &e);

        // // Tests
        // printf("F2B\n");
        // printf("\tMantisse = %+.15E\n", m);
        // printf("\tExponent = %d\n", e);
        // printf("\tSign = '%s'\n", ((sign)?"-":"+"));

        // Case of too big exponent - convert to infinite
        if(e > 0x7E)
        {
            // Negative
            if(sign)
            {
                memcpy(buf, "\x80\x00\x00" "\x7F", 4);
            }
            // Positive
            else
            {
                memcpy(buf, "\x7F\xFF\xFF" "\x7F", 4);
            }
        }
        // Case of too small exponent - convert to zero
        else if(e < -0x7E)
        {
            memcpy(buf, "\x00\x00\x00" "\x00", 4);
        }
        // Normal cases
        {
            // Convert mantissa into integer
            // Get 23 most significants bits of m
            m = ldexpf(m, 23);

            // Negative, convert to positive
            if(sign)
                m *= -1;

            // Truncate to integer value
            long int m_int = truncf(m); // At least 32-bits needed

            // // Tests
            // printf("\tMantissa integer = %ld et m equivalent: %f\n", m_int, m);

            // Store m_int as unsigned 23-bits little-endian integer
            m_int &= 0x7FFFFF;
            int i;
            for (i = 0; i < 3; i++, m_int >>= 8)
                buf[i] = (uint8)m_int;

            // Add signbit
            buf[2] |= sign << 7;

            // Store exponent as signed 8-bits integer
            buf[3] = (uint8)e;
        }
    }
}

void Bytes2Float(float* f, const uint8 buf[4])
{
    // Special cases - NaN (Not a Number)
    if(memcmp(buf, "\x00\x00\x00" "\x7F", 4) == 0)
    {
        // NaN is supported
#ifdef NAN
        *f = NAN;
        // NaN is not supported, use 0 instead (we could return an error)
#else
        *f = 0;
#endif
    }
    // Special cases - Infinite negative
    else if (memcmp(buf, "\x80\x00\x00" "\x7F", 4) == 0)
    {
        *f = -INFINITY;
    }
    // Special cases - Infinite positive
    else if (memcmp(buf, "\x7F\xFF\xFF" "\x7F", 4) == 0)
    {
        *f = INFINITY;
    }
    // Normal cases and zero
    else
    {
        // Get exponent
        unsigned int e_unsigned = buf[3];
        int e;

        // Positive
        if (e_unsigned <= 0x7F)
            e = e_unsigned;
        // Negative
        else
            e = (signed int)(e_unsigned - 0x7F - 1) - 0x7F - 1;


        long int m_int = 0; // At least 32-bits needed

        // Get mantissa
        int i;
        for(i=2; i>=0; i--)
        {
            m_int <<= 8;
            m_int |= buf[i];
        }

        // Get sign
        char sign = m_int >> 23;
        m_int &= 0x7FFFFF;

        // // Tests
        // printf("B2F\n");
        // printf("\tMantissa integer = %ld\n", m_int);

        // Adapt mantissa to precision
        m_int <<= ((FLT_MANT_DIG < 23)?23 - FLT_MANT_DIG:0);

        // Scale mantissa [0.5, 1)
        float m = ldexpf((float)m_int,((FLT_MANT_DIG < 23)?-FLT_MANT_DIG:-23));
        if(sign)
            m *= -1;

        // // Tests
        // printf("\tMantisse = %+.15E\n", m);
        // printf("\tExponent = %d\n", e);
        // printf("\tSign = '%s'\n", ((sign)?"-":"+"));

        // Exponent higher that max exponent
        int max_e = 0;
        frexpf(FLT_MAX, &max_e);
        if (e > max_e)
        {
            if(m > 0)
                m = INFINITY;
            else
                m = -INFINITY;
        }
        else
            *f = ldexpf(m, e);
    }
}

/*int test(float x, const char* name)
{
    uint8 buf[4]={0,0,0,0}, buf2[4]={0,0,0,0};
    float x2;
    int error1, error2;

    Float2Bytes(buf, x);
    Bytes2Float(&x2, buf);
    Float2Bytes(buf2, x2);

    printf("Premier résultat Float2Bytes\n");
    printf("\t%+.15E '%s' -> %02X %02X %02X %02X\n",
          x,
          name,
          buf[0],buf[1],buf[2],buf[3]);

    printf("Réversibilité Float2Bytes/Bytes2Float : Bytes2Float(Float2Bytes(x))\n");
    if ((error1 = memcmp(&x, &x2, sizeof(x))) != 0)
        puts("\t\033[31;01m____KO____\033[00m");
    else
        puts("\t\033[32;01m!!!!OK!!!!\033[00m!!!!OK!!!!");
    printf("\tResults : %+.15E et %+.15E \n",x,x2);

    printf("Réversibilité des buffers : Float2Bytes(Bytes2Float(Float2Bytes(x)))\n");
    if ((error2 = memcmp(buf, buf2, sizeof(buf))) != 0)
        puts("\t\033[31;01m____KO____\033[00m");
    else
        puts("\t\033[32;01m!!!!OK!!!!\033[00m!!!!OK!!!!");
    printf("\tResults : %02X %02X %02X %02X et %02X %02X %02X %02X \n",
    buf[0],buf[1],buf[2],buf[3],buf2[0],buf2[1],buf2[2],buf2[3]);

    puts("");

    return error1 || error2;
}
*/

/*int testInf(void)
{
  uint8 buf[10];
  float x, x2;
  int error;

  x = DBL_MAX;
  Float2Bytes(buf, x);
  if (!++buf[8])
    ++buf[9]; // increment the exponent beyond the maximum
  Bytes2Float(&x2, buf);

  printf("%02X %02X %02X %02X %02X %02X %02X %02X  %02X %02X -> %+.15E\n",
         buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],
         x2);

  if ((error = !isinf(x2)) != 0)
    puts("Bytes2Float(Float2Bytes(DBL_MAX) * 2) != INF");
    else
    puts("Bytes2Float(Float2Bytes(DBL_MAX) * 2) == INF - OK");


  puts("");

  return error;
}
*/

/*void fullTest(void)
{
    int i = 0;

    do
    {
        test(testData[i].value, testData[i].name);
        i++;
    }while(testData[i].value != -INFINITY);
}
*/
