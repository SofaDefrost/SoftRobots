/* -*- coding: utf-8 -*-
 *
 * OneEuroFilter.h -
 *
 * Author: Nicolas Roussel (nicolas.roussel@inria.fr)
 *
 */

#include <iostream>
#include <stdexcept>
#include <cmath>
#include <ctime>
#ifdef WIN32
#define M_PI 3.1415926535897932384626433832795
#endif

#ifndef OneEuroFilter_h
#define OneEUroFilter_h

namespace gametrak {
// -----------------------------------------------------------------
// Utilities

static const double UndefinedTime = -1.0 ;

// -----------------------------------------------------------------

class LowPassFilter {
    
  double y, a, s ;
  bool initialized ;

  void setAlpha(double alpha) ;

public:

  LowPassFilter(double alpha, double initval=0.0) ;

  double filter(double value) ;

  double filterWithAlpha(double value, double alpha) ;

  bool hasLastRawValue(void) ;

  double lastRawValue(void) ;

} ;

// -----------------------------------------------------------------

class OneEuroFilter {

  double freq ;
  double mincutoff ;
  double beta_ ;
  double dcutoff ;
  LowPassFilter *x ;
  LowPassFilter *dx ;
  double lasttime ;

  double alpha(double cutoff) ;

public:

  void setFrequency(double f) ;

  void setMinCutoff(double mc) ;

  void setBeta(double b) ;

  void setDerivateCutoff(double dc) ;


  OneEuroFilter(double freq, 
		double mincutoff=1.0, double beta_=0.0, double dcutoff=1.0) ;

  double filter(double value, double timestamp=UndefinedTime) ;

  ~OneEuroFilter(void) ;

} ;

}

#endif