/* -*- coding: utf-8 -*-
 *
 * OneEuroFilter.cc -
 *
 * Author: Nicolas Roussel (nicolas.roussel@inria.fr)
 *
 */

#include "OneEuroFilter.h"

namespace gametrak {


  void LowPassFilter::setAlpha(double alpha)
  {
    if (alpha<0.0 || alpha>1.0)
    {
        alpha = 0.3;
        std::cout<<"Warning: alpha should be in [0.0., 1.0], set default alpha=0.3"<<std::endl;
    }

    a = alpha ;
  }


  LowPassFilter::LowPassFilter(double alpha, double initval) {
    y = s = initval ;
    setAlpha(alpha) ;
    initialized = false ;
  }

  double LowPassFilter::filter(double value) {
    double result ;
    if (initialized)
      result = a*value + (1.0-a)*s ;
    else {
      result = value ;
      initialized = true ;
    }
    y = value ;
    s = result ;
    return result ;
  }

  double LowPassFilter::filterWithAlpha(double value, double alpha) {
    setAlpha(alpha) ;
    return filter(value) ;
  }

  bool LowPassFilter::hasLastRawValue(void) {
    return initialized ;
  }

  double LowPassFilter::lastRawValue(void) {
    return y ;
  }



// -----------------------------------------------------------------


  double OneEuroFilter::alpha(double cutoff) {
    double te = 1.0 / freq ;
    double tau = 1.0 / (2*M_PI*cutoff) ;
    return 1.0 / (1.0 + tau/te) ;
  }

  void OneEuroFilter::setFrequency(double f) {
    if (f<=0) throw std::range_error("freq should be >0") ;
    freq = f ;
  }

  void OneEuroFilter::setMinCutoff(double mc) {
    if (mc<=0) throw std::range_error("mincutoff should be >0") ;
    mincutoff = mc ;
  }

  void OneEuroFilter::setBeta(double b) {
    beta_ = b ;
  }

  void OneEuroFilter::setDerivateCutoff(double dc) {
    if (dc<=0) throw std::range_error("dcutoff should be >0") ;
    dcutoff = dc ;
  }


  OneEuroFilter::OneEuroFilter(double freq, 
		double mincutoff, double beta_, double dcutoff) {
    setFrequency(freq) ;
    setMinCutoff(mincutoff) ;
    setBeta(beta_) ;
    setDerivateCutoff(dcutoff) ;
    x = new LowPassFilter(alpha(mincutoff)) ;
    dx = new LowPassFilter(alpha(dcutoff)) ;
    lasttime = UndefinedTime ;
  }

  double OneEuroFilter::filter(double value, double timestamp) {
    // update the sampling frequency based on timestamps
    if (lasttime!=UndefinedTime && timestamp!=UndefinedTime)
      freq = 1.0 / (timestamp-lasttime) ;
    lasttime = timestamp ;
    // estimate the current variation per second 
    double dvalue = x->hasLastRawValue() ? (value - x->lastRawValue())*freq : 0.0 ; // FIXME: 0.0 or value?
    double edvalue = dx->filterWithAlpha(dvalue, alpha(dcutoff)) ;
    // use it to update the cutoff frequency
    double cutoff = mincutoff + beta_*fabs(edvalue) ;
    // filter the given value
    return x->filterWithAlpha(value, alpha(cutoff)) ;
  }

  OneEuroFilter::~OneEuroFilter(void) {
    delete x ;
    delete dx ;
  }


}
