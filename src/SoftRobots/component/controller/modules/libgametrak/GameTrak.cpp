/* -*- mode: c++ -*-
 *
 * libgametrak/GameTrak.cpp --
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

#include "GameTrak.h"
#include "HIDAPIGameTrak.h"

#ifdef __APPLE__
  #include <CoreServices/CoreServices.h>
#endif

 namespace gametrak {

  GameTrak::GameTrak(void) {
    debugLevel = 0;

    rawLeftTheta = 0;
    rawLeftPhi = 0;
    rawLeftL = 0;
    rawRightTheta = 0;
    rawRightPhi = 0;
    rawRightL = 0;
    button = false;

    filteringEnabled = true;

    mincutoff = 1.0; 
    beta = 0.001;
    dcutoff = 1.0;

    filterLeftTheta = new OneEuroFilter(120, mincutoff, beta, dcutoff) ;
    filterLeftPhi = new OneEuroFilter(120, mincutoff, beta, dcutoff) ;
    filterLeftL = new OneEuroFilter(120, mincutoff, beta, dcutoff) ;
    filterRightTheta = new OneEuroFilter(120, mincutoff, beta, dcutoff) ;
    filterRightPhi = new OneEuroFilter(120, mincutoff, beta, dcutoff) ;
    filterRightL = new OneEuroFilter(120, mincutoff, beta, dcutoff) ;

    rawLeftThetaf = 0;
    rawLeftPhif = 0;
    rawLeftLf = 0;
    rawRightThetaf = 0;
    rawRightPhif = 0;
    rawRightLf = 0;

    LeftTheta = 0; // degrees
    LeftPhi = 0; // degrees
    LeftL = 0; // mm

    RightTheta = 0; // degrees
    RightPhi = 0; // degrees
    RightL = 0; // mm

    // String ends positions
    LeftX = 0;
    LeftY = 0;
    LeftZ = 0;
    RightX = 0;
    RightY = 0;
    RightZ = 0;

    // Calibration
    useCalibration = false;
    calibrating = false;
    calibrated = false;

    minRawLeftTheta = 0;
    minRawLeftPhi = 0;
    minRawLeftL = 0;
    minRawRightTheta = 0;
    minRawRightPhi  = 0;
    minRawRightL  = 0; 

    maxRawLeftTheta  = 4096;
    maxRawLeftPhi  = 4096;
    maxRawLeftL  = 4096;
    maxRawRightTheta  = 4096;
    maxRawRightPhi  = 4096;
    maxRawRightL  = 4096;
  }

  GameTrak *
  GameTrak::create(const char *device_uri) {
    std::string uri ;
    if (device_uri) uri = device_uri ;
    return create(uri) ;
  }

  GameTrak *
  GameTrak::create(std::string device_uri) {
  	return new HIDAPIGameTrak(device_uri);
  }

  void GameTrak::tuneFiltering(double mincutoff, double beta) {
    this->mincutoff = mincutoff;
    this->beta = beta;
    filterLeftTheta->setMinCutoff(mincutoff);
    filterLeftPhi->setMinCutoff(mincutoff);
    filterLeftL->setMinCutoff(mincutoff);
    filterRightTheta->setMinCutoff(mincutoff);
    filterRightPhi->setMinCutoff(mincutoff);
    filterRightL->setMinCutoff(mincutoff);

    filterLeftTheta->setBeta(mincutoff);
    filterLeftPhi->setBeta(mincutoff);
    filterLeftL->setBeta(mincutoff);
    filterRightTheta->setBeta(mincutoff);
    filterRightPhi->setBeta(mincutoff);
    filterRightL->setBeta(mincutoff);
  }

  void GameTrak::FilterRawvalues(double timestamp) {
    rawLeftThetaf = filterLeftTheta->filter(rawLeftTheta, timestamp);
    rawLeftPhif = filterLeftPhi->filter(rawLeftPhi, timestamp);
    rawLeftLf = filterLeftL->filter(rawLeftL, timestamp);
    rawRightThetaf = filterRightTheta->filter(rawRightTheta, timestamp);
    rawRightPhif = filterRightPhi->filter(rawRightPhi, timestamp);
    rawRightLf = filterRightL->filter(rawRightL, timestamp);

  }

  Vecteur3D GameTrak::Transform(double Theta, double Phi, double L)
  {
    Quaternion Qtheta, Qphi, Qres;
    Qtheta.AxisToQuaternion(Vecteur3D(0,0,1),Theta);
    Qphi.AxisToQuaternion(Vecteur3D(1,0,0),Phi);
    Qres = Qtheta * Qphi;
    return Qres.RotateVector(Vecteur3D(0,L,0));
  }

    void getRightCursor(TimeStamp::inttime *ts, double *rX, double *rY, double *rZ);
    void getButton(TimeStamp::inttime *ts, bool *but);
    
  void GameTrak::getGametrakData(TimeStamp::inttime *ts,
                                 double *lX, double *lY, double *lZ,
                                 double *rX, double *rY, double *rZ,
                                 bool *but) {
#ifdef WIN32
      // TODO
#else
      pthread_mutex_lock(&mutex);
#endif
      (*ts) = sync_timeStamp;
      (*lX) = sync_LeftX;
      (*lY) = sync_LeftY;
      (*lZ) = sync_LeftZ;
      (*rX) = sync_RightX;
      (*rY) = sync_RightY;
      (*rZ) = sync_RightZ;
      (*but) = sync_button;
#ifdef WIN32
      // TODO
#else
      pthread_mutex_unlock(&mutex);
#endif
}

  void GameTrak::getLeftCursor(TimeStamp::inttime *ts,
                               double *lX, double *lY, double *lZ) {
#ifdef WIN32
      // TODO
#else
      pthread_mutex_lock(&mutex);
#endif
      (*ts) = sync_timeStamp;
      (*lX) = sync_LeftX;
      (*lY) = sync_LeftY;
      (*lZ) = sync_LeftZ;
#ifdef WIN32
      // TODO
#else
      pthread_mutex_unlock(&mutex);
#endif
  }

  void GameTrak::getRightCursor(TimeStamp::inttime *ts,
                                double *rX, double *rY, double *rZ) {
#ifdef WIN32
      // TODO
#else
      pthread_mutex_lock(&mutex);
#endif
      (*ts) = sync_timeStamp;
      (*rX) = sync_RightX;
      (*rY) = sync_RightY;
      (*rZ) = sync_RightZ;
#ifdef WIN32
      // TODO
#else
      pthread_mutex_unlock(&mutex);
#endif
  }

  void GameTrak::getButton(TimeStamp::inttime *ts,
                           bool *but) {
#ifdef WIN32
      // TODO
#else
      pthread_mutex_lock(&mutex);
#endif
      (*ts) = sync_timeStamp;
      (*but) = sync_button;
#ifdef WIN32
      // TODO
#else
      pthread_mutex_unlock(&mutex);
#endif
  }

  void GameTrak::enterCalibration()
  {
  	if (this->debugLevel > 0) {
        std::cout << "Entering in calibration mode." << std::endl;
  	}
    calibrating = true;
    calibrated = false;

    double mid = 4096.0 / 2.0;

    minRawLeftTheta = mid;
    minRawLeftPhi = mid;
    minRawLeftL = mid;
    minRawRightTheta = mid;
    minRawRightPhi  = mid;
    minRawRightL  = mid; 

    maxRawLeftTheta  = mid;
    maxRawLeftPhi  = mid;
    maxRawLeftL  = mid;
    maxRawRightTheta  = mid;
    maxRawRightPhi  = mid;
    maxRawRightL  = mid; 

  }

  std::string GameTrak::leaveCalibration()
  {
  	if (this->debugLevel > 0) {
        std::cout << "Leaving calibration mode." << std::endl;
  	}
    calibrating = false;
    calibrated = true;
    useCalibration = true;
    return getCalibrationString();
  }

  std::string GameTrak::getCalibrationString() {
    std::stringstream q ;
    q << "milt=" <<  minRawLeftTheta
      << "&milp=" <<  minRawLeftPhi
      << "&mill=" <<  minRawLeftL
      << "&mirt=" <<  minRawRightTheta
      << "&mirp=" <<  minRawRightPhi
      << "&mirl=" <<  minRawRightL

      << "&malt=" <<  maxRawLeftTheta
      << "&malp=" <<  maxRawLeftPhi
      << "&mall=" <<  maxRawLeftL
      << "&mart=" <<  maxRawRightTheta
      << "&marp=" <<  maxRawRightPhi
      << "&marl=" <<  maxRawRightL;

    return q.str();
  }

  void GameTrak::calibrate() {
    if (rawLeftTheta < minRawLeftTheta) minRawLeftTheta = rawLeftTheta;
    if (rawLeftPhi < minRawLeftPhi) minRawLeftPhi = rawLeftPhi;
    if (rawLeftL < minRawLeftL) minRawLeftL = rawLeftL;
    if (rawRightTheta < minRawRightTheta) minRawRightTheta = rawRightTheta;
    if (rawRightPhi < minRawRightPhi) minRawRightPhi = rawRightPhi;
    if (rawRightL < minRawRightL) minRawRightL = rawRightL;

    if (rawLeftTheta > maxRawLeftTheta) maxRawLeftTheta = rawLeftTheta;
    if (rawLeftPhi > maxRawLeftPhi) maxRawLeftPhi = rawLeftPhi;
    if (rawLeftL > maxRawLeftL) maxRawLeftL = rawLeftL;
    if (rawRightTheta > maxRawRightTheta) maxRawRightTheta = rawRightTheta;
    if (rawRightPhi > maxRawRightPhi) maxRawRightPhi = rawRightPhi;
    if (rawRightL > maxRawRightL) maxRawRightL = rawRightL;

	if (this->debugLevel > 1) {
    	std::cout << getCalibrationString() << std::endl;
	}
  }

  bool GameTrak::isCalibrating() {
    return calibrating;
  }

  void
  GameTrak::idle(int milliseconds) {
#ifdef WIN32
    Sleep(milliseconds) ;
#endif
#ifdef __linux__
    usleep(milliseconds*1000) ;
#endif
#ifdef __APPLE__
    CFRunLoopRunInMode(kCFRunLoopDefaultMode, milliseconds/1000.0, false) ;
#endif 
  }

}
