/* -*- mode: c++ -*-
 *
 * libgametrak/GameTrak.h --
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

#ifndef GameTrak_h
#define GameTrak_h

#include <sstream>
#include <iomanip>
#include "utils/Quaternion.h"
#include "utils/TimeStamp.h"
#include "utils/URI.h"
#include "utils/Quaternion.h"
#include "utils/OneEuroFilter.h"
#include <stdexcept>

#ifdef WIN32
    // TODO
#else
    #include <pthread.h>
    static pthread_mutex_t mutex;
#endif

namespace gametrak {

  class GameTrak {

  protected:
    int debugLevel;
    // Raw values from encoders
    double rawLeftTheta, rawLeftPhi, rawLeftL;
    double rawRightTheta, rawRightPhi, rawRightL;

    bool button;

    // OneEuroFilter settings
    bool filteringEnabled;
    double mincutoff;
    double beta;
    double dcutoff;

    OneEuroFilter *filterLeftTheta, *filterLeftPhi, *filterLeftL;
    OneEuroFilter *filterRightTheta, *filterRightPhi, *filterRightL;

    // Raw values from encoders after filtering
    double rawLeftThetaf, rawLeftPhif, rawLeftLf;
    double rawRightThetaf, rawRightPhif, rawRightLf;

    // calibration values
    bool useCalibration;
    bool calibrating;
    bool calibrated;

    double minRawLeftTheta, minRawLeftPhi, minRawLeftL;
    double minRawRightTheta, minRawRightPhi, minRawRightL; 

    double maxRawLeftTheta, maxRawLeftPhi, maxRawLeftL;
    double maxRawRightTheta, maxRawRightPhi, maxRawRightL; 

    // Metric values
    double LeftTheta; // degrees
    double LeftPhi; // degrees
    double LeftL; // mm

    double RightTheta; // degrees
    double RightPhi; // degrees
    double RightL; // mm

    // String ends positions
    double LeftX, LeftY, LeftZ;
    double RightX, RightY, RightZ;

    // Last synchronized Gametrak Data
    TimeStamp::inttime sync_timeStamp;
    double sync_LeftX, sync_LeftY, sync_LeftZ;
    double sync_RightX, sync_RightY, sync_RightZ;
    bool   sync_button;

    GameTrak(void) ;

    void FilterRawvalues(double timestamp) ;

    void calibrate() ;

    Vecteur3D Transform(double Theta, double Phi, double L) ;

    bool gametrakConnected;
    int nbOfTryMax;

    bool pullMode;

  public:

    typedef void (*GameTrakCallback)(void *context, 
				     TimeStamp::inttime timestamp, 
                    double leftx, double lefty, double leftz,
				     double rightx, double righty, double rightz,
				     bool button) ;

    static GameTrak *create(const char *device_uri=0) ;
    static GameTrak *create(std::string device_uri) ;

    static void idle(int milliseconds) ;

    bool isGametrakConnected() { return gametrakConnected; }

    bool isInPullMode() { return pullMode; }
    void getGametrakData(TimeStamp::inttime *ts,
                         double *lX, double *lY, double *lZ,
                         double *rX, double *rY, double *rZ,
                         bool *but);
    void getLeftCursor(TimeStamp::inttime *ts, double *lX, double *lY, double *lZ);
    void getRightCursor(TimeStamp::inttime *ts, double *rX, double *rY, double *rZ);
    void getButton(TimeStamp::inttime *ts, bool *but);

    void enterCalibration(void) ;
    std::string leaveCalibration(void) ;
    bool isCalibrating(void) ;
    std::string getCalibrationString() ;
    bool toggleCalibration(void) {useCalibration = !useCalibration; return useCalibration ;} ;

    virtual bool isActive(void) const {return true; } ;

    virtual URI getURI(bool expanded=false) const = 0 ;

    virtual void enableFiltering(void) {filteringEnabled = true;} ;
    virtual void disableFiltering(void) {filteringEnabled = false;} ;
    virtual bool toggleFiltering(void) {filteringEnabled = !filteringEnabled; return filteringEnabled ;} ;

    // http://www.lifl.fr/~casiez/1euro/
    virtual void tuneFiltering(double mincutoff, double beta) ;

    virtual void setGameTrakCallback(GameTrakCallback callback, void *context=0) = 0 ;

    virtual void setDebugLevel(int /*level*/) {}
    virtual void debug(std::ostream& /*out*/) const {}

    virtual ~GameTrak(void) {}

  } ;

}

#endif
