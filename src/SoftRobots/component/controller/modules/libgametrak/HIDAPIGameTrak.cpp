/* -*- mode: c++ -*-
 *
 * libgametrak/HIDAPIGameTrak.cpp --
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

#include "HIDAPIGameTrak.h"

#include <sstream>

namespace gametrak {


  HIDAPIGameTrak::HIDAPIGameTrak(URI uri):GameTrak() {

#ifdef WIN32
    // TODO
#else
    pthread_mutex_init(&mutex,NULL);
#endif

    URI::getQueryArg(uri.query, "debugLevel", &debugLevel) ;
    debugLevel = 1;

    serial_number = "";
    URI::getQueryArg(uri.query, "serial_number", &serial_number) ;

    devicePath = "";
    URI::getQueryArg(uri.query, "devicePath", &devicePath) ;

    nbOfTryMax = -1;
    URI::getQueryArg(uri.query, "nbOfTryMax", &nbOfTryMax) ;
    nbOfTryMax = 10;

    pullMode = false;
    URI::getQueryArg(uri.query, "pullMode", &pullMode);

    // Enumerate and print the HID devices on the system
    struct hid_device_info *devs, *cur_dev;
    
    //hid_init();
    devs = hid_enumerate(0x0, 0x0);
    cur_dev = devs; 
    while (cur_dev) {
      if ((debugLevel>0) && (cur_dev->vendor_id == 0x14B7) && (cur_dev->product_id == 0x0982)) {
        printf("Device Found\n  type: %04hx %04hx\n  path: %s\n  serial_number: %ls",
          cur_dev->vendor_id, cur_dev->product_id, cur_dev->path, cur_dev->serial_number);
        printf("\n");
        printf("  Manufacturer: %ls\n", cur_dev->manufacturer_string);
        printf("  Product:      %ls\n", cur_dev->product_string);
        printf("\n");
      }
      cur_dev = cur_dev->next;
    }
    hid_free_enumeration(devs);

    deviceConnected = false;

    connect();

    // Filtering
    URI::getQueryArg(uri.query, "filter", &filteringEnabled) ;
    URI::getQueryArg(uri.query, "mincutoff", &mincutoff) ;
    URI::getQueryArg(uri.query, "beta", &beta) ;
    URI::getQueryArg(uri.query, "dcutoff", &dcutoff) ;

    beta *= 2.;

    rawLeftThetafPrev = 0;
    rawLeftPhifPrev = 0;
    rawLeftLfPrev = 0;
    rawRightThetafPrev = 0;
    rawRightPhifPrev = 0;
    rawRightLfPrev = 0;

    callback = 0 ;
    callback_context = 0 ;

    if (URI::getQueryArg(uri.query, "milt", &minRawLeftTheta) &&
    URI::getQueryArg(uri.query, "milp", &minRawLeftPhi) &&
    URI::getQueryArg(uri.query, "mill", &minRawLeftL) &&
    URI::getQueryArg(uri.query, "mirt", &minRawRightTheta) &&
    URI::getQueryArg(uri.query, "mirp", &minRawRightPhi) &&
    URI::getQueryArg(uri.query, "mirl", &minRawRightL) &&

    URI::getQueryArg(uri.query, "malt", &maxRawLeftTheta) &&
    URI::getQueryArg(uri.query, "malp", &maxRawLeftPhi) &&
    URI::getQueryArg(uri.query, "mall", &maxRawLeftL) &&
    URI::getQueryArg(uri.query, "mart", &maxRawRightTheta) &&
    URI::getQueryArg(uri.query, "marp", &maxRawRightPhi) &&
    URI::getQueryArg(uri.query, "marl", &maxRawRightL)) {
    	calibrated = true;
    }

    URI::getQueryArg(uri.query, "useCalibration", &useCalibration) ;

    run = true;
    threadFinished = false;

#ifdef WIN32
  hThreads[0]=CreateThread(NULL, NULL, eventloop, LPVOID(this), 0, &dwThreadId);
#else
    int ret = pthread_create(&thread, NULL, eventloop, (void*)this) ;
    if (ret<0) {
      perror("HIDAPIGameTrak::HIDAPIGameTrak") ;
      throw std::runtime_error("HIDAPIGameTrak: pthread_create failed") ;    
    }
#endif
  }

/*bool HIDAPIGameTrak::isActive(void) const {
  return deviceConnected ;
}*/

void HIDAPIGameTrak::connect() {
    int nbOfTry = 1;
    gametrakConnected = true;
    // Pictrak mod: http://janoc.rd-h.com/archives/129
    pictrak = false;

    while (!deviceConnected) {
      if ((nbOfTryMax != -1) && (nbOfTry > nbOfTryMax)) {
          gametrakConnected = false;
          std::cout << "Warning : /!\\ Number of \"try to connect\" max was reached /!\\" << std::endl ;
          std::cout << "        ( Hint : Either no Gametraks are physically connected  |" << std::endl ;
          std::cout << "        |        or the the \"nbOfTryMax\" argument in the       |" << std::endl ;
          std::cout << "        |        URI is too small...                           )" << std::endl ;
          return;
      }
      nbOfTry++;

      // Open the device using the VID, PID,
      // and optionally the Serial number.
      try {
          std::cout << "Connecting..." << std::endl;

          if (devicePath != "")
          {
              // Open the device using device path
              handle = hid_open_path(devicePath.c_str());
              if (handle != NULL) deviceConnected = true;
          }
          else if (serial_number != "")
          {
              std::wstring widestr = std::wstring(serial_number.begin(), serial_number.end());
              handle = hid_open(0x14B7, 0x0982, (widestr.c_str()));
              if (handle != NULL) deviceConnected = true;
          }
          else
          {
              handle = hid_open(0x14B7, 0x0982, NULL);
              if (handle != NULL) deviceConnected = true;
          }

          if (handle == NULL) {
              throw std::runtime_error("HIDAPIGameTrak: pb opening GameTrak") ;
          }

          // Check if we have a PicTrak
          if (handle != NULL) {
              wchar_t product[255];
              //int res =
              hid_get_product_string(handle, product, 255);
              if (wcscmp(product,L"GameTrak PIC") == 0)
                  pictrak = true;
          }
      } catch (std::exception e)
      {
          if (debugLevel > 0) std::cout << "Retrying to connect..." << std::endl;
      }
#ifdef WIN32
      Sleep(200) ;
#else
      usleep(200000);
#endif
     
    }
}

void HIDAPIGameTrak::disconnect() {

	if (handle != NULL && deviceConnected) {
		hid_close(handle);
	}

}

#ifdef WIN32
DWORD WINAPI HIDAPIGameTrak::eventloop(LPVOID context)
{
#else
  void *
  HIDAPIGameTrak::eventloop(void *context) {
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL) ;
    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL) ;
#endif

    HIDAPIGameTrak *self = (HIDAPIGameTrak*)context ;

    unsigned char buf[16];

    int res = 0;

    while (self->run) {
      if (self->deviceConnected) {
        try {
          // Read requested state
          res = hid_read(self->handle, buf, 16);
          if (res < 0)
            throw std::runtime_error("HIDAPIGameTrak: hid_read error") ;
        } catch (std::exception e) {
          self->deviceConnected = false;
          self->handle = NULL;
          if (self->debugLevel > 0) std::cout << "Trying to reconnect ..." << std::endl;
          self->connect();
        }

        if (self->debugLevel > 2) {
          std::cout << "Raw buffer = ";
          for (int i=0; i<16; i++) std::cout << (int)buf[i] << " ";
          std::cout << std::endl;
        }

        if (self->pictrak)
          self->rawLeftTheta = ((buf[1] << 8) + buf[0]);
        else
          self->rawLeftTheta = (buf[15] << 8) + buf[14];
        self->rawLeftPhi = (buf[3] << 8) + buf[2];
        self->rawLeftL = (buf[5] << 8) + buf[4];

        self->rawRightTheta = (buf[7] << 8) + buf[6];
        self->rawRightPhi = (buf[9] << 8) + buf[8];
        self->rawRightL = (buf[11] << 8) + buf[10];

        bool button;
        if (self->pictrak)
          button = buf[12] == 1;
        else
          button = buf[12] == 16;

        if (self->debugLevel > 1) 
          std::cout << "LeftRawTheta=" << std::setw(3) << self->rawLeftTheta
            << " LeftRawPhi=" << std::setw(3) << self->rawLeftPhi
            << " LeftRawL=" << std::setw(3) << self->rawLeftL
            << " RightRawTheta=" << std::setw(3) << self->rawRightTheta
            << " RightRawPhi=" << std::setw(3) << self->rawRightPhi
            << " RightRawL=" << std::setw(3) << self->rawRightL << std::endl;


        TimeStamp::inttime now = TimeStamp::createAsInt();

        bool send = true; 
        if (self->filteringEnabled) {
          self->FilterRawvalues(now * 1.0E-9);

          // If position changed then call the callback

          send = false;
          if ((floor(self->rawLeftThetaf) != self->rawLeftThetafPrev) ||
              (floor(self->rawLeftPhif) != self->rawLeftPhifPrev) ||
              (floor(self->rawLeftLf) != self->rawLeftLfPrev) ||
              (floor(self->rawRightThetaf) != self->rawRightThetafPrev) ||
              (floor(self->rawRightPhif) != self->rawRightPhifPrev) ||
              (floor(self->rawRightLf) != self->rawRightLfPrev)) {
              send = true;
           } 

          self->rawLeftThetafPrev = floor(self->rawLeftThetaf);
          self->rawLeftPhifPrev = floor(self->rawLeftPhif);
          self->rawLeftLfPrev = floor(self->rawLeftLf);
          self->rawRightThetafPrev = floor(self->rawRightThetaf);
          self->rawRightPhifPrev = floor(self->rawRightPhif);
          self->rawRightLfPrev = floor(self->rawRightLf);
        } else {
          self->rawLeftThetaf = self->rawLeftTheta;
          self->rawLeftPhif = self->rawLeftPhi;
          self->rawLeftLf =self->rawLeftL;
          self->rawRightThetaf = self->rawRightTheta;
          self->rawRightPhif = self->rawRightPhi;
          self->rawRightLf = self->rawRightL;
        }     

        if (self->calibrating) {
          self->calibrate();
        }


        // Metric values
        double angleMax = 34.7; // degrees - measured (default value was 30.0)
        double stringLength = 3065.0; // mm - measured (default value was 3000.0)
        double distance2strings = 130.0; // mm - measured (default value was 100.0)
        double stringOffset = 35.0; // mm - measured
        double mmPerRawVal;
        if (self->pictrak)
           mmPerRawVal = 200.0/186.0; // measured on one pictrak only
        else
           mmPerRawVal = 200.0/150.0; // measured on one normal gt only

        if (self->useCalibration) {// && self->calibrated) {
          //double mid = 4096.0/2.0;

          double midLeftTheta = (self->maxRawLeftTheta - self->minRawLeftTheta)/2+self->minRawLeftTheta;
          if (self->pictrak)
	          self->LeftTheta = -(self->rawLeftThetaf - midLeftTheta) * angleMax / (midLeftTheta-self->minRawLeftTheta);
	      else
	          self->LeftTheta = (self->rawLeftThetaf - midLeftTheta) * angleMax / (midLeftTheta-self->minRawLeftTheta);
          double midLeftPhi = (self->maxRawLeftPhi - self->minRawLeftPhi)/2+self->minRawLeftPhi;
          self->LeftPhi = -(self->rawLeftPhif - midLeftPhi) * angleMax / (midLeftPhi-self->minRawLeftPhi);
          self->LeftL = stringOffset+mmPerRawVal*(self->maxRawLeftL-self->rawLeftLf);//((stringLength-stringOffset)/(self->maxRawLeftL - self->minRawLeftL)) * (self->maxRawLeftL-self->rawLeftLf);

          double midRightTheta = (self->maxRawRightTheta - self->minRawRightTheta)/2+self->minRawRightTheta;
          self->RightTheta = -(self->rawRightThetaf - midRightTheta) * angleMax / (midRightTheta-self->minRawRightTheta);
          double midRightPhi = (self->maxRawRightPhi - self->minRawRightPhi)/2+self->minRawRightPhi;
          self->RightPhi = -(self->rawRightPhif - midRightPhi) * angleMax / (midRightPhi-self->minRawRightPhi);
          self->RightL = stringOffset+mmPerRawVal*(self->maxRawRightL-self->rawRightLf);//stringOffset+((stringLength-stringOffset)/(self->maxRawRightL - self->minRawRightL)) * (self->maxRawRightL-self->rawRightLf);
          /*double midRightTheta = (self->maxRawRightTheta - self->minRawRightTheta);
          self->RightTheta = -(self->rawRightThetaf - midRightTheta) * angleMax / midRightTheta;
          double midRightPhi = (self->maxRawRightPhi - self->minRawRightPhi);
          self->RightPhi = -(self->rawRightPhif - midRightPhi) * angleMax / midRightPhi;
          self->RightL = - stringLength/(self->maxRawRightL - self->minRawRightL) * self->rawRightLf + stringLength; */
        } else { 
          double mid = 4096.0/2.0;

          if (self->pictrak)
            self->LeftTheta = -(self->rawLeftThetaf - mid) * angleMax / mid;
          else
            self->LeftTheta = (self->rawLeftThetaf - mid) * angleMax / mid;
          
          self->LeftPhi = -(self->rawLeftPhif - mid) * angleMax / mid;
          self->LeftL = - stringLength/4096.0 * self->rawLeftLf + stringLength;

          self->RightTheta = -(self->rawRightThetaf - mid) * angleMax / mid;
          self->RightPhi = -(self->rawRightPhif - mid) * angleMax / mid;
          self->RightL = - stringLength/4096.0 * self->rawRightLf + stringLength; 
        }

        if (self->debugLevel > 1) 
          std::cout << "LeftTheta=" << std::setw(3) << self->LeftTheta
            << " LeftPhi=" << std::setw(3) << self->LeftPhi
            << " LeftL=" << std::setw(3) << self->LeftL
            << " RightTheta=" << std::setw(3) << self->RightTheta
            << " RightPhi=" << std::setw(3) << self->RightPhi
            << " RightL=" << std::setw(3) << self->RightL << std::endl;

        Vecteur3D LeftHand = self->Transform(self->LeftTheta * M_PI / 180.0, self->LeftPhi * M_PI / 180.0, self->LeftL);
        self->LeftX = LeftHand.x - distance2strings / 2.0;
        self->LeftY = LeftHand.y;
        self->LeftZ = LeftHand.z;

        Vecteur3D RightHand = self->Transform(self->RightTheta * M_PI / 180.0, self->RightPhi * M_PI / 180.0, self->RightL);
        self->RightX = RightHand.x + distance2strings / 2.0;
        self->RightY = RightHand.y;
        self->RightZ = RightHand.z;

#ifdef WIN32
        // TODO
#else
        pthread_mutex_lock(&mutex);
#endif
        self->sync_timeStamp = now;
        self->sync_button = button;
        self->sync_LeftX = self->LeftX;
        self->sync_LeftY = self->LeftY;
        self->sync_LeftZ = self->LeftZ;
        self->sync_RightX = self->RightX;
        self->sync_RightY = self->RightY;
        self->sync_RightZ = self->RightZ;
//        std::cout<<self->LeftX<<std::endl;
#ifdef WIN32
        // TODO
#else
        pthread_mutex_unlock(&mutex);
#endif

        // bool send = true;
        // if (send && (self->callback != 0))
        //   self->callback(self->callback_context, now, floor(self->rawLeftThetaf), floor(self->rawLeftPhif), floor(self->rawLeftLf), floor(self->rawRightThetaf), floor(self->rawRightPhif), floor(self->rawRightLf), button);

        // bool send = true;
        // if (send && (self->callback != 0))
        //   self->callback(self->callback_context, now, self->LeftTheta, self->LeftPhi, self->LeftL, self->RightTheta, self->RightPhi, self->RightL, button);

        if (!self->pullMode && send && (self->callback != 0)) 
          self->callback(self->callback_context, now, self->LeftX, self->LeftY, self->LeftZ, self->RightX, self->RightY, self->RightZ, button);
      } else {
#ifdef WIN32
		Sleep(100) ;
#else
		usleep(100000);
#endif
      }
    }

		self->disconnect();
        self->threadFinished = true;

    return 0 ;
  }

  void
  HIDAPIGameTrak::setGameTrakCallback(GameTrakCallback cbck, void *ctx) {
    callback = cbck ;
    callback_context = ctx ;
  }


  URI
  HIDAPIGameTrak::getURI(bool /*expanded*/) const {
    URI uri ;
    uri.scheme = "hidapigt" ;
    // int i = 0 ;
    // std::stringstream q ;
    // if (expanded || hz!=DUMMY_DEFAULT_HZ) 
    //   q << (i++?"&":"") << "hz=" << hz ;
    // if (expanded || cpi!=DUMMY_DEFAULT_CPI) 
    //   q << (i++?"&":"") << "cpi=" << cpi ;
    // uri.query = q.str() ;
    return uri ;
  }

  HIDAPIGameTrak::~HIDAPIGameTrak() {
    run = false;
#ifdef WIN32
    WaitForSingleObject(hThreads[0], INFINITE);
#else
    while (!threadFinished)
        usleep(100000);
#endif
  }

}
