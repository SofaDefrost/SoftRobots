/* -*- mode: c++ -*-
 *
 * pointing/utils/TimeStamp.h --
 *
 * Initial software
 * Authors: Nicolas Roussel
 * Copyright © INRIA
 *
 * http://libpointing.org/
 *
 * This software may be used and distributed according to the terms of
 * the GNU General Public License version 2 or any later version.
 *
 */

#ifndef TimeStamp_h
#define TimeStamp_h

#include <string>

#include <stdint.h>

namespace gametrak {

  class TimeStamp {

  public:

    /**
     * This class is used to represent points in time.
     * 
     * The date+time information is stored as a single 64-bit integer
     * with nanosecond precision, which makes it easy to compute
     * durations. It can also be converted to/from a UTC ISO 8601
     * string-based representation.
     * 
     * TimeStamps are described as the number of nanoseconds elapsed
     * since 1970-01-01T00:00:00.000Z. The minimum date+time that can be
     * represented is 1901-12-13T20:45:52Z. The maximum one is
     * 2038-01-19T03:14:07Z. These values were chosen to remain
     * compatible with POSIX 32-bit based time_t representations.
     *
     * Useful links: 
     *     - http://en.wikipedia.org/wiki/Unix_time
     *     - http://en.wikipedia.org/wiki/ISO_8601
     *
     */

    typedef int64_t inttime ;

    static const inttime one_week ;
    static const inttime one_day ;
    static const inttime one_hour ;
    static const inttime one_minute ;
    static const inttime one_second ;
    static const inttime one_millisecond ;
    static const inttime one_microsecond ;
    static const inttime one_nanosecond ;

    static const inttime undef, min, max ; // undef < min < max

  protected:

    inttime t ;

    static inttime now(void) ;

    static inttime string2int(std::string s) ;
    static std::string int2string(inttime i) ;

    static inttime ext2int(int year, int month, int day, 
			   int hour, int min, int sec, int msec) ;
    static void int2ext(inttime t, 
			int *year, int *month, int *day, 
			int *hour, int *min, int *sec, int *msec, 
			int *wday, int *yday, bool utc) ;

  public:

    TimeStamp(void) : t(now()) {}
    TimeStamp(TimeStamp &s) : t(s.t) {}

    // All the specified times are assumed to be UTC-based
    TimeStamp(inttime ms) ;
    TimeStamp(std::string s) : t(string2int(s)) {}

    TimeStamp(int year, int month, int day, 
	      int hour, int min, int sec, int msec) : 
      t(ext2int(year, month, day, hour, min, sec, msec)) {}

    bool operator<(const TimeStamp &other) const {
      return (t<other.t) ;
    }

    bool operator>(const TimeStamp &other) const {
      return (t>other.t) ;
    }

    TimeStamp::inttime operator=(const TimeStamp::inttime &t) ;

    bool operator==(const TimeStamp &other) const {
      return (t==other.t) ;
    }

    bool operator==(const TimeStamp::inttime &t) const {
      return (this->t==t) ;
    }

    TimeStamp::inttime operator-(const TimeStamp &other) const {
      return t-other.t ;
    }

    TimeStamp::inttime operator+(const TimeStamp &other) const {
      return t+other.t ;
    }

    TimeStamp::inttime operator-(const TimeStamp::inttime &t) const {
      return this->t-t ;
    }

    TimeStamp::inttime operator+(const TimeStamp::inttime &t) const {
      return this->t+t ;
    }

    void refresh(void) {
      t = now() ;
    }

    // the number of nanoseconds since 1970-01-01T00:00:00.000Z
    inttime getAsInt(void) {
      return t ; 
    }

    // a UTC ISO 8601 combined data/time string (e.g. 2006-02-25T11:59:12.113Z)
    std::string getAsString(void) {
      return int2string(t) ; 
    } 

    // Use 0 for components you don't want
    void getAsUTCTime(int *year, int *month, int *day,
		      int *hour, int *min, int *sec, int *msec,
		      int *wday=0, int *yday=0) {
      return int2ext(t, 
		     year, month, day, 
		     hour, min, sec, msec, 
		     wday, yday,
		     true) ;
    }

    // Use 0 for components you don't want
    void getAsLocalTime(int *year, int *month, int *day,
			int *hour, int *min, int *sec, int *msec,
			int *wday=0, int *yday=0) {
      return int2ext(t, 
		     year, month, day, hour, min, sec, msec, 
		     wday, yday, 
		     false) ;
    }

    static inttime getLocalUTCOffset(void) ;

    static inttime createAsInt(void) { return now() ; }
    static inttime createAsIntFrom(std::string s) { return string2int(s) ; }
    static inttime createAsIntFrom(int year, int month, int day, int hour, int min, int sec, int msec) { return ext2int(year,month,day,hour,min,sec,msec) ; }

    static std::string createAsString(void) { return int2string(now()) ; }
    static std::string createAsStringFrom(inttime i) { return int2string(i) ; }
    static std::string createAsStringFrom(int year, int month, int day, int hour, int min, int sec, int msec) { return int2string(ext2int(year,month,day,hour,min,sec,msec)) ; }

  } ;

}

#endif
