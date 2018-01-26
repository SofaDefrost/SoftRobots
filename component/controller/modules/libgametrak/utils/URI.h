/* -*- mode: c++ -*-
 *
 * pointing/utils/URI.h --
 *
 * Initial software
 * Authors: Nicolas Roussel
 * Copyright © Inria
 *
 * http://libpointing.org/
 *
 * This software may be used and distributed according to the terms of
 * the GNU General Public License version 2 or any later version.
 *
 */

#ifndef URI_h
#define URI_h

/*

  Based on:
  http://www.ics.uci.edu/pub/ietf/uri/rfc2396.txt

  FIXME: Should be updated to:
  http://labs.apache.org/webarch/uri/rfc/rfc3986.html

  See also:
  http://www.ics.uci.edu/~fielding/
  http://www.ics.uci.edu/~fielding/url/

*/

#include <string>

namespace gametrak {

  struct URI {

    std::string scheme ;

    std::string opaque ;

    // If not opaque
    std::string user, password ;
    std::string host ;
    int port ;
    std::string path ;

    std::string query ;
    std::string fragment ;
 
  public:

    typedef enum {
      NONE = 0,
      NORMAL = 1,
      UNSAFE = 2,
      RESERVED = 4
    } URIENCODING ;

    static std::string encode(const std::string &src, int flags = NORMAL) ;
    static std::string decode(const std::string &src) ;

    static bool getQueryArg(const std::string &q, const std::string &key, std::string *value=0) ;
    static bool getQueryArg(const std::string &q, const std::string &key, bool *value) ;
    static bool getQueryArg(const std::string &q, const std::string &key, int *value) ;
    static bool getQueryArg(const std::string &q, const std::string &key, unsigned int *value) ;
    static bool getQueryArg(const std::string &q, const std::string &key, unsigned long *value) ;
    static bool getQueryArg(const std::string &q, const std::string &key, double *value) ;
    static bool getQueryArg(const std::string &q, const std::string &key, float *value) ;

    // --------------------------------------------------------------------------

    URI(void) : port(0) {}
    URI(const URI& src) ;
    URI(const std::string& s) { load(s) ; }
    URI(const char *s) { if (s) load(s) ; }

    URI& operator = (const URI& src) ;

    bool operator ==(const URI &other) const ;
    bool operator !=(const URI &other) const ;

    // Compares only scheme, opaque, host, port and path
    bool resemble(const URI &other) const ;

    void clear(void) ;
    void load(const std::string &uri) ;

    bool isEmpty(void) const ;

    void generalize(void) ;

    std::string asString(void) const ;

    void debug(std::ostream& out) const ;

  } ;

}

#endif
