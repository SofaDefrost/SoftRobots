/* -*- mode: c++ -*-
 *
 * pointing/utils/URI.cpp --
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

#include "URI.h"

#include <iostream>
#include <sstream>
#include <algorithm>
#include <cstdio>

namespace gametrak {

#ifdef __APPLE__
#include <xlocale.h>
#else
#define atof_l(str,loc) atof(str)
#endif

  static void
  split(std::string &s, const std::string splitters, std::string &part, bool keepsep, bool preserve) {
    std::string::size_type pos = s.find_first_of(splitters) ;
    if (pos!=std::string::npos) {    
      part.assign(s, 0, pos) ;
      s.erase(0, (keepsep ? pos : pos+1)) ;
    } else {
      if (!preserve) {
	part.assign(s) ;
	s = "" ;
      }
    }
  }

  // ----------------------------------------------------------------------

  URI::URI(const URI& src) {
    scheme = src.scheme ;
    opaque = src.opaque ;
    user = src.user ;
    password = src.password ;
    host = src.host ;
    path = src.path ;
    port = src.port ;
    query = src.query ;
    fragment = src.fragment ;
  }

  URI&
  URI::operator = (const URI& src) {
    if (&src!=this) {
      scheme = src.scheme ;
      opaque = src.opaque ;
      user = src.user ;
      password = src.password ;
      host = src.host ;
      path = src.path ;
      port = src.port ;
      query = src.query ;
      fragment = src.fragment ;
    }
    return *this ;
  }

  bool
  URI::operator==(const URI &other) const {
    if (scheme!=other.scheme) return false ;
    if (opaque!=other.opaque) return false ;
    if (user!=other.user) return false ;
    if (password!=other.password) return false ;
    if (host!=other.host) return false ;
    if (port!=other.port) return false ;
    if (path!=other.path) return false ;
    if (query!=other.query) return false ;
    if (fragment!=other.fragment) return false ;
    return true ;
  }

  bool
  URI::operator!=(const URI &other) const {
    return !(*this == other);
  }

  bool
  URI::resemble(const URI &other) const {
    if (scheme!=other.scheme) return false ;
    if (opaque!=other.opaque) return false ;
    if (host!=other.host) return false ;
    if (port!=other.port) return false ;
    if (path!=other.path) return false ;
    return true ;
  }

  void
  URI::clear(void) {
    scheme = "" ;
    opaque = "" ;
    user = password = host = path = "" ;
    port = 0 ;
    query = "" ;
    fragment = "" ;
  }

  void
  URI::load(const std::string &s) {
    clear() ;

    std::string uri = s ;
    split(uri, ":", scheme, false, true) ;

    bool isOpaque = false ;

    if ((uri.size()>0) && (uri[0]=='/')) {
      if (uri[1]=='/') {
	uri.erase(0,2) ;
	std::string site ;
	split(uri, "/?", site, true, false) ;
	if (site.length()) {
	  split(site, "@", password, false, true) ;
	  if (password.length())
	    split(password, ":", user, false, false) ;
	  split(site, ":", host, false, false) ;
	  port = atoi(site.c_str()) ;
	}
      } 
    } else
      isOpaque = (scheme.length()>0) ;

    if (isOpaque) {
      // Was: split(uri, "#", _opaque, false, false) ;
      split(uri, "?", opaque, false, false) ;
      split(uri, "#", query, false, false) ;
    } else {
      split(uri, "?", path, false, false) ;
      split(uri, "#", query, false, false) ;
    }
 
    fragment.assign(uri) ;
  }

  // ----------------------------------------------------------------------

  bool
  URI::isEmpty(void) const {
    return scheme.empty()
      && opaque.empty()
      && user.empty() && password.empty()
      && host.empty() && port==0
      && path.empty()
      && query.empty() && fragment.empty() ;
  }

  void
  URI::generalize(void) {
    user = "" ;
    password = "" ;
    query = "" ;
    fragment = "" ;
  }

  std::string
  URI::asString(void) const {
    std::stringstream tmp ;
    tmp << user ;
    if (password!="") tmp << ":" << password ;
    if (user!="" || password!="") tmp << "@" ;
    tmp << host ;
    if (port) tmp << ":" << port ;
    std::string authority = tmp.str() ;

    std::stringstream result ;
    if (scheme!="") result << scheme << ":" ;
    if (opaque!="") 
      result << opaque ;
    else {
      if (authority!="") result << "//" << authority ;
      if (path!="") result << path ;
      // if (query!="") result << "?" << query ; // rfc2396
    }
    if (query!="") result << "?" << query ; // rfc3986?
    if (fragment!="") result << "#" << fragment ;
    return result.str() ;
  }

  // ----------------------------------------------------------------------

  void
  URI::debug(std::ostream& out) const {
    out << "Kind     : " ;
    out << (scheme!="" ? "ABSOLUTE " : "RELATIVE ") ;
    if (opaque!="") out << "OPAQUE " ;
    out << std::endl ;

    if (scheme!="") 
      out << "Scheme   : " << scheme << std::endl ;

    if (opaque!="") 
      out << "OPAQUE   : " << opaque << std::endl ;
    else {
      out << "User     : " << user << std::endl ;
      out << "Password : " << password << std::endl ;
      out << "Host     : " << host << std::endl ;
      out << "Port     : " << port << std::endl ;
      out << "Path     : " << path << std::endl ;
    }

    out << "Query    : " << query << std::endl ;
    out << "Fragment : " << fragment << std::endl ;

    out.flush() ;
  }

  // ----------------------------------------------------------------------

  std::string
  URI::encode(const std::string &src, int flags) {
    static const char m[] = {
      //                  1                   2                   3
      //1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
      0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
      2,1,2,2,1,2,4,1,1,1,1,4,1,1,1,4,1,1,1,1,1,1,1,1,1,1,4,4,2,4,2,4,
      4,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,1,
      2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,0,
      0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
      0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
      0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
      0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
    };
    static const char* h = "0123456789ABCDEF";
    std::string result ;

    for( std::string::const_iterator i=src.begin();
	 i != src.end(); 
	 ++i ) {
      char c = *i ;
      if (m[(int)c]&flags)
	result.append(&c,1) ;
      else {
	char tmp[3] ;
	sprintf(tmp,"%%%c%c", h[c>>4], h[c&0x0F]) ;
	result.append(tmp,3) ;
      }
    }

    return result ;
  }

  // ----------------------------------------------------------------------

  inline static unsigned int
  hex2int(char *s) {
    unsigned int t = 0;
    for(int i=0; i<2; i++)
      if(s[i]>='0' && s[i]<='9')
	t = (t<<4)+(s[i]-'0');
      else if(s[i]>='a' && s[i]<='f')
	t = (t<<4)+(s[i]-'a'+10);
      else if(s[i]>='A' && s[i]<='F')
	t = (t<<4)+(s[i]-'A'+10);
      else
	break;
    return t ;
  }

  std::string
  URI::decode(const std::string &src) {
    std::string result ;

    for( std::string::const_iterator i=src.begin();
	 i != src.end(); 
	 ++i ) {
      char c = *i ;   
      if (c == '+') c = ' ' ;
      else if (c == '%') {
	char tmp[2] ;
	++i ; tmp[0] = *i ;
	++i ; tmp[1] = *i ;
	c = (char)hex2int(tmp) ;
      }
      result.append(&c,1) ;
    }

    return result ;
  }

  // ----------------------------------------------------------------------

  bool
  URI::getQueryArg(const std::string &q, const std::string &key, std::string *value) {
    if (q=="") return false ;

    std::string::size_type from=0 ;
    do {
      std::string::size_type b = q.find(key, from) ;
      if (b==std::string::npos) return false ;

      // found one candidate...
      std::string::size_type next = b+key.length() ;
      bool okLeft = (b==0 || q[b-1]=='&') ;
      bool okRight = (next==q.length() || q[next]=='=' || q[next]=='&') ;
      if (okLeft && okRight) {
	// candidate seems good...
	if (value) {
	  *value = "" ;
	  if (q[next]=='=' && next+1<q.length()) {
	    std::string::size_type e = q.find("&",next+1) ;
	    std::string::size_type n = (e==std::string::npos) ? (q.length()-next+1) : (e-next-1) ;
	    std::string result ;
	    result.assign(q, next+1, n) ;
	    *value = decode(result) ;
	  }
	}
	return true ;
      }
      from = next ;
    } while (from<q.length()) ;

    return false ;
  }

  bool
  URI::getQueryArg(const std::string &q, const std::string &key, bool *value) {
    std::string tmp ;
    if (!getQueryArg(q, key, &tmp)) return false ;
    std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::tolower) ;
    if (tmp=="" || tmp=="true")
      *value = true ;
    else if (tmp=="false")
      *value = false ;
    else 
      *value = (bool)atoi(tmp.c_str()) ;
    return value ;
  }

  bool
  URI::getQueryArg(const std::string &q, const std::string &key, int *value) {
    std::string tmp ;
    if (!getQueryArg(q, key, &tmp)) return false ;
    *value = atoi(tmp.c_str()) ;
    return true ;
  }

  bool
  URI::getQueryArg(const std::string &q, const std::string &key, unsigned int *value) {
    std::string tmp ;
    if (!getQueryArg(q, key, &tmp)) return false ;
    *value = (unsigned int)atoi(tmp.c_str()) ;
    return true ;
  }

  bool
  URI::getQueryArg(const std::string &q, const std::string &key, unsigned long *value) {
    std::string tmp ;
    if (!getQueryArg(q, key, &tmp)) return false ;
    *value = (unsigned long)strtol(tmp.c_str(), (char**)NULL,0) ;
    return true ;
  }

  bool
  URI::getQueryArg(const std::string &q, const std::string &key, double *value) {
    std::string tmp ;
    if (!getQueryArg(q, key, &tmp)) return false ;
    *value = (double)atof_l(tmp.c_str(), NULL) ;
    return true ;
  }

  bool
  URI::getQueryArg(const std::string &q, const std::string &key, float *value) {
    std::string tmp ;
    if (!getQueryArg(q, key, &tmp)) return false ;
    *value = atof_l(tmp.c_str(), NULL) ;
    return true ;
  }

}
