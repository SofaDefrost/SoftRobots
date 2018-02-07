#ifndef SYSTIMEWIN_H
#define SYSTIMEWIN_H

#include <sys/timeb.h>
#include <time.h>
#include <WinSock2.h>
#include <Windows.h>
#include <zmq.hpp>

int gettimeofday (struct timeval *tp, void *tz);

#endif