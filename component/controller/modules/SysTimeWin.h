#ifndef SYSTIMEWIN_H
#define SYSTIMEWIN_H

#include <sys/timeb.h>
#include <time.h>
#include <Windows.h>

int gettimeofday (struct timeval *tp, void *tz);

#endif