#ifndef _COMPASS
#define _COMPASS

#include "../compasscal_lib/compasscal.h"
#include <stdio.h>
#include <phidget22.h>

#ifdef WIN32
#include <Windows.h>
#define SLEEP(dlay) Sleep(dlay);
#else
#include <unistd.h>
#define SLEEP(dlay) usleep(dlay*1000);
#endif

#endif
