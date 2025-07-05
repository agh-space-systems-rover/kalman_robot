#ifndef _COMPASS
#define _COMPASS

#include "compasscal.h"
#include <stdio.h>
#include <libphidget22/phidget22.h>

#ifdef WIN32
#include <Windows.h>
#define SLEEP(dlay) Sleep(dlay);
#else
#include <unistd.h>
#define SLEEP(dlay) usleep(dlay*1000);
#endif

#endif
