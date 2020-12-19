#include <iostream>
#include <stdio.h>
#include <pigpio.h>

//#include <stdint.h>
#include <unistd.h>
//#include <stdlib.h>
//#include <getopt.h>

#include "gpioif.h"

using namespace std;

#define DEFAULT_VERBOSITY 1

GPIO::fhandle = -1;

GPIO::GPIO {
    if (fHandle<0) fHandle=gpioInitialise();
}


GPIO::~GPIO {
    if (fhandle>=0) gpioTerminate();
    fHandle = -1;
}
