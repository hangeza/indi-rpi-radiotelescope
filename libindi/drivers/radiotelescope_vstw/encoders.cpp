#include <iostream>
#include <stdio.h>
#include <pigpio.h>

//#include <stdint.h>
#include <unistd.h>
//#include <stdlib.h>
//#include <getopt.h>

#include "encoders.h"
#include "gpioif.h"

using namespace std;

#define DEFAULT_VERBOSITY 1
#define DEFAULT_LD_GPIO 5
#define SPI_BAUD_DEFAULT 500000UL


encoder::fSPIHandle = -1;
encoder::fNrInstances = 0;

encoder::encoder(int spiChannel, int nrBitsPerRev, int nrBitsTurns) 
    : GPIO(), fChannel(spiChannel), fNrBitsPerRev(nrBitsPerRev), fNrBitsTurns(nrBitsTurns)
{
  init();
  fNrInstances++;
  fActiveLoop=false;
}

encoder::~encoder()
{
  fActiveLoop = false;
  if (fThread) fThread->join();
  delete fThread;
  fNrInstances--;
  // close SPI device if nobody else uses it
  if (!fNrInstances && fSPIHandle>=0) spiClose(fSPIHandle);
}


void encoder::init()
{
  if (fHandle<0) return;
  unsigned int spiFlags = 0;
  if (fSPIHandle<0) fSPIHandle=spiOpen(fChannel & 0x03,SPI_BAUD_DEFAULT, spiFlags);
    
    if (fSPIHandle < 0)
    {
      // spi open failed.
      //message("SPI open failed.", fVerbose, true);
    } else {
      // ok
      //message("SPI open ok",fVerbose);
      //fOpen = true;
      fThread = new std::thread( [this] { this->readLoop(); } );
      fActiveLoop=true;
    }
}

// this is the background thread loop
void encoder::readLoop()
{
  while (fActiveLoop) {
    uint32_t data;
    int nbytes=readDataWord(data);
    fPos = data>>8;
//     fTurns = data & ((1<<fNrBitsTurns)-1)
    usleep(100000L);
  }
}


bool encoder::read(char* buf, int nBytes)
{
    if (!isInitialized()) return false;
    if (!nBytes) return true;
    int res=spiRead(fSPIHandle, buf, nBytes);
    if (res>0) {
      //message("SPI read succeeded.",2);
      return true;
    }
    //message("SPI read failed",0,true);
    return false;
}

int encoder::readDataWord(uint32_t& data)
{
  if (!isInitialized()) return 0;
  const static int n = 4;
  char buf[n];
  bool ok=read(buf, n);
  if (!ok) return 0;
  data=0;
  for (int i=n-1; i>=0; i--)
    data|=((uint32_t)buf[i])<<((n-1-i)*8);
//     data|=((uint32_t)buf[i])<<(i*8);
  return n;
}



