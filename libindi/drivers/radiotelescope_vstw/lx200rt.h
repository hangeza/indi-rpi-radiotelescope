/*
    LX200 Radio Telescope
    Copyright (C) 2009 HG Zaunick (zhg@gmx.de)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#ifndef LX200RT_H
#define LX200RT_H

#include "indidevapi.h"
#include "indicom.h"

#define	POLLMS		750		/* poll period, ms */
#define mydev		"LX200 RT" /* The device name */

class LX200RT
{
   public:
      LX200RT();
      virtual ~LX200RT();

      virtual void ISGetProperties (const char *dev);
      virtual void ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n);
      virtual void ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n);
      virtual void ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n);
      virtual void ISSnoopDevice (XMLEle *root);
      virtual void ISPoll();
      virtual void ISIdleTask();
      virtual void getBasicData();

      int checkPower(INumberVectorProperty *np);
      int checkPower(ISwitchVectorProperty *sp);
      int checkPower(ITextVectorProperty *tp);
      void handleError(ISwitchVectorProperty *svp, int err, const char *msg);
      void handleError(INumberVectorProperty *nvp, int err, const char *msg);
      void handleError(ITextVectorProperty *tvp, int err, const char *msg);
      bool isTelescopeOn(void);
      void connectTelescope();
      void slewError(int slewCode);
      void getAlignment();
      void handleAltAzSlew();
      int handleCoordSet();
      int getOnSwitch(ISwitchVectorProperty *sp);
      void setCurrentDeviceName(const char * devName);
      void correctFault();
      void enableSimulation(bool enable);
      void updateTime();
      void updateLocation();
      void mountSim();

      int fd;

   protected:
      int timeFormat;
      int currentSiteNum;
      int trackingMode;

      double JD;
      double lastRA;
      double lastDEC;
      bool   fault;
      bool   simulation;
      char   thisDevice[64];
      int    currentSet;
      int    lastSet;
};

void changeLX200RTDeviceName(const char * newName);
void changeAllDeviceNames(const char *newName);

#endif
