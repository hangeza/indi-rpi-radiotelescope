#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>

#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <syslog.h>

//#include "astro.h"
#include "rttask.h"


using namespace std;
using namespace hgz;


const string _cmd_driftscan = "./rt_transitscan %f %f %s";
const string _cmd_tracking = "./rt_track %f %f %s";
const string _cmd_horscan = "./rt_scan_hor %f %f %f %f %s";
const string _cmd_equscan = "./rt_scan_equ %f %f %f %f %s";


template <class T>
std::string to_string(T t, std::ios_base & (*f)(std::ios_base&))
{
  std::ostringstream oss;
  oss << f << t;
  return oss.str();
}


//
// RTTask
//

int RTTask::fNumTasks=0;
bool RTTask::fAnyActive=false;
std::string RTTask::fDataPath="/daten/";
std::string RTTask::fExecutablePath="";


RTTask::RTTask()
{
   fId=0;
	fNumTasks++;
}

RTTask::~RTTask()
{
	fNumTasks--;
}


int RTTask::RunShellCommand(const char *strCommand)
{
	int iForkId, iStatus;
	iForkId = vfork();
	if (iForkId == 0)       // This is the child
	{
		int setsidstatus=setsid();
		//int pgidstatus=setpgid(getpid(),0);
		syslog (LOG_DEBUG, "setsid() status= %d", setsidstatus);
		//syslog (LOG_DEBUG, "changed PGID to %d (status=%d)", getpid(), pgidstatus);
		syslog (LOG_DEBUG, "running shell command: %s", strCommand);
		iStatus = execl("/bin/sh","sh","-c", strCommand, (char*) NULL);
		exit(iStatus);	// We must exit here,
							// or we will have multiple
							// mainlines running...
	}
	else if (iForkId > 0)   // Parent, no error
	{
		// return pid of child process
		iStatus = iForkId;
		// wait 10ms to be sure that the parent exited
		usleep(10000);
	}
	else    // Parent, with error (iForkId == -1)
	{
		iStatus = -1;
	}
	return(iStatus);
}


int RTTask::Start()
{
	if (fState==FINISHED) return (int)FINISHED;
	if (fState==ACTIVE) return (int)ACTIVE;
	if (fState==CANCELLED) return (int)CANCELLED;
	if (fAnyActive) { return (fState=WAITING); }
	fState=ACTIVE;
	fAnyActive=true;
	fStartTime=Time::Now();
	return 0;
}

int RTTask::Stop()
{
	if (fState==FINISHED) return (int)FINISHED;
	if (fState==ACTIVE) {
		for (int i=0; i<fPIDList.size(); i++) {
			int iDeadId=0;
			while (iDeadId==0) {
//				int iStatus=kill(fPIDList[i], SIGKILL);
				// kill all child processes in process group ID = PID of child
				int iStatus=kill(-fPIDList[i], SIGKILL);
				usleep(10000);
				int iChildiStatus;
				iDeadId = waitpid(fPIDList[i], &iChildiStatus, WNOHANG);
				if (iDeadId>0) {
					syslog (LOG_INFO, "stopping child processes with PGID %d, kill=%d", fPIDList[i], iStatus);
					//result=0;
				}
				else if (iDeadId<0) {
					syslog (LOG_ERR, "failed to stop child processes with PGID %d; kill=%d, waitpid=%d", fPIDList[i], iStatus, iDeadId);
					//result=-1;
				}
			}
		}
		fPIDList.clear();
		fState=STOPPED;
		fAnyActive=false;
	} else if (fState==CANCELLED || fState==STOPPED || fState==ERROR) {
		return fState;
	} else fState=STOPPED;
	return 0;
}

int RTTask::Cancel()
{
	int result=Stop();
	fState=CANCELLED;
	return result;
}

void RTTask::Print() const
{
   std::cout<<"RT Task:\n";
   std::cout<<setfill('0');
   std::cout<<" id    : "<<setw(8)<<hex<<fId<<dec<<std::endl;
}

void RTTask::Process()
{
	if (fState==FINISHED || fState==STOPPED || fState==CANCELLED) return;
	if (fVerbose>4) cout<<"RTTask::Process()"<<endl;
	if (fScheduleTime.timestamp()-Time::Now().timestamp()<0. &&
	    (fScheduleTime.timestamp()+fMaxRunTime*3600.-Time::Now().timestamp())>0.) {
		if (fState==WAITING) {
			if (!fAnyActive) Start();
		} else
		if (fState!=ACTIVE) {
			if (fVerbose>3) cout<<"RTTask::Process(): started task"<<endl;
			Start();
		}
	} else {
		if (fState==WAITING) {
			RTTask::Stop();	// need to call the base-class method only
									// since the measurement process never took place
			fState=CANCELLED;
		}
	}
	if (fState==ACTIVE) {
		// Wait till the commands complete
		int iChildiStatus = 0;
		int iDeadId = waitpid(-1, &iChildiStatus, WNOHANG);
		if (iDeadId < 0)
		{
			// Wait id error
		}
		else if (iDeadId > 0)
		{
			// child process finished, so just mark the task as finished
			fPIDList.clear();
			fAnyActive=false;
			//RTTask::Stop();	// need to call the base-class method only
									// since the measurement process finished alone
			fState=FINISHED;
			return;
		}
		else  // iDeadId == 0, no processes died
		{}

		if ((fElapsedTime=(Time::Now().timestamp()-fStartTime.timestamp())/3600.)>fMaxRunTime) {
			// max. runtime constraint fulfilled; stop the measurement by force
			Stop();
			if (fVerbose>3) cout<<"RTTask::Process(): stopped task"<<endl;
			fState=FINISHED;
			//fElapsedTime=fMaxRunTime;
		}
	}
}


//
// DriftScanTask
//

int DriftScanTask::Start()
{
	if (fVerbose>3) cout<<"DriftScanTask::Start()"<<endl;
	int result=RTTask::Start();
	if (result==0) {
		// hier code, um messung auszuführen
//		int returnvalue = system ("ls -la > doof &");

		string cmdstring;
		char tmpstr[256];
		char datafilestr[256];
		sprintf(datafilestr,"task_drift%04d%02d%02d_%05d",fStartTime.year(),fStartTime.month(),fStartTime.day(),(long)fStartTime.timestamp()%86400L);
		fDataFile=string(datafilestr);
//		fDataFile="task_drift"+to_string<long>((long)fStartTime.timestamp(), std::dec);

		cmdstring="cd "+fExecutablePath+" && ";
		sprintf(tmpstr, string(_cmd_driftscan).c_str(), (float)fStartCoords.Phi(),
(float)fStartCoords.Theta(), string(fDataPath+"/"+fDataFile).c_str() );
		cmdstring+=tmpstr;
#if __cplusplus > 199711L
		if (std::isnormal(fIntTime)) cmdstring+=" "+to_string<int>(fIntTime, std::dec);
#else
		if (std::isnormal<double>(fIntTime)) cmdstring+=" "+to_string<int>(fIntTime, std::dec);
#endif		
		syslog (LOG_DEBUG, "executing command: %s", cmdstring.c_str());
		int iStatus = RunShellCommand(cmdstring.c_str());
//		int iStatus = RunShellCommand("sleep 1000");
//		int iStatus = RunShellCommand(string("cd "+fExecutablePath+" && rt_doof").c_str());
		if (iStatus>0) {
			syslog (LOG_NOTICE, "starting driftscan task with id=%d (pid %d)", this->ID(), iStatus);
			fPIDList.push_back(iStatus);
			result=0;
		}
		else {
			syslog (LOG_ERR, "failed to start driftscan task with id=%d", this->ID());
			result=-1;
		}
	}
	return result;
}

int DriftScanTask::Stop()
{
	if (fVerbose>3) cout<<"DriftScanTask::Stop()"<<endl;
	int result=RTTask::Stop();
	syslog (LOG_NOTICE, "stopping driftscan task with id=%d", this->ID());
	return result;
}


//
// TrackingTask
//

int TrackingTask::Start()
{
	if (fVerbose>3) cout<<"TrackingTask::Start()"<<endl;
	int result=RTTask::Start();
	if (result==0) {
		// hier code, um messung auszuführen
//		int returnvalue = system ("ls -la > doof &");

		string cmdstring;
		char tmpstr[256];
		char datafilestr[256];
		sprintf(datafilestr,"task_track%04d%02d%02d_%05d",fStartTime.year(),fStartTime.month(),fStartTime.day(),(long)fStartTime.timestamp()%86400L);
		fDataFile=string(datafilestr);
//		fDataFile="task"+to_string<long>((long)fStartTime.timestamp(), std::dec);

		cmdstring="cd "+fExecutablePath+" && ";
		sprintf(tmpstr, string(_cmd_tracking).c_str(), (float)fTrackCoords.Phi(),
(float)fTrackCoords.Theta(), string(fDataPath+"/"+fDataFile).c_str() );
		cmdstring+=tmpstr;
#if __cplusplus > 199711L
		if (std::isnormal(fIntTime)) cmdstring+=" "+to_string<int>(fIntTime, std::dec);
#else
		if (std::isnormal<double>(fIntTime)) cmdstring+=" "+to_string<int>(fIntTime, std::dec);
#endif		

//		sprintf(cmdstr, string("cd "+fExecutablePath+" && "+_cmd_tracking).c_str(), (float)fTrackCoords.Phi(), (float)fTrackCoords.Theta(), string(fDataPath+"/"+fDataFile).c_str() );
		syslog (LOG_DEBUG, "executing command: %s", cmdstring.c_str());
		int iStatus = RunShellCommand(cmdstring.c_str());
//		int iStatus = RunShellCommand("sleep 1000");
//		int iStatus = RunShellCommand(string("cd "+fExecutablePath+" && rt_doof").c_str());
		if (iStatus>0) {
			syslog (LOG_NOTICE, "starting tracking task with id=%d (pid %d)", this->ID(), iStatus);
			fPIDList.push_back(iStatus);
			result=0;
		}
		else {
			syslog (LOG_ERR, "failed to start tracking task with id=%d", this->ID());
			result=-1;
		}
	}
	return result;
}

int TrackingTask::Stop()
{
	if (fVerbose>3) cout<<"TrackingTask::Stop()"<<endl;
	return RTTask::Stop();
}


//
// HorScanTask
//

int HorScanTask::Start()
{
	if (fVerbose>3) cout<<"HorScanTask::Start()"<<endl;
	int result=RTTask::Start();
	if (result==0) {
		// hier code, um messung auszuführen
		string cmdstring;
		char tmpstr[256];
		char datafilestr[256];
		sprintf(datafilestr,"task_horscan%04d%02d%02d_%05d",fStartTime.year(),fStartTime.month(),fStartTime.day(),(long)fStartTime.timestamp()%86400L);
		fDataFile=string(datafilestr);
//		fDataFile="task"+to_string<long>((long)fStartTime.timestamp(), std::dec);

		cmdstring="cd "+fExecutablePath+" && ";
		sprintf(tmpstr, string(_cmd_horscan).c_str(),(float)fStartCoords.Phi(), (float)fEndCoords.Phi(),
		 (float)fStartCoords.Theta(), (float)fEndCoords.Theta(),
		 string(fDataPath+"/"+fDataFile).c_str() );
		cmdstring+=tmpstr;
#if __cplusplus > 199711L
		if (isnormal(fStepAz) && isnormal(fStepAlt)) 
#else
		if (isnormal<double>(fStepAz) && isnormal<double>(fStepAlt)) 
#endif		
		{
			cmdstring+=" "+to_string<double>(fStepAz, std::dec)+" "+to_string<double>(fStepAlt, std::dec);
#if __cplusplus > 199711L
			if (isnormal(fIntTime)) cmdstring+=" "+to_string<double>(fIntTime, std::dec);
#else
			if (isnormal<double>(fIntTime)) cmdstring+=" "+to_string<double>(fIntTime, std::dec);
#endif		
		} else {
			// assume 1 deg step size, if no values are supplied
			cmdstring+=" 1 1";
#if __cplusplus > 199711L
			if (isnormal(fIntTime)) cmdstring+=" "+to_string<double>(fIntTime, std::dec);
#else
			if (isnormal<double>(fIntTime)) cmdstring+=" "+to_string<double>(fIntTime, std::dec);
#endif		
		}

//		sprintf(cmdstr, string("cd "+fExecutablePath+" && "+_cmd_horscan).c_str(), (float)fStartCoords.Phi(), (float)fEndCoords.Phi(), (float)fStartCoords.Theta(), (float)fEndCoords.Theta(), string(fDataPath+"/"+fDataFile).c_str() );
		syslog (LOG_DEBUG, "executing command: %s", cmdstring.c_str());
		int iStatus = RunShellCommand(cmdstring.c_str());
//		int iStatus = RunShellCommand("sleep 1000");
//		int iStatus = RunShellCommand(string("cd "+fExecutablePath+" && rt_doof").c_str());
		if (iStatus>0) {
			syslog (LOG_NOTICE, "starting HorScan task with id=%d (pid %d)", this->ID(), iStatus);
			fPIDList.push_back(iStatus);
			result=0;
		}
		else {
			syslog (LOG_ERR, "failed to start HorScan task with id=%d", this->ID());
			result=-1;
		}
	}
	return result;
}

int HorScanTask::Stop()
{
	if (fVerbose>3) cout<<"HorScanTask::Stop()"<<endl;
	return RTTask::Stop();
}


//
// EquScanTask
//

int EquScanTask::Start()
{
	if (fVerbose>3) cout<<"EquScanTask::Start()"<<endl;
	int result=RTTask::Start();
	if (result==0) {
		// hier code, um messung auszuführen
		string cmdstring;
		char tmpstr[256];
		char datafilestr[256];
		sprintf(datafilestr,"task_equscan%04d%02d%02d_%05d",fStartTime.year(),fStartTime.month(),fStartTime.day(),(long)fStartTime.timestamp()%86400L);
		fDataFile=string(datafilestr);
//		fDataFile="task"+to_string<long>((long)fStartTime.timestamp(), std::dec);
		cmdstring="cd "+fExecutablePath+" && ";
		sprintf(tmpstr, string(_cmd_equscan).c_str(),(float)fStartCoords.Phi(), (float)fEndCoords.Phi(),
		 (float)fStartCoords.Theta(), (float)fEndCoords.Theta(),
		 string(fDataPath+"/"+fDataFile).c_str() );
		cmdstring+=tmpstr;
#if __cplusplus > 199711L
		if (isnormal(fStepRa) && isnormal(fStepDec))
#else
		if (isnormal<double>(fStepRa) && isnormal<double>(fStepDec))
#endif		
		{
			cmdstring+=" "+to_string<double>(fStepRa, std::dec)+" "+to_string<double>(fStepDec, std::dec);
#if __cplusplus > 199711L
			if (isnormal(fIntTime)) cmdstring+=" "+to_string<double>(fIntTime, std::dec);
#else
			if (isnormal<double>(fIntTime)) cmdstring+=" "+to_string<double>(fIntTime, std::dec);
#endif			
		  
		} else {
			// assume 1 deg step size, if no values are supplied
			cmdstring+=" 1 1";
#if __cplusplus > 199711L
			if (isnormal(fIntTime)) cmdstring+=" "+to_string<double>(fIntTime, std::dec);
#else
			if (isnormal<double>(fIntTime)) cmdstring+=" "+to_string<double>(fIntTime, std::dec);
#endif
		}
		syslog (LOG_DEBUG, "executing command: %s", cmdstring.c_str());
		int iStatus = RunShellCommand(cmdstring.c_str());
//		int iStatus = RunShellCommand("sleep 1000");
//		int iStatus = RunShellCommand(string("cd "+fExecutablePath+" && rt_doof").c_str());
		if (iStatus>0) {
			syslog (LOG_NOTICE, "starting EquScan task with id=%d (pid %d)", this->ID(), iStatus);
			fPIDList.push_back(iStatus);
			result=0;
		}
		else {
			syslog (LOG_ERR, "failed to start EquScan task with id=%d", this->ID());
			result=-1;
		}
	}
	return result;
}

int EquScanTask::Stop()
{
	if (fVerbose>3) cout<<"EquScanTask::Stop()"<<endl;
	return RTTask::Stop();
}

//
// GotoHorTask
//

int GotoHorTask::Start()
{
	if (fVerbose>3) cout<<"GotoHorTask::Start()"<<endl;
	int result=RTTask::Start();
	if (result==0) {
		// hier code, um messung auszuführen
		char cmdstr[256];
		// first, send goto command to indi
		sprintf(cmdstr,"echo -n $(indi_setprop -p 7273 \"LX200 RT.HORIZONTAL_COORD.AZ;ALT=%f;%f\")",fGotoCoords.Phi(),fGotoCoords.Theta());
		int returnvalue = system (cmdstr);
		// now, wait until pos reached
		sprintf(	cmdstr,
					"indi_eval -p 7273 -t 2 -w 'abs(\"LX200 RT.HORIZONTAL_COORD.AZ\"-%f)<.085 && abs(\"LX200 RT.HORIZONTAL_COORD.ALT\"-%f)<.085'",
					(float)fGotoCoords.Phi(), (float)fGotoCoords.Theta());
		syslog (LOG_DEBUG, "executing command: %s", cmdstr);
		int iStatus = RunShellCommand(cmdstr);
		if (iStatus>0) {
			syslog (LOG_NOTICE, "starting slew to horizontal coordinate, task id=%d (pid %d)", this->ID(), iStatus);
			fPIDList.push_back(iStatus);
			result=0;
		}
		else {
			syslog (LOG_ERR, "failed to slew to horizontal coordinate, task id=%d", this->ID());
			result=-1;
		}
	}
	return result;
}

int GotoHorTask::Stop()
{
	if (fVerbose>3) cout<<"GotoHorTask::Stop()"<<endl;
	int result=RTTask::Stop();
	syslog (LOG_NOTICE, "stopping goto hor task with id=%d", this->ID());
	return result;
}


//
// GotoEquTask
//

int GotoEquTask::Start()
{
	if (fVerbose>3) cout<<"GotoEquTask::Start()"<<endl;
	int result=RTTask::Start();
	if (result==0) {
		// hier code, um messung auszuführen
		char cmdstr[256];
		// first, send goto command to indi
		sprintf(cmdstr,"echo -n $(indi_setprop -p 7273 \"LX200 RT.EQUATORIAL_EOD_COORD.RA;DEC=%f;%f\")",fGotoCoords.Phi(),fGotoCoords.Theta());
		int returnvalue = system (cmdstr);
		// now, wait until pos reached
		sprintf(	cmdstr,
					"indi_eval -p 7273 -t 2 -w 'abs(\"LX200 RT.EQUATORIAL_EOD_COORD.RA\"-%f)<.007 && abs(\"LX200 RT.EQUATORIAL_EOD_COORD.DEC\"-%f)<.09'",
					(float)fGotoCoords.Phi(), (float)fGotoCoords.Theta());
		syslog (LOG_DEBUG, "executing command: %s", cmdstr);
		int iStatus = RunShellCommand(cmdstr);
		if (iStatus>0) {
			syslog (LOG_NOTICE, "starting slew to equatorial coordinate, task id=%d (pid %d)", this->ID(), iStatus);
			fPIDList.push_back(iStatus);
			result=0;
		}
		else {
			syslog (LOG_ERR, "failed to slew to equatorial coordinate, task id=%d", this->ID());
			result=-1;
		}
	}
	return result;
}

int GotoEquTask::Stop()
{
	if (fVerbose>3) cout<<"GotoEquTask::Stop()"<<endl;
	int result=RTTask::Stop();
	syslog (LOG_NOTICE, "stopping goto equ task with id=%d", this->ID());
	return result;
}


//
// MaintenanceTask
//

int MaintenanceTask::Start()
{
   if (fVerbose>3) cout<<"MaintenanceTask::Start()"<<endl;
   int result=RTTask::Start();
   if (result==0) {
      // hier code, um messung auszuführen
      char cmdstr[256];
      // first, send goto command to indi
      sprintf(cmdstr,"sleep %d",int(fMaxRunTime*3600));
      syslog (LOG_DEBUG, "executing command: %s", cmdstr);
      int iStatus = RunShellCommand(cmdstr);
      if (iStatus>0) {
         syslog (LOG_NOTICE, "starting maintenance task, task id=%d (pid %d)", this->ID(), iStatus);
         fPIDList.push_back(iStatus);
         result=0;
      }
      else {
         syslog (LOG_ERR, "failed to start maintenance task, task id=%d", this->ID());
         result=-1;
      }
   }
   return result;
}

int MaintenanceTask::Stop()
{
   if (fVerbose>3) cout<<"MaintenanceTask::Stop()"<<endl;
   int result=RTTask::Stop();
   syslog (LOG_NOTICE, "stopping maintenance task with id=%d", this->ID());
   return result;
}

