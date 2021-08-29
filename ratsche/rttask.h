#ifndef _RTTASK_H
#define _RTTASK_H


#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <utility>

#include "time.h"
#include "astro.h"

/** @class RTTask
abstract base class for RT tasks
*/
class RTTask
{
   public:
		enum TASKSTATE { IDLE=0, WAITING, ACTIVE, FINISHED, STOPPED, CANCELLED, ERROR, TIMEOUT };
		enum TASKTYPE { DRIFT=0, TRACK, HORSCAN, EQUSCAN, GOTOHOR, GOTOEQU, PARK, MAINTENANCE, UNPARK };

		RTTask();
		RTTask(long id, int priority, const hgz::Time& scheduleTime, double intTime, int refInterval, double altPeriod)
		 : fId(id), fPriority(priority), fScheduleTime(scheduleTime), fSubmitTime(hgz::Time::Now()), fIntTime(intTime), fRefInterval(refInterval), fAltPeriod(altPeriod)
		{
			fNumTasks++;
			fUser="N/A";
			fElapsedTime=0.;
			fState=IDLE;
			fVerbose=4;
		}
		RTTask(long id, int priority, const hgz::Time& scheduleTime, const hgz::Time& submitTime,
      		 double intTime, int refInterval, double altPeriod)
		 : fId(id), fPriority(priority), fScheduleTime(scheduleTime), fSubmitTime(submitTime),
      	  fIntTime(intTime), fRefInterval(refInterval), fAltPeriod(altPeriod)
		{
			fNumTasks++;
			fUser="N/A";
			fElapsedTime=0.;
			fState=IDLE;
			fVerbose=4;
		}
		virtual ~RTTask();

		inline long ID() { return fId; }
		inline void SetID(long a_id) { fId=a_id; }
		inline long Priority() { return fPriority; }
		hgz::Time scheduleTime() const { return fScheduleTime; }
		hgz::Time submitTime() const { return fSubmitTime; }
		std::string User() const { return fUser; }
		inline void SetUser(const std::string& a_user) { fUser=a_user; }
		std::string Comment() const { return fComment; }
		inline void SetComment(const std::string& a_comment) { fComment=a_comment; }

		virtual int Start();
		virtual int Stop();
		virtual int Cancel();
//		virtual int Resume()=0;

		virtual TASKTYPE type() const =0;

		double IntTime() const { return fIntTime; }
		int RefInterval() const { return fRefInterval; }
		double AltPeriod() const { return fAltPeriod; }

		// Task State Methods
		TASKSTATE State() const { return fState; }
		double Eta() const;
		double ElapsedTime() const { return fElapsedTime; }
		double MaxRunTime() const { return fMaxRunTime; }
		void SetMaxRunTime(double runtime) { fMaxRunTime=runtime; }
		static int NumTasks() { return fNumTasks; }
		static bool isActiveTask() { return fAnyActive; }

		static void SetDataPath(const std::string& path) { fDataPath=path; }
		static const std::string& DataPath() { return fDataPath; }
		static void SetExecutablePath(const std::string& path) { fExecutablePath=path; }
		static const std::string& ExecutablePath() { return fExecutablePath; }

		int Verbose() const { return fVerbose; }
		void SetVerbose(int verbosity=1) { fVerbose=verbosity; }

		virtual void Print() const;
		virtual void Process();

	protected:
		long fId;
		int fPriority;
		hgz::Time fScheduleTime;
		hgz::Time fSubmitTime;
		hgz::Time fStartTime;
		double fIntTime;
		int fRefInterval;
		double fAltPeriod;
		std::string fUser;
		std::string fComment;
		TASKSTATE fState;
		double fElapsedTime;
		double fMaxRunTime;
		std::string fDataFile;
		std::string fLogFile;
		// static (global) Members
		static int fNumTasks;
		static bool fAnyActive;
		static std::string fDataPath;
		static std::string fExecutablePath;
		std::vector<int> fPIDList;
		int fVerbose;

		int RunShellCommand(const char *strCommand);
};


/** @class DriftScanTask
RT drift scan task
*/
class DriftScanTask : public RTTask
{
	public:
		DriftScanTask()
			: RTTask()
		{}
		DriftScanTask(long id, int priority, const hgz::Time& scheduleTime, const hgz::Time& submitTime,
      				  double intTime, int refInterval, double altPeriod, const hgz::SphereCoords& startCoords)
			: RTTask(id, priority, scheduleTime, submitTime, intTime, refInterval, altPeriod)
		{
			fStartCoords=startCoords;
		}
		virtual ~DriftScanTask() {}

		virtual TASKTYPE type() const { return RTTask::DRIFT; }

		hgz::SphereCoords StartCoords() const { return fStartCoords; }

		virtual int Start();
		virtual int Stop();
		virtual int Cancel() { return RTTask::Cancel(); }

		virtual void Print() const {}

	private:
		hgz::SphereCoords fStartCoords;
};


/** @class TrackingTask
RT tracking scan task
*/
class TrackingTask : public RTTask
{
	public:
		TrackingTask(long id, int priority, const hgz::Time& scheduleTime, const hgz::Time& submitTime,
      				 double intTime, int refInterval, double altPeriod, const hgz::SphereCoords& trackCoords)
			: RTTask(id, priority, scheduleTime, submitTime, intTime, refInterval, altPeriod)
		{
			fTrackCoords=trackCoords;
		}
		virtual ~TrackingTask() {}

		virtual TASKTYPE type() const { return RTTask::TRACK; }

		hgz::SphereCoords TrackCoords() const { return fTrackCoords; }

		virtual int Start();
		virtual int Stop();
		virtual int Cancel() { return RTTask::Cancel(); }
	private:
		hgz::SphereCoords fTrackCoords;
};


/** @class HorScanTask
RT scan task for scans in horizontal coordinates
*/
class HorScanTask : public RTTask
{
	public:
		HorScanTask()
      	: RTTask()
		{}
		HorScanTask(long id, int priority, const hgz::Time& scheduleTime, const hgz::Time& submitTime,
						double intTime, int refInterval, double altPeriod,
						const hgz::SphereCoords& startCoords, const hgz::SphereCoords& endCoords,
						double stepAz, double stepAlt)
			: RTTask(id, priority, scheduleTime, submitTime, intTime, refInterval, altPeriod),
			  fStartCoords(startCoords), fEndCoords(endCoords), fStepAz(stepAz), fStepAlt(stepAlt)
		{}
		virtual ~HorScanTask() {}

		virtual TASKTYPE type() const { return RTTask::HORSCAN; }

		hgz::SphereCoords StartCoords() const { return fStartCoords; }
		hgz::SphereCoords EndCoords() const { return fEndCoords; }
		double StepAz() const { return fStepAz; }
		double StepAlt() const { return fStepAlt; }

		virtual int Start();
		virtual int Stop();
		virtual int Cancel() { return RTTask::Cancel(); }

		virtual void Print() const {}

	private:
		hgz::SphereCoords fStartCoords, fEndCoords;
		double fStepAz,fStepAlt;
};


/** @class EquScanTask
RT scan task for scans in equatorial coordinates
*/
class EquScanTask : public RTTask
{
	public:
		EquScanTask()
      	: RTTask()
		{}
		EquScanTask(long id, int priority, const hgz::Time& scheduleTime, const hgz::Time& submitTime,
						double intTime, int refInterval, double altPeriod,
						const hgz::SphereCoords& startCoords, const hgz::SphereCoords& endCoords,
						double stepRa, double stepDec)
			: RTTask(id, priority, scheduleTime, submitTime, intTime, refInterval, altPeriod),
			  fStartCoords(startCoords), fEndCoords(endCoords), fStepRa(stepRa), fStepDec(stepDec)
		{}
		virtual ~EquScanTask() {}

		virtual TASKTYPE type() const { return RTTask::EQUSCAN; }

		hgz::SphereCoords StartCoords() const { return fStartCoords; }
		hgz::SphereCoords EndCoords() const { return fEndCoords; }
		double StepRa() const { return fStepRa; }
		double StepDec() const { return fStepDec; }

		virtual int Start();
		virtual int Stop();
		virtual int Cancel() { return RTTask::Cancel(); }

		virtual void Print() const {}

	private:
		hgz::SphereCoords fStartCoords, fEndCoords;
		double fStepRa,fStepDec;
};


/** @class GotoHorTask
task for slew operation to horizontal coordinate
*/
class GotoHorTask : public RTTask
{
	public:
		GotoHorTask()
			: RTTask()
		{}
		GotoHorTask(long id, int priority, const hgz::Time& scheduleTime, const hgz::Time& submitTime,
      				double altPeriod, const hgz::SphereCoords& gotoCoords)
			: RTTask(id, priority, scheduleTime, submitTime, 0, 0, altPeriod)
		{
			fGotoCoords=gotoCoords;
		}
		virtual ~GotoHorTask() {}

		virtual TASKTYPE type() const { return RTTask::GOTOHOR; }

		hgz::SphereCoords GotoCoords() const { return fGotoCoords; }

		virtual int Start();
		virtual int Stop();
		virtual int Cancel() { return RTTask::Cancel(); }

		virtual void Print() const {}

	private:
		hgz::SphereCoords fGotoCoords;
};

/** @class GotoEquTask
task for slew operation to equatorial coordinate
*/
class GotoEquTask : public RTTask
{
	public:
		GotoEquTask()
			: RTTask()
		{}
		GotoEquTask(long id, int priority, const hgz::Time& scheduleTime, const hgz::Time& submitTime,
      				double altPeriod, const hgz::SphereCoords& gotoCoords)
			: RTTask(id, priority, scheduleTime, submitTime, 0, 0, altPeriod)
		{
			fGotoCoords=gotoCoords;
		}
		virtual ~GotoEquTask() {}

		virtual TASKTYPE type() const { return RTTask::GOTOEQU; }

		hgz::SphereCoords GotoCoords() const { return fGotoCoords; }

		virtual int Start();
		virtual int Stop();
		virtual int Cancel() { return RTTask::Cancel(); }

		virtual void Print() const {}

	private:
		hgz::SphereCoords fGotoCoords;
};


/** @class MaintenanceTask
dummy task for maintenance operations. The purpose of this task is to simply block the taskmanager for a given time
*/
class MaintenanceTask : public RTTask
{
   public:
      MaintenanceTask()
         : RTTask()
      {}
      MaintenanceTask(long id, int priority, const hgz::Time& scheduleTime, const hgz::Time& submitTime,
                  double altPeriod)
         : RTTask(id, priority, scheduleTime, submitTime, 0, 0, altPeriod)
      {}
      virtual ~MaintenanceTask() {}

      virtual TASKTYPE type() const { return RTTask::MAINTENANCE; }

      virtual int Start();
      virtual int Stop();
      virtual int Cancel() { return RTTask::Cancel(); }

      virtual void Print() const {}

   private:
};

/** @class ParkTask
task to park the telescope
*/
class ParkTask : public RTTask
{
   public:
      ParkTask()
         : RTTask()
      {}
      ParkTask(long id, int priority, const hgz::Time& scheduleTime, const hgz::Time& submitTime,
                  double altPeriod)
         : RTTask(id, priority, scheduleTime, submitTime, 0, 0, altPeriod)
      {}
      virtual ~ParkTask() {}

      virtual TASKTYPE type() const { return RTTask::PARK; }

      virtual int Start();
      virtual int Stop();
      virtual int Cancel() { return RTTask::Cancel(); }

      virtual void Print() const {}

   private:
};

/** @class UnparkTask
task to unpark the telescope
*/
class UnparkTask : public RTTask
{
   public:
      UnparkTask()
         : RTTask()
      {}
      UnparkTask(long id, int priority, const hgz::Time& scheduleTime, const hgz::Time& submitTime,
                  double altPeriod)
         : RTTask(id, priority, scheduleTime, submitTime, 0, 0, altPeriod)
      {}
      virtual ~UnparkTask() {}

      virtual TASKTYPE type() const { return RTTask::UNPARK; }

      virtual int Start();
      virtual int Stop();
      virtual int Cancel() { return RTTask::Cancel(); }

      virtual void Print() const {}

   private:
};

#endif // _RTTASK_H

