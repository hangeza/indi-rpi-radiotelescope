#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <syslog.h>
#include <unistd.h>		// for getopt()
#include <ctype.h>
#include <fcntl.h>
#include <signal.h>

#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>

#include "ratsche_message.h"
#include "rttask.h"
#include "time.h"

using namespace std;
using namespace hgz;

const int MSQ_ID = 10;

const string defaultTaskFile = "/home/ratsche/ratsche_tasks";

void Usage(const char* progname)
{
	cout<<"Radiotelescope Task Scheduler - HG Zaunick 2010-2011"<<endl;
	cout<<endl;
	cout<<" Usage : "<<string(progname)<<"  [-vlph?] -k <keyID> -d <taskID> -a <taskfile>"<<endl;
	cout<<"  command line options are:   "<<endl;
	cout<<"	 -l            list all tasks"<<endl;
	cout<<"	 -p            export tasklist (for storage in file) to stdout"<<endl;
	cout<<"	 -k <keyID>    use message queue with key keyID"<<endl;
	cout<<"	 -a <taskfile> add task(s) supplied in file taskfile"<<endl;
	cout<<"	 -c <taskID>   cancel task with ID taskId"<<endl;
	cout<<"	 -s <taskID>   stop task with ID taskId"<<endl;
	cout<<"	 -e <taskID>   erase task with ID taskId"<<endl;
	cout<<"	 -E            erase all tasks"<<endl;
	cout<<"	 -d            run as daemon (scheduling server)"<<endl;
	cout<<"	 -x <path>     path to the executable macros"<<endl;
	cout<<"	 -o <path>     path to data output"<<endl;
	cout<<"	 -v            increase verbosity level for stderr and syslog"<<endl;
	cout<<"	 -h -?         show this help and exit"<<endl;
	cout<<endl;
}


void error(const string& progname, const string& message) {
//	cerr<<string(asctime(localtime(time(NULL))))<<" "<<<<endl;
	char str[100];
	time_t now=time(NULL);
	strftime(str, 100, "%Y/%m/%d %H:%M:%S", localtime(&now));
	cerr<<string(str)<<" error in "<<progname<<" : "<<message<<endl;
}


int send_message(int msqid, int fromID, int toID, int action, int subaction, task_t* task, int seriesID=1, int seriesCount=1) {
	message_t smsg;
	int result;
//	int pid=getpid();

	smsg.mtype = toID;
	smsg.maction = action;
	smsg.msubaction = subaction;
	smsg.msenderID=fromID;
	smsg.mseriesID=seriesID;
	smsg.mseriesCount=seriesCount;

	if (task!=NULL) {
		smsg.mtask=*task;
		(void) strcpy(smsg.mtask.user, task->user);
		(void) strcpy(smsg.mtask.comment, task->comment);
	}

	size_t buf_length = sizeof(smsg);

	unsigned long int cnt=0;
	while ((result=msgsnd(msqid, &smsg, buf_length, IPC_NOWAIT)) < 0 && cnt++<100) {
		perror("msgsnd");
	}
	if (result<0) {
		syslog (LOG_CRIT, "error in msgsnd: unable to access message queue");
		return -1;
	}

//	syslog (LOG_DEBUG, "Sent %d bytes\n", buf_length);
//   printf("Message: Sent %d bytes\n", buf_length);
   return result;
}

int receive_message(int msqid, int* fromID, int toID, int* action, int* subaction, task_t* task, int* seriesID=NULL, int* seriesCount=NULL) {
	message_t rmsg;
//	int pid=getpid();

	size_t buf_length = sizeof(rmsg) ;
	int result;

	unsigned long int cnt=0;
	while ((result=msgrcv(msqid, &rmsg, buf_length, toID, IPC_NOWAIT)) < 0 && cnt++<100) {
		if (errno==ENOMSG){
			// no message, return simply
			return -1;
		}
		// wait 10ms and hope that access to the queue will work in a new iteration
		usleep(10000);
	}
	if (result<0) {
		syslog (LOG_CRIT, "error in msgrcv: unable to access message queue");
		return -1;
	}

	if (fromID!=NULL) *fromID=rmsg.msenderID;
	if (action!=NULL) *action=rmsg.maction;
	if (subaction!=NULL) *subaction=rmsg.msubaction;
	if (seriesID!=NULL) *seriesID=rmsg.mseriesID;
	if (seriesCount!=NULL) *seriesCount=rmsg.mseriesCount;
	if (task!=NULL) {
		*task=rmsg.mtask;
		(void) strcpy(task->user, rmsg.mtask.user);
		(void) strcpy(task->comment, rmsg.mtask.comment);
	}
	return result;
}

/*
typedef struct task_struct {
	long				id;
	char				type;
	time_t			start_time;
	char				priority;
	double			alt_period;
	char				user[16];
	struct coords	coords1;
	struct coords	coords2;
	double			step1;
	double			step2;
	double			int_time;
	int				ref_cycle;
	char				comment[128];
} task_t;
*/

void list_tasks(const vector<task_t>& tasklist) {
	cout<<"# RT TASK"<<endl;
	cout<<"# v0.1"<<endl;
	cout<<"# task file for definition of measurement(s) to be done with radiotelescope"<<endl;
	cout<<"# priority = {0=ignore|1=immediate|2=immediate when free|3=asap when optimal|4=anytime when optimal|5=low priority}"<<endl;
	cout<<"# mode = {drift|track|equscan|horscan}"<<endl;
	cout<<"# alt-period : (float hours) time-period after which the conditions are expected identical (e.g. 24h for equatorial features)"<<endl;
	cout<<"#              0=always repeatable,"<<endl;
	cout<<"#             -1=can not or shall not be executed at a later time"<<endl;
	cout<<"#              *=not defined"<<endl;
	cout<<"# x1,y1: coordinates of the lower left corner of the scanwindow for 2d-scans;"<<endl;
	cout<<"#  coordinates of initial position for drift (coordinates are Hor.) and track (coordinates are Equ)"<<endl;
	cout<<"# x2,y2: coordinates of the upper right corner of the scanwindow for 2d-scans;"<<endl;
	cout<<"# meaning of columns:"<<endl;
	cout<<"# start-time mode priority alt-period user x1 y1 x2 y2 step1 step2 int-time ref-cycle max.duration"<<endl;

	for (int i=0; i< tasklist.size(); i++){
		task_t task=tasklist[i];
		char str[100];
		strftime(str, 100, "%Y/%m/%d %H:%M:%S", localtime(&task.start_time));
		cout<<string(str)<<" "<<(int)task.type<<" "<<(int)task.priority<<" "
			 <<task.alt_period<<" "<<string(task.user)<<" "
			 <<task.coords1.x<<" "<<task.coords1.y<<" "
			 <<task.coords2.x<<" "<<task.coords2.y<<" "
			 <<task.step1<<" "<<task.step2<<" "
			 <<task.int_time<<" "<<task.ref_cycle<<" "<<task.duration<<endl;
	}
	return;
}

void export_tasks(std::ostream& ostr, const vector<task_t>& tasklist) {
   ostr<<"# RT TASK"<<endl;
   ostr<<"# v0.1"<<endl;
   ostr<<"# task file for definition of measurement(s) to be done with the RT300 radio telescope"<<endl;
   ostr<<"# priority = {0=ignore|1=immediate|2=immediate when free|3=asap when optimal|4=anytime when optimal|5=low priority}"<<endl;
   ostr<<"# mode = {drift|track|equscan|horscan}"<<endl;
   ostr<<"# alt-period : (float hours) time-period after which the conditions are expected identical (e.g. 24h for equatorial features)"<<endl;
   ostr<<"#              0=always repeatable,"<<endl;
   ostr<<"#             -1=can not or shall not be executed at a later time"<<endl;
   ostr<<"#              *=not defined"<<endl;
   ostr<<"# x1,y1: coordinates of the lower left corner of the scanwindow for 2d-scans;"<<endl;
   ostr<<"#  coordinates of initial position for drift (coordinates are Hor.) and track (coordinates are Equ)"<<endl;
   ostr<<"# x2,y2: coordinates of the upper right corner of the scanwindow for 2d-scans;"<<endl;
   ostr<<"# meaning of columns:"<<endl;
   ostr<<"# start-time mode priority alt-period user x1 y1 x2 y2 step1 step2 int-time ref-cycle max.duration"<<endl;

   for (int i=0; i< tasklist.size(); i++){
      task_t task=tasklist[i];
      char str[100];
      strftime(str, 100, "%Y/%m/%d %H:%M:%S", localtime(&task.start_time));
      ostr<<string(str)<<" "<<(int)task.type<<" "<<(int)task.priority<<" "
          <<task.alt_period<<" "<<string(task.user)<<" "
          <<task.coords1.x<<" "<<task.coords1.y<<" "
          <<task.coords2.x<<" "<<task.coords2.y<<" "
          <<task.step1<<" "<<task.step2<<" "
          <<task.int_time<<" "<<task.ref_cycle<<" "<<task.duration<<endl;
   }
   return;
}

void print_tasklist(const vector<task_t>& tasklist) {
	cout<<"# id start-time mode priority alt-period user x1 y1 x2 y2 step1 step2 int-time ref-cycle max-duration elapsed eta status"<<endl;
	for (int i=0; i< tasklist.size(); i++){
		task_t task=tasklist[i];
		char str[100];
		strftime(str, 100, "%Y/%m/%d %H:%M:%S", localtime(&task.start_time));
		cout<<task.id<<" "<<string(str)<<" "<<(int)task.type<<" "<<(int)task.priority<<" "
			 <<task.alt_period<<" "<<string(task.user)<<" "
			 <<task.coords1.x<<" "<<task.coords1.y<<" "
			 <<task.coords2.x<<" "<<task.coords2.y<<" "
			 <<task.step1<<" "<<task.step2<<" "
			 <<task.int_time<<" "<<task.ref_cycle<<" "
			 <<task.duration<<" "<<task.elapsed<<" "<<task.eta<<" "<<task.status<<endl;
	}
	return;
}

void print_task(const task_t& task) {
	cout<<"*** Task "<<task.id<<" ***"<<endl;
	cout<<" type       : "<<(int)task.type<<endl;
	cout<<" priority   : "<<(int)task.priority<<endl;
	printf(" start time : %s",asctime(localtime(&task.start_time)));
	printf(" submit time: %s",asctime(localtime(&task.submit_time)));
	cout<<" user       : "<<string(task.user)<<endl;
	cout<<" alt. period: "<<task.alt_period<<endl;
	cout<<" 1st point  : ("<<task.coords1.x<<" , "<<task.coords1.y<<")"<<endl;
	cout<<" 2nd point  : ("<<task.coords2.x<<" , "<<task.coords2.y<<")"<<endl;
	cout<<" step(x)    : "<<task.step1<<endl;
	cout<<" step(y)    : "<<task.step2<<endl;
	cout<<" int time   : "<<task.int_time<<endl;
	cout<<" ref cycle  : "<<task.ref_cycle<<endl;
	cout<<" max. duration : "<<task.duration<<endl;
	cout<<" elapsed time  : "<<task.elapsed<<endl;
	cout<<" eta        : "<<task.eta<<endl;
	cout<<" status     : "<<task.status<<endl;
	cout<<" comment    : "<<string(task.comment)<<endl;
}



int getTasklistFromFile(const string& filename, vector<task_t>& tasklist) {
	// parse file for following format:
	// start-time mode priority alt-period user x1 y1 x2 y2 step1 step2 int-time ref-cycle
	ifstream infile(filename.c_str());
	if (!infile.good() || infile.eof() || !infile.is_open()) return -1;
	while (!infile.eof()) {
		task_t task;
		string line;
		getline(infile,line);
//		cout<<" line1="<<line<<" , size="<<line.size()<<endl;
		if (line.empty()) continue;
		// remove comment lines
		size_t found=line.find_first_of('#');
		if (found!=string::npos) {
			//cout<<" # found at "<<found<<endl;
			line=line.erase(found, string::npos);
			//cout<<" line2="<<line<<" , size="<<line.size()<<endl;
		}
		int i=0;
		// remove leading spaces
		while (!line.empty() && (isspace(line[0]) || isdigit(line[0])==0)) line.erase(0,1);
		if (line.empty()) continue;
		// parse line
		istringstream isline(line);
		string _date,_time,_mode,_user,_prio;
		string _alt_period, _x1, _x2, _y1, _y2, _step1, _step2, _int_time, _ref_cycle, _duration;
		isline>>_date>>_time>>_mode>>_prio>>_alt_period>>_user
				>>_x1>>_y1>>_x2>>_y2>>_step1>>_step2>>_int_time>>_ref_cycle>>_duration;

/*
		cout<<"parsed data:"<<endl;
		cout<<" date="<<_date<<endl;
		cout<<" time="<<_time<<endl;
		cout<<"  x1="<<_x1<<" y1="<<_y1<<"  x2="<<_x2<<" y2="<<_y2<<endl;
*/
		if (_mode=="drift" || _mode=="DRIFT") {
			task.type=RTTask::DRIFT;
		} else if (_mode=="track" || _mode=="TRACK") {
			task.type=RTTask::TRACK;
		} else if (_mode=="equscan" || _mode=="EQUSCAN") {
			task.type=RTTask::EQUSCAN;
		} else if (_mode=="horscan" || _mode=="HORSCAN") {
			task.type=RTTask::HORSCAN;
		} else if (_mode=="gotohor" || _mode=="GOTOHOR") {
			task.type=RTTask::GOTOHOR;
		} else if (_mode=="gotoequ" || _mode=="GOTOEQU") {
			task.type=RTTask::GOTOEQU;
		} else if (_mode=="maintenance" || _mode=="MAINTENANCE") {
			task.type=RTTask::MAINTENANCE;
		} else if (_mode=="park" || _mode=="PARK") {
			task.type=RTTask::PARK;
		} else if (_mode=="unpark" || _mode=="UNPARK") {
			task.type=RTTask::UNPARK;
		} else {
			errno=0;
			task.type=strtol(_mode.c_str(),NULL,10);
			if (errno) {
				error("ratsche", "could not determine the task type");
				return -1;
			}
		}
		errno=0;
		task.priority=strtol(_prio.c_str(),NULL,10);
		if (errno || !isdigit(_prio[0])) {
			error("","could not determine priority");
		}
		errno=0;
		task.alt_period=0.;
		task.alt_period=strtod(_alt_period.c_str(),NULL);
		if (errno || _alt_period[0]=='*') {
			error("","could not determine alt_period, setting to -1 (=singular)");
		}

		(void) strcpy(task.user, _user.c_str());
		(void) strcpy(task.comment, "");
		errno=0;
		task.coords1.x=strtod(_x1.c_str(),NULL);
		if (errno || _x1[0]=='*') {
			error("","could not determine x1, setting to NAN");
			task.coords1.x=NAN;
		}
		errno=0;
		task.coords1.y=strtod(_y1.c_str(),NULL);
		if (errno || _y1[0]=='*') {
			error("","could not determine y1, setting to NAN");
			task.coords1.y=NAN;
		}
		errno=0;
		task.coords2.x=strtod(_x2.c_str(),NULL);
		if (errno || _x2[0]=='*') {
			error("","could not determine x2, setting to NAN");
			task.coords2.x=NAN;
		}
		errno=0;
		task.coords2.y=strtod(_y2.c_str(),NULL);
		if (errno || _y2[0]=='*') {
			error("","could not determine y2, setting to NAN");
			task.coords2.y=NAN;
		}
		errno=0;
		task.step1=strtod(_step1.c_str(),NULL);
		if (errno || _step1[0]=='*') {
			error("","could not determine step1, setting to NAN");
			task.step1=NAN;
		}
		errno=0;
		task.step2=strtod(_step2.c_str(),NULL);
		if (errno || _step2[0]=='*') {
			error("","could not determine step2, setting to NAN");
			task.step2=NAN;
		}
		errno=0;
		task.int_time=strtod(_int_time.c_str(),NULL);
		if (errno || _int_time[0]=='*') {
			error("","could not determine int-time, setting to NAN");
			task.int_time=NAN;
		}
		errno=0;
		task.ref_cycle=strtod(_ref_cycle.c_str(),NULL);
		if (errno || _ref_cycle[0]=='*') {
			error("","could not determine ref_cycle, setting to 0");
			task.ref_cycle=0;
		}
		errno=0;
		task.duration=strtod(_duration.c_str(),NULL);
		if (errno || _duration[0]=='*') {
			error("","could not determine duration, setting to 24h");
			task.duration=24.;
		}

		task.submit_time=time(NULL);
		task.status=0;
		task.eta=-1.;
		task.elapsed=0.;

		struct tm starttm;
		int y,m;
		sscanf(_date.c_str(),"%4d%*c%2d%*c%2d",&y,&m,&starttm.tm_mday);

		starttm.tm_year=y-1900;
		starttm.tm_mon=m-1;
		float s;
		sscanf(_time.c_str(),"%2d%*c%2d%*c%02f",&starttm.tm_hour,&starttm.tm_min,&s);
		starttm.tm_sec=(int)s;
		starttm.tm_isdst=-1;
		task.start_time=mktime(&starttm);

		tasklist.push_back(task);
	}
	return 0;
}


task_t toMsgTask(RTTask* task)
{
	task_t msgtask;
	msgtask.id=task->ID();
	msgtask.priority=task->Priority();
	msgtask.type=(int)task->type();
	msgtask.start_time=task->scheduleTime().timestamp();
	msgtask.submit_time=task->submitTime().timestamp();
	msgtask.int_time=task->IntTime();
	msgtask.ref_cycle=task->RefInterval();
	msgtask.alt_period=task->AltPeriod();
	msgtask.coords1.x=0.;
	msgtask.coords1.y=0.;
	msgtask.coords2.x=0.;
	msgtask.coords2.y=0.;
	msgtask.step1=0.;
	msgtask.step2=0.;
	msgtask.duration=task->MaxRunTime();
	msgtask.elapsed=task->ElapsedTime();
	msgtask.eta=task->Eta();
	msgtask.status=task->State();
	(void) strcpy(msgtask.user, task->User().c_str());
	(void) strcpy(msgtask.comment, task->Comment().c_str());
	switch (task->type()) {
		case RTTask::DRIFT:
			msgtask.coords1.x=dynamic_cast<DriftScanTask*>(task)->StartCoords().Phi();
			msgtask.coords1.y=dynamic_cast<DriftScanTask*>(task)->StartCoords().Theta();
			break;
		case RTTask::TRACK:
			msgtask.coords1.x=dynamic_cast<TrackingTask*>(task)->TrackCoords().Phi();
			msgtask.coords1.y=dynamic_cast<TrackingTask*>(task)->TrackCoords().Theta();
			break;
		case RTTask::HORSCAN:
			msgtask.coords1.x=dynamic_cast<HorScanTask*>(task)->StartCoords().Phi();
			msgtask.coords1.y=dynamic_cast<HorScanTask*>(task)->StartCoords().Theta();
			msgtask.coords2.x=dynamic_cast<HorScanTask*>(task)->EndCoords().Phi();
			msgtask.coords2.y=dynamic_cast<HorScanTask*>(task)->EndCoords().Theta();
			msgtask.step1=dynamic_cast<HorScanTask*>(task)->StepAz();
			msgtask.step2=dynamic_cast<HorScanTask*>(task)->StepAlt();
			break;
		case RTTask::EQUSCAN:
			msgtask.coords1.x=dynamic_cast<EquScanTask*>(task)->StartCoords().Phi();
			msgtask.coords1.y=dynamic_cast<EquScanTask*>(task)->StartCoords().Theta();
			msgtask.coords2.x=dynamic_cast<EquScanTask*>(task)->EndCoords().Phi();
			msgtask.coords2.y=dynamic_cast<EquScanTask*>(task)->EndCoords().Theta();
			msgtask.step1=dynamic_cast<EquScanTask*>(task)->StepRa();
			msgtask.step2=dynamic_cast<EquScanTask*>(task)->StepDec();
			break;
		case RTTask::GOTOHOR:
			msgtask.coords1.x=dynamic_cast<GotoHorTask*>(task)->GotoCoords().Phi();
			msgtask.coords1.y=dynamic_cast<GotoHorTask*>(task)->GotoCoords().Theta();
			break;
		case RTTask::GOTOEQU:
			msgtask.coords1.x=dynamic_cast<GotoEquTask*>(task)->GotoCoords().Phi();
			msgtask.coords1.y=dynamic_cast<GotoEquTask*>(task)->GotoCoords().Theta();
			break;
		default:
			break;
	}
	return msgtask;
}


RTTask* fromMsgTask(const task_t& msgtask)
{
	RTTask* task;

	switch ((RTTask::TASKTYPE)msgtask.type) {
		case RTTask::DRIFT:
			task=new DriftScanTask(msgtask.id, msgtask.priority,
										  Time((long double)msgtask.start_time), Time((long double)msgtask.submit_time),
										  msgtask.int_time, msgtask.ref_cycle, msgtask.alt_period,
										  hgz::SphereCoords(msgtask.coords1.x, msgtask.coords1.y));
			break;
		case RTTask::TRACK:
			task=new TrackingTask(msgtask.id, msgtask.priority,
										  Time((long double)msgtask.start_time), Time((long double)msgtask.submit_time),
										  msgtask.int_time, msgtask.ref_cycle, msgtask.alt_period,
										  hgz::SphereCoords(msgtask.coords1.x, msgtask.coords1.y));
			break;
		case RTTask::HORSCAN:
			task=new HorScanTask(msgtask.id, msgtask.priority,
										Time((long double)msgtask.start_time), Time((long double)msgtask.submit_time),
										msgtask.int_time, msgtask.ref_cycle, msgtask.alt_period,
										hgz::SphereCoords(msgtask.coords1.x, msgtask.coords1.y),
										hgz::SphereCoords(msgtask.coords2.x, msgtask.coords2.y),
										msgtask.step1, msgtask.step2);
			break;
		case RTTask::EQUSCAN:
			task=new EquScanTask(msgtask.id, msgtask.priority,
										Time((long double)msgtask.start_time), Time((long double)msgtask.submit_time),
										msgtask.int_time, msgtask.ref_cycle, msgtask.alt_period,
										hgz::SphereCoords(msgtask.coords1.x, msgtask.coords1.y),
										hgz::SphereCoords(msgtask.coords2.x, msgtask.coords2.y),
										msgtask.step1, msgtask.step2);
			break;
		case RTTask::GOTOHOR:
			task=new GotoHorTask(msgtask.id, msgtask.priority,
										Time((long double)msgtask.start_time), Time((long double)msgtask.submit_time),
										msgtask.alt_period,
										hgz::SphereCoords(msgtask.coords1.x, msgtask.coords1.y));
			break;
		case RTTask::GOTOEQU:
			task=new GotoEquTask(msgtask.id, msgtask.priority,
										Time((long double)msgtask.start_time), Time((long double)msgtask.submit_time),
										msgtask.alt_period,
										hgz::SphereCoords(msgtask.coords1.x, msgtask.coords1.y));
			break;
		case RTTask::MAINTENANCE:
			task=new MaintenanceTask(msgtask.id, msgtask.priority,
										Time((long double)msgtask.start_time), Time((long double)msgtask.submit_time),
										msgtask.alt_period);
			break;
		case RTTask::PARK:
			task=new ParkTask(msgtask.id, msgtask.priority,
										Time((long double)msgtask.start_time), Time((long double)msgtask.submit_time),
										msgtask.alt_period);
			break;
		case RTTask::UNPARK:
			task=new UnparkTask(msgtask.id, msgtask.priority,
										Time((long double)msgtask.start_time), Time((long double)msgtask.submit_time),
										msgtask.alt_period);
			break;
		default:
			return NULL;
			break;
	}
	task->SetMaxRunTime(msgtask.duration);
	task->SetComment(msgtask.comment);
	task->SetUser(msgtask.user);
	return task;
}


// process tasklist
void processTaskList(vector<RTTask*>& tasklist) {
	// remove identical tasks
	for (int first=0; first<(int)tasklist.size()-1; first++) {
		for (int second=first+1; second<tasklist.size(); second++) {
			//cout<<"compare "<<tasklist[first]->ID()<<" and "<<tasklist[second]->ID()<<endl;
			// do we have the same type?
			if (tasklist[first]->type()!=tasklist[second]->type()) continue;
			// do the start times differ less than 30s?
			if (fabs(tasklist[first]->scheduleTime().timestamp()-tasklist[second]->scheduleTime().timestamp())>30.) continue;
			// do the int times differ by more than 1e-3?
			if (fabs(tasklist[first]->IntTime()-tasklist[second]->IntTime())>1e-3) continue;
			// do the ref intervals differ by more than 5?
			if (abs(tasklist[first]->RefInterval()-tasklist[second]->RefInterval())>5) continue;
			// when we reach here, the tasks are considered identical, so remove the second one
			syslog (LOG_WARNING, "task id %d is identical to id %d. removing the latter", (int)tasklist[first]->ID(), (int)tasklist[second]->ID());
			delete tasklist[second];
			tasklist.erase(tasklist.begin()+second);
		}
	}


	// sort tasklist for start-time in ascending order
	for (int first=0; first<(int)tasklist.size()-1; first++) {
		for (int second=first+1; second<tasklist.size(); second++) {
			if (tasklist[first]->scheduleTime().timestamp()>tasklist[second]->scheduleTime().timestamp()) {
				swap(tasklist[first], tasklist[second]);
			}
		}
	}
	// now process each task by calling the tasks Process() method
	for (vector<RTTask*>::iterator it=tasklist.begin(); it!=tasklist.end(); ++it) {
		(*it)->Process();
	}
}


void daemonize()
{
        int i;

        if(getppid()==1) return; /* already a daemon */
        i=fork();
        if (i<0) exit(1); /* fork error */
        if (i>0) exit(0); /* parent exits */

        /* child (daemon) continues */
        setsid(); /* obtain a new process group */
        for (i=getdtablesize();i>=0;--i) close(i); /* close all descriptors */
        i=open("/dev/null",O_RDWR); dup(i); dup(i); /* handle standard I/O */

        /* first instance continues */
        signal(SIGCHLD,SIG_IGN); /* ignore child */
        signal(SIGTSTP,SIG_IGN); /* ignore tty signals */
        signal(SIGTTOU,SIG_IGN);
        signal(SIGTTIN,SIG_IGN);
}


// main program
int main(int argc, char *argv[])
{
	int msqid;
	int msgflg = IPC_CREAT | 0666;
	key_t key;
	message_t smsg;
	size_t buf_length;
	bool server=false;
	int fromid=0;
	int verbose=0;
	vector<pair<int,int> > cmdLineActions;
	bool exportTaskList=false;
	long lastTaskID=-1;
//	int cmdLineAction=AC_NONE;
//	int cmdLineSubAction=0;

	key = MSQ_ID;

	// command line parsing
	int ch;
//	const char* progname=argv[0];
	string infile = "";
	string execpath = "";
	string datapath = "/tmp/ratsche";

	while ((ch = getopt(argc, argv, "vlpdEe:a:s:c:k:x:o:h?")) != EOF) {
		switch ((char)ch) {
			case 'v':
				// increase verbosity level
				verbose++;
				break;
			case 'a':
				// add request only once to the action list
				if (optarg!="" && infile=="") {
					infile=string(optarg);
					cmdLineActions.push_back(make_pair((int)AC_ADD,0));
				}
				break;
			case 'k':
				if (optarg!="") {
					key=atoi(optarg);
				}
				break;
			case 'p':
				cmdLineActions.push_back(make_pair((int)AC_LIST,0));
				exportTaskList=true;
				break;
			case 'd':
				server=true;
				break;
			case 'l':
				cmdLineActions.push_back(make_pair((int)AC_LIST,0));
				break;
			case 'E':
				cmdLineActions.push_back(make_pair((int)AC_CLEAR,0));
				break;
			case 'e':
				cmdLineActions.push_back(make_pair((int)AC_DELETE,atoi(optarg)));
				break;
			case 's':
				cmdLineActions.push_back(make_pair((int)AC_STOP,atoi(optarg)));
				break;
			case 'c':
				cmdLineActions.push_back(make_pair((int)AC_CANCEL,atoi(optarg)));
				break;
			case 'x':
				execpath=optarg;
				break;
			case 'o':
				datapath=optarg;
				break;
			case 'h':
			case '?':  Usage(argv[0]); return 0;
			default: break;
		}
	}

	if (verbose>4) verbose=4;

/*
   argc -= optind;
   argv += optind;

   if (argc==1) { infile=*argv; }
*/




	if (verbose>3)	cout<<"pid="<<getpid()<<endl;
	if (verbose>3) printf("Calling msgget with key %#lx and flag %#o\n",key,msgflg);

	if ((msqid = msgget(key, msgflg )) < 0) {
		perror("msgget");
		exit(1);
	}
	else if (verbose>2)	printf("msgget: msgget succeeded: msqid = %d\n", msqid);

	struct msqid_ds msqinfo;
	// see first if the message queue is full
	if (msgctl(msqid, IPC_STAT, &msqinfo)<0) {
		perror("msgctl");
		exit(1);
	} else {
		// message queue is full and we started as server process, so throw away all messages
		// since they are useless
		if (verbose>3) cout<<" nr. of messages in queue: "<<msqinfo.msg_qnum<<endl;
		if (msqinfo.msg_qnum>200 && server) {
			int fromid, action, subaction;
			while (receive_message(msqid, &fromid, 0, &action, &subaction, NULL) >= 0);
		}
	}

	if (send_message(msqid, getpid(), 1, AC_PING, 0, NULL) < 0) {
		perror("send_message");
		exit(1);
	}
	else if (verbose>1) printf("sent ping\n");

	// wait 100ms
	usleep(100000);

	int action,subaction;
	if (receive_message(msqid, &fromid, getpid(), &action, &subaction, NULL) < 0) {
		if (errno==ENOMSG){
			if (!server) {
				cerr<<string(argv[0])<<": no connection to server"<<endl;
				return(-1);
			} else {
				if (verbose>1)
					cout<<"no message: i'm a server"<<endl;
			}
		} else perror("receive_message");
	}
	else {
		if (server) {
			cerr<<string(argv[0])<<": server already running"<<endl;
			return(-1);
		} else {
			if (verbose>1) printf("received pong: i'm client nr. %d\n", getpid());
		}
	}


	// execute the program in server mode from here on
	if (server)
	{
		//daemon(NULL, NULL);
		daemon(0, 0);
		//daemonize();
		//vector<task_t> tasklist;
		vector<RTTask*> tasklist2;
		try
		{
			int facility_priority = LOG_NOTICE; // default log priority is LOG_NOTICE
			if (verbose==1) facility_priority = LOG_INFO; // show also infos in syslog if verbose setting enabled
			if (verbose>1) facility_priority = LOG_DEBUG; // show all in syslog if verbosity increased

			setlogmask (LOG_UPTO (facility_priority));

			//openlog (NULL, LOG_CONS | LOG_PERROR | LOG_PID, LOG_DAEMON);
			openlog (NULL, LOG_PID, LOG_DAEMON);
			syslog (LOG_NOTICE, "starting RT task scheduler server");
			syslog (LOG_NOTICE, "using message queue 0x%04x",key);

			if (!execpath.empty()) {
				RTTask::SetExecutablePath(execpath);
				syslog (LOG_NOTICE, "using executable path %s",execpath.c_str());
			}
			if (!datapath.empty()) {
				RTTask::SetDataPath(datapath);
				syslog (LOG_NOTICE, "using data path %s",datapath.c_str());
			}
			// clear queue
			int nrOldMsg=0;
			while (receive_message(msqid, &fromid, 0, &action, &subaction, NULL) >= 0) {nrOldMsg++;}
			if (nrOldMsg) syslog (LOG_WARNING, "found %d zombie message(s) in queue...deleting",nrOldMsg);

			// try to load tasklist from previous session
			{
				// add task(s)
				vector<task_t> tasklist;
				if (getTasklistFromFile(defaultTaskFile, tasklist)!=0) {
					error(argv[0], "reading task file");
				} else {
					//list_tasks(tasklist);
					// submit tasklist
					// loop over tasks
					syslog (LOG_NOTICE, "loading tasklist from previous session, adding %d tasks", tasklist.size());
					for (int i=0; i<tasklist.size(); i++) {
						task_t task=tasklist[i];
						task.id=++lastTaskID;
						syslog (LOG_INFO, "received ADD request, adding new task (id=%d) to list", task.id);
						RTTask* taskptr=fromMsgTask(task);
						if (taskptr!=NULL) tasklist2.push_back(taskptr);
					}
				}
			}
			// stay in endless loop
			while (true) {
				// see if there is a message in the queue
				task_t task;
				if (receive_message(msqid, &fromid, 1, &action, &subaction, &task) >= 0) {
					RTTask* taskptr;
					switch (action) {
						case AC_PING:
							// received ping, send back echo
							syslog (LOG_DEBUG, "received PING message, sending back echo");
							if (send_message(msqid, 1, fromid, AC_PING, 0, NULL) < 0) {
								syslog (LOG_CRIT, "unable to send message to message queue");
								perror("send_message");
								//exit(1);
							}
							else {
								//printf("pingpong\n");
							}
							break;
						case AC_LIST:
							// List all tasks
							syslog (LOG_INFO, "received LIST request, sending back list of %d task(s)", tasklist2.size());
							//cout<<"received LIST"<<endl;
							//cout<<" tasklist-size()="<<tasklist.size()<<endl;
							if (!tasklist2.size()) {
								// send empty list
								if (send_message(msqid, 1, fromid, AC_LIST, 0, NULL, 1, 0) < 0) {
									syslog (LOG_CRIT, "unable to send message to message queue");
									perror("send_message");
									//exit(1);
								}
							} else
							for (int i=0; i<tasklist2.size(); i++) {
								task_t _task=toMsgTask(tasklist2[i]);
								//_task.id=
								if (send_message(msqid, 1, fromid, AC_LIST, 0, &_task, i+1, tasklist2.size()) < 0) {
	//							if (send_message(msqid, 1, fromid, AC_LIST, 0, &tasklist[i], i+1, tasklist.size()) < 0) {
									syslog (LOG_CRIT, "unable to send message to message queue");
									perror("send_message");
									//exit(1);
								}
							}
							break;
						case AC_ADD:
							// add task
							//cout<<"received ADD"<<endl;
							task.id=++lastTaskID;
							syslog (LOG_INFO, "received ADD request, adding new task (id=%d) to list", task.id);
	//						print_task(task);
							taskptr=fromMsgTask(task);
							if (taskptr!=NULL) tasklist2.push_back(taskptr);
	//						tasklist.push_back(task);
							break;
						case AC_DELETE:
							// delete task
							syslog (LOG_INFO, "received DELETE request, deleting task (id=%d) from list", subaction);
							for (vector<RTTask*>::iterator it=tasklist2.begin(); it!=tasklist2.end(); ++it) {
								if ((*it)->ID()==subaction) {
									if (*it!=NULL) delete *it;
									tasklist2.erase(it);
									syslog (LOG_DEBUG," deleted task id=%d, new size=%d", subaction, tasklist2.size());
									break;
								}
							}
							//syslog (LOG_WARNING, "trying to delete task id=%d, which does not exist", subaction);
							break;
						case AC_STOP:
							// stop task
							syslog (LOG_INFO, "received STOP request, stopping task (id=%d)", subaction);
							for (vector<RTTask*>::iterator it=tasklist2.begin(); it!=tasklist2.end(); ++it) {
								if ((*it)->ID()==subaction) {
									if (*it!=NULL) (*it)->Stop();
									syslog (LOG_DEBUG," stopped task id=%d", subaction);
									break;
								}
							}
							break;
						case AC_CANCEL:
							// cancel task
							syslog (LOG_INFO, "received CANCEL request, cancelling task (id=%d)", subaction);
							for (vector<RTTask*>::iterator it=tasklist2.begin(); it!=tasklist2.end(); ++it) {
								if ((*it)->ID()==subaction) {
									if (*it!=NULL) (*it)->Cancel();
									syslog (LOG_DEBUG," cancelled task id=%d", subaction);
									break;
								}
							}
							break;
						case AC_CLEAR:
							// delete all tasks
							syslog (LOG_INFO, "received CLEAR request, deleting all tasks");
							while ( !tasklist2.empty() ) {
								auto it = tasklist2.begin();
								long int id = (*it)->ID();
								if ( (*it)->State() == RTTask::ACTIVE ) {
									(*it)->Cancel();
								}
								delete *it;
								tasklist2.erase(it);
								syslog (LOG_DEBUG," deleted task id=%d, new size=%d", id, tasklist2.size());
							}
							break;
						default: break;
					}
					// process all tasks
					processTaskList(tasklist2);
					// the tasklist has been modified, so backup it to file
					vector<task_t> _tasklist;
					for (int i=0; i<tasklist2.size(); i++) {
						_tasklist.push_back(toMsgTask(tasklist2[i]));
					}
					ofstream ostr(defaultTaskFile.c_str());
					export_tasks(ostr, _tasklist);
				} else {
					// process all tasks
					processTaskList(tasklist2);
					// sleep for 10ms
					usleep(10000);
				}
			}
		} // if (server)
		catch (...) {
			// if there's any uncaught exception, clear the task list cleanly,
			// write to the system log and exit
			syslog (LOG_CRIT, "caught unhandled exception");
			syslog (LOG_CRIT, "stopping server.");
			// unqueue task list
			while (tasklist2.size()) {
				if (tasklist2[0]!=NULL) delete tasklist2[0];
				tasklist2.erase(tasklist2.begin());
			}
			exit(-1);
		}
		// Todo: evaluate system signals (SIGTERM, SIGKILL etc.)
		// and unmount the task list cleanly
	}


//*******************************************************//

	// run as client

	task_t task;
	task.id=1234;
	task.type=1;
	task.priority=2;
	task.start_time=time(NULL);
	task.submit_time=time(NULL);
	task.type=1;
	(void) strcpy(task.user, "hgz");
	(void) strcpy(task.comment, "dummy task");

	//print_task(task);

	// evaluate command line options
	for (int i=0; i<cmdLineActions.size(); i++)
	{
		int act=cmdLineActions[i].first;
		int subact=cmdLineActions[i].second;

		// list tasks
		if (act==AC_LIST) {
			if (send_message(msqid, getpid(), 1, AC_LIST, 0, NULL) < 0) {
				perror("send_message");
				exit(1);
			}
			else if (verbose>1) printf("sent LIST\n");

			usleep(10000);

			unsigned long int ctr=0;
			int msgcount=-1;
			vector<task_t> tasklist;
			while (ctr<100 && msgcount!=0) {
				int serid,sercnt,fromid;
				if (receive_message(msqid, &fromid, getpid(), &action, &subaction, &task, &serid, &sercnt) >= 0) {
					if (sercnt==0) { msgcount=0; break; }
					msgcount=sercnt-serid;
					tasklist.push_back(task);
					if (verbose>3) cout<<"rx task entry: serid="<<serid<<" , sercnt="<<sercnt<<endl;
				} else {
					// sleep 10ms
					usleep(10000);
					ctr++;
				}
			}
			if (msgcount<0) error(argv[0],"timeout receiving LIST");
			else {
				if (verbose>2) cout<<"received "<<tasklist.size()<<" entries. (rx-dur="<<ctr*10<<"ms)"<<endl;
				if (exportTaskList) {
					export_tasks(cout, tasklist);
					exportTaskList=false;
				} else {
					print_tasklist(tasklist);
				}
			}
		} else if (act==AC_DELETE) {
			// delete task
			if (send_message(msqid, getpid(), 1, AC_DELETE, subact, NULL) < 0) {
				perror("send_message");
				exit(1);
			}
			else if (verbose>1) printf("sent DELETE\n");
                } else if (act==AC_CLEAR) {
                        // delete all tasks
                        if (send_message(msqid, getpid(), 1, AC_CLEAR, 0, NULL) < 0) {
                                perror("send_message");
                                exit(1);
                        }
                        else if (verbose>1) printf("sent CLEAR\n");
		} else if (act==AC_STOP) {
			// stop task
			if (send_message(msqid, getpid(), 1, AC_STOP, subact, NULL) < 0) {
				perror("send_message");
				exit(1);
			}
			else if (verbose>1) printf("sent STOP\n");
		} else if (act==AC_CANCEL) {
			// cancel task
			if (send_message(msqid, getpid(), 1, AC_CANCEL, subact, NULL) < 0) {
				perror("send_message");
				exit(1);
			}
			else if (verbose>1) printf("sent CANCEL\n");
		} else if (act==AC_ADD) {
			// add task(s)
			vector<task_t> tasklist;
			if (getTasklistFromFile(infile, tasklist)!=0) {
				error(argv[0], "reading task file");
			} else {
//				list_tasks(tasklist);
				// submit tasklist
				// loop over tasks
				for (int i=0; i<tasklist.size(); i++) {
					if (send_message(msqid, getpid(), 1, AC_ADD, 0, &tasklist[i]) < 0) {
						error(argv[0], "adding task failed");
						perror("send_message");
					}
					else if (verbose>1) printf("added Task\n");
				}
			}
		}
	}

	exit(0);

//	cout<<"sizeof(RTTask)="<<sizeof(RTTask)<<endl;
//	cout<<"sizeof(DriftScanTask)="<<sizeof(DriftScanTask)<<endl;
//	DriftScanTask doof;
//	doof.SetComment("das istn sinnloser kommentar.");
//	cout<<"sizeof(DriftScanTask)="<<sizeof(doof)<<endl;
	time_t t1 = time(NULL);
	Time t2=Time::Now();
	cout<<"time1="<<t1<<endl;
	cout<<"time2="<<t2.timestamp()<<endl;
}

