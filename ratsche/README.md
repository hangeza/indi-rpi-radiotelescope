# RaTSche - The Radio Telescope Task Scheduler

This is a manager for observation time schedule and execution of tasks for the Pi Radio Telescope (PiRaTe) system.
Ratsche runs as a daemon service and communicates via the unix message queue (MSQ) system. New tasks can be defined and 
the status of current tasks can be monitored via a simple command line interface (CLI) program (ratsche client). 
This allows for relatively easy integration in js or php code for web interfaces.

Detailed features:
- Currently, the following tasks can be handled by Ratsche:
 * Park
 * Unpark
 * GotoHor - Slew to given set of horizontal (Az/Alt) coordinates
 * GotoEqu - Slew to given set of equatorial (RA/Dec) coordinates
 * Drift - Transit scan at given (fixed) horizontal coordinate for a certain duration and integration time. Save fieldstrength values of receiver system to file.
 * Tracking - Track given equatorial coordinate and record fieldstrength with given Duration and integration time from receiver system to file.
 * ScanHor - Scan a 2d grid (defined by window and stepping) in horizontal coordinates and record receiver fieldstrength to file.
 * ScanEqu - Scan a 2d grid (defined by window and stepping) in equatorial coordinates and record receiver fieldstrength to file.
 * Maintenance - Lock the scope for a given time at its current position for maintenance purposes
- One or multiple tasks can be defined using a simple task file format similar to the definition of cron jobs. The format of the file is the following:
    # RT TASK
    # v0.1 
    # task file for definition of measurement(s) to be done with radiotelescope
    # priority = {0=ignore|1=immediate|2=immediate when free|3=asap when optimal|4=anytime when optimal|5=low priority}
    # mode = {drift|track|equscan|horscan}
    # alt-period : (float hours) time-period after which the conditions are expected identical (e.g. 24h for equatorial features),
    #              0=always repeatable,
    #             -1=can not or shall not be executed at a later time
    #              *=not defined
    # x1,y1: coordinates of the lower left corner of the scanwindow for 2d-scans; coordinates of initial position for drift (coordinates are Hor.) and track (coordinates are equ)
    # meaning of columns:
    # start-time mode priority alt-period user x1 y1 x2 y2 step1 step2 int-time ref-cycle max.duration
    2021/09/01 10:48:00 unpark   1 0 hgz *    *   *  * * * * * 0.1
    2021/09/01 10:49:00 gotohor   1 0 hgz 190    45   *  * * * * * 0.02
    2021/09/01 10:50:00 maintenance   1 0 hgz *    *   *  * * * * * 0.01
    2021/09/01 10:52:00 drift   1 0 hgz 180    60   *  * * * 10 1 0.05
    2021/09/01 10:57:00 park   1 0 hgz *    *   *  * * * * * 0.1
- To add the task list to the scheduler, simply do:
 `ratsche -a task_file`

    