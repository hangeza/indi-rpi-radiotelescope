# RaTSche - The Radio Telescope Task Scheduler

This is a manager for observation time schedule and execution of tasks for the Pi Radio Telescope (PiRaTe) system.
Ratsche runs as a daemon service and communicates via the unix message queue (MSQ) system. New tasks can be defined and 
the status of current tasks can be monitored via a simple command line interface (CLI) program (ratsche client). 
This allows for relatively easy integration in js or php code for web interfaces.

**Detailed features**

Currently, the following tasks can be handled by Ratsche:

 * Park
 * Unpark
 * GotoHor - Slew to given set of horizontal (Az/Alt) coordinates
 * GotoEqu - Slew to given set of equatorial (RA/Dec) coordinates
 * Drift - Transit scan at given (fixed) horizontal coordinate for a certain duration and integration time. Save fieldstrength values of receiver system to file.
 * Tracking - Track given equatorial coordinate and record fieldstrength with given Duration and integration time from receiver system to file.
 * ScanHor - Scan a 2d grid (defined by window and stepping) in horizontal coordinates and record receiver fieldstrength to file.
 * ScanEqu - Scan a 2d grid (defined by window and stepping) in equatorial coordinates and record receiver fieldstrength to file.
 * Maintenance - Lock the scope for a given time at its current position for maintenance purposes

One or multiple tasks can be defined using a simple task file format similar to the definition of cron jobs. The format of the file is the following:
```
# RT TASK                                                                                                                                                   
# v0.2 
# task file for definition of measurement(s) with the PiRaTe radio telescope
# fields are ignored or assumed with defaults when marked with '*'
# priority = {0=ignore|1=immediate|2=immediate when free|3=asap when optimal|4=anytime when optimal|5=low priority}
# mode = { drift | track | equscan | horscan | gotohor | gotoequ | park | unpark | maintenance }
# alt-period : time period after which the conditions are expected identical and the measurement can equally commence (hours)
#              (e.g. 24h for scans of stellar objects),
#              or  0 = task may be started any time,
#                 -1 = task shall only be started in the given time window defined by start time and max-duration
#                  * = don't care, start the task when convenient
# x1,y1: coordinates of the lower left corner of the scanwindow for 2d-scans (Az/Alt for Hor; RA/Dec for Equ)
#     or coordinates of measurement position for drift tasks (Az/Alt) 
#     or coordinates of measurement position for track tasks (RA/Dec)
# x2,y2: coordinates of the upper right corner of the scanwindow for 2d-scans (Az/Alt for Hor; RA/Dec for Equ)
# step1,step2 : step sizes for 2d scans (deg/deg for Hor; hours/deg for Equ)
# int_time : detector adc integration time constant in seconds
# ref_cycle : N/A
# max duration : maximum allowed run time of task (hours)
# meaning of columns:
# start-time mode priority alt-period user x1 y1 x2 y2 step1 step2 int-time ref-cycle max.duration comment
2021/09/04 11:30:00 unpark   1 0 rtuser *    *   *  * * * * * 0.1 "unpark"
2021/09/01 10:50:00 maintenance   1 0 rtuser *    *   *  * * * * * 1 "maintenance cycle"
2021/09/03 17:28:00 drift   1 0 rtuser 180    60   *  * * * 5 * 0.5 "transit scan test"
2021/09/04 14:15:00 track   1 -1 rtuser 10.9  7.0   *  * * * 10 * 1.0 "test sun track 12GHz"
2021/09/03 18:30:00 horscan 2  0 rtuser   170  24  190 34 0.5 0.5  0.5 * 0.1 "Test scan Az/Alt"
2021/09/04 11:31:00 equscan 2  1 rtuser   10.7 4 11.25 10 0.015 0.15  1 * 3 "sun scan 12GHz"
2021/09/04 13:30:00 park   1 0 rtuser *    *   *  * * * * * 0.1 "park"
2021/09/03 10:41:00 maintenance   1 -1 rtuser *    *   *  * * * * * 0.1 "maintenance cycle"

```

To add the task list to the scheduler, simply do `ratsche -a task_file`. To show the current status of all tasks, use `ratsche -l`.
