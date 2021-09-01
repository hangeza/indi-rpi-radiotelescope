#ifndef RATSCHE_MESSAGES_H
#define RATSCHE_MESSAGES_H

// start-time mode priority alt-period user x1 y1 x2 y2 step1 step2 int-time ref-cycle

enum { AC_NONE=0, AC_PING=1, AC_LIST=2, AC_ADD=4, AC_DELETE=8, AC_CANCEL=16, AC_STOP=32, AC_CLEAR=64 };

struct coords {
	coords() : x(0.), y(0.) {}
	coords(double _x, double _y)
		: x(_x), y(_y)
	{}
	double x;
	double y;
};

typedef struct task_struct {
	task_struct() {}
	task_struct(const task_struct& task)
		: id(task.id), type(task.type), start_time(task.start_time),
		 submit_time(task.submit_time), priority(task.priority),
		 alt_period(task.alt_period), coords1(task.coords1), coords2(task.coords2),
		 step1(task.step1), step2(task.step2),
		 int_time(task.int_time), ref_cycle(task.ref_cycle),
		 duration(task.duration), elapsed(task.elapsed), eta(task.eta), status(task.status)
	{
		(void) strcpy(user, task.user);
		(void) strcpy(comment, task.comment);
	}
	
	long				id;
	char				type;
	time_t			start_time;
	time_t			submit_time;
	char				priority;
	double			alt_period;
	char				user[16];
	struct coords	coords1;
	struct coords	coords2;
	double			step1;
	double			step2;
	double			int_time;
	int				ref_cycle;
	double			duration;
	double			elapsed;
	double			eta;
	int				status;
	char				comment[128];
} task_t;


typedef struct msg_struct {
	long int		mtype;
	int			msenderID;
	int			maction;
	int			msubaction;
	int			mseriesID;
	int			mseriesCount;
	task_t		mtask;
} message_t;

#endif // RATSCHE_MESSAGES_H

