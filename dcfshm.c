
#include <syslog.h>
#include <stdarg.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/shm.h>
#include <linux/serial.h>

#define PIDFILE "/var/run/dcfshm.pid"

/* Hack: include <asm/termios.h> gives a lot of double definition errors */
struct termios2 {
	tcflag_t c_iflag;		/* input mode flags */
	tcflag_t c_oflag;		/* output mode flags */
	tcflag_t c_cflag;		/* control mode flags */
	tcflag_t c_lflag;		/* local mode flags */
	cc_t c_line;			/* line discipline */
	cc_t c_cc[NCCS];		/* control characters */
	speed_t c_ispeed;		/* input speed */
	speed_t c_ospeed;		/* output speed */
};

/* atoi() at compile time */
#define BUILD_YEAR (\
    __DATE__[7] == '?' ? 1900 \
    : (((__DATE__[7] - '0') * 1000 ) \
    + (__DATE__[8] - '0') * 100 \
    + (__DATE__[9] - '0') * 10 \
    + __DATE__[10] - '0'))

/* dcf is missing the century, so this software is only valid for 100 years */
const int pivot_year = BUILD_YEAR;
int debug;

static inline void memory_barrier(void)
{
	asm volatile ("" : : : "memory");
}
/*
 * DCF77 raw time code
 *
 * From "Zur Zeit", Physikalisch-Technische Bundesanstalt (PTB), Braunschweig
 * und Berlin, Maerz 1989
 *
 * Timecode transmission:
 * AM:
 *	time marks are send every second except for the second before the
 *	next minute mark
 *	time marks consist of a reduction of transmitter power to 25%
 *	of the nominal level
 *	the falling edge is the time indication (on time)
 *	time marks of a 100ms duration constitute a logical 0
 *	time marks of a 200ms duration constitute a logical 1
 * FM:
 *	see the spec. (basically a (non-)inverted psuedo random phase shift)
 *
 * Encoding:
 * Second	Contents
 * 0  - 10	AM: free, FM: 0
 * 11 - 14	free
 * 15		R     - "call bit" used to signalize irregularities in the control facilities
 *		        (until 2003 indicated transmission via alternate antenna)
 * 16		A1    - expect zone change (1 hour before)
 * 17 - 18	Z1,Z2 - time zone
 *		 0  0 illegal
 *		 0  1 MEZ  (MET)
 *		 1  0 MESZ (MED, MET DST)
 *		 1  1 illegal
 * 19		A2    - expect leap insertion/deletion (1 hour before)
 * 20		S     - start of time code (1)
 * 21 - 24	M1    - BCD (lsb first) Minutes
 * 25 - 27	M10   - BCD (lsb first) 10 Minutes
 * 28		P1    - Minute Parity (even)
 * 29 - 32	H1    - BCD (lsb first) Hours
 * 33 - 34      H10   - BCD (lsb first) 10 Hours
 * 35		P2    - Hour Parity (even)
 * 36 - 39	D1    - BCD (lsb first) Days
 * 40 - 41	D10   - BCD (lsb first) 10 Days
 * 42 - 44	DW    - BCD (lsb first) day of week (1: Monday -> 7: Sunday)
 * 45 - 49	MO    - BCD (lsb first) Month
 * 50           MO0   - 10 Months
 * 51 - 53	Y1    - BCD (lsb first) Years
 * 54 - 57	Y10   - BCD (lsb first) 10 Years
 * 58 		P3    - Date Parity (even)
 * 59		      - usually missing (minute indication), except for leap insertion
 */

#define DCF_CALLBIT 1
#define DCF_ZONECHANGE 2
#define DCF_DST 4
#define DCF_LEAP 8
struct dcftime {
	unsigned int flags;
	unsigned char minutes;
	unsigned char hours;
	unsigned char days;
	unsigned char dow;
	unsigned char months;
	unsigned char years;
};

struct shmTime {
	int    mode; /* 0 - if valid is set:
		      *       use values,
		      *       clear valid
		      * 1 - if valid is set:
		      *       if count before and after read of values is equal,
		      *         use values
		      *       clear valid
		      */
	volatile int    count;
	time_t		clockTimeStampSec;
	int		clockTimeStampUSec;
	time_t		receiveTimeStampSec;
	int		receiveTimeStampUSec;
	int		leap;
	int		precision;
	int		nsamples;
	volatile int    valid;
	unsigned	clockTimeStampNSec;	/* Unsigned ns timestamps */
	unsigned	receiveTimeStampNSec;	/* Unsigned ns timestamps */
	int		dummy[8];
};

static inline int lprint(const char *format, ...)
{
va_list ap;
int ret;

ret = 0;
va_start(ap, format);
if (debug) {
	ret = vprintf(format, ap);
} else {
	vsyslog(LOG_DAEMON|LOG_INFO, format, ap);
}
va_end(ap);
return ret;
}

static inline int dprint(const char *format, ...)
{
va_list ap;
int ret;

ret = 0;
va_start(ap, format);
if (debug) {
	ret = vprintf(format, ap);
}
va_end(ap);
return ret;
}

static inline int pcheck(register unsigned char *bits, register char count)
{
	register int i;
	register unsigned char p = 1;

	for (i = 0; i <= count; i++)
		p ^= bits[i];
	return p;
}

struct dcftime *parsedcf(char bitno, unsigned char *bits)
{
	static struct dcftime ret;
	
	memset(&ret, 0, sizeof(ret));
	if (bitno < 58) {
		lprint("Not enough bits to parse %d\n", bitno);
		return NULL;
	}
	ret.flags |= (bits[15])?DCF_CALLBIT:0;
	ret.flags |= (bits[16])?DCF_ZONECHANGE:0;
	if (!(bits[17]^bits[18])) {
		lprint("Illegal time zone %d %d\n", bits[17],bits[18]);
		return NULL;
	}
	ret.flags |= (bits[17])?DCF_DST:0;
	ret.flags |= (bits[19])?DCF_LEAP:0;
	if (!(bits[20])) {
		lprint("No start of time code marker\n");
		return NULL;
	}
	if (!pcheck(bits + 21, 7)) {
		lprint("Minute parity wrong\n");
		return NULL;
	}
	ret.minutes = bits[21] + 2 * bits[22] + 4 * bits[23] + 8 * bits[24] +
		(bits[25] + 2 * bits[26] + 4 * bits[27]) * 10;
	if (!pcheck(bits + 29, 6)) {
		lprint("Hour parity wrong\n");
		return NULL;
	}
	ret.hours = bits[29] + 2 * bits[30] + 4 * bits[31] + 8 * bits[32] +
		(bits[33] + 2 * bits[34]) * 10;
	if (!pcheck(bits + 36, 22)) {
		lprint("Date parity wrong\n");
		return NULL;
	}
	ret.days = bits[36] + 2 * bits[37] + 4 * bits[38] + 8 * bits[39] +
		(bits[40] + 2 * bits[41]) * 10;
	ret.dow = bits[42] + 2 * bits[43] + 4 * bits[44];
	ret.months = bits[45] + 2 * bits[46] + 4 * bits[47] + 8 * bits[48] +
		(bits[49]) * 10;
	ret.years = bits[50] + 2 * bits[51] + 4 * bits[52] + 8 * bits[53] +
		(bits[54] + 2 * bits[55] + 4 * bits[56] + 8 * bits[57]) * 10;
	
	lprint("Parsed DCF time: %02d.%02d.%02d %02d:%02d, pivot year %d\n", ret.days, ret.months, ret.years, ret.hours, ret.minutes, pivot_year);
	return &ret;
}

static inline time_t dcf_to_time(register struct dcftime *p)
{
	static struct tm t;
	int y;

	t.tm_sec = 0;
	t.tm_min = p->minutes;
	t.tm_hour = p->hours;
	t.tm_mday = p->days;
	t.tm_mon = p->months - 1;
	y = p->years + (pivot_year / 100) * 100;
	if (y < pivot_year ) { y += 100 ;}
	t.tm_year = y - 1900;
	t.tm_isdst = p->flags & DCF_DST;
	return mktime(&t);
}

static inline char char_to_bit(unsigned char c)
{
	/* Theory of operation:
	 * We receive a bitstream from the dcf receiver that ideally consists
	 * of 1111 0000 for a 0 and 0000 0000 for a 1. Patterns that that
	 * with at least  three 1 are considered 0, the rest 1.
	 */
	if (c < 224) return 1;
	return 0;
}

char *char_to_bin(unsigned char c)
{
static char ret[9];
static const char zeroone[] = "01";
int i;

for (i = 7; i >= 0; i--) {
	ret[i] = zeroone[ c&1 ];
	c >>= 1;
}
return ret;
}

static inline void eprint(long int i, char *s)
{
if (i < 0) {
	lprint("%s: %d %s\n", s, errno, strerror(errno));
	exit(1);
}
}

void usage(const char *p, int c)
{
printf("Usage: %s [-d] [-u <unit>] [-b <baudrate>] [-f <device>]\n", p);
printf("\t-d\tdebug: do not fork and print lots of internal info\n");
printf("\t-u\tuse <unit> for the ntp shm driver (default 0)\n");
printf("\t-b\tuse <baudrate> for communication (default 9600)\n");
printf("\t-f\tuse <device> to communicated with (default /dev/ttyUSB0)\n");
exit(c);
}

int main (int argc, char **argv)
{
int fd;
unsigned char c;
struct termios term;
#if 0
struct termios2 tio2;
#endif
struct serial_struct ser_info;
int on = TIOCM_RTS;
struct timeval t, tl;
char time[20];
unsigned long td;
unsigned char bits[61];
unsigned char bitno;
struct dcftime *dtime;
int dcf_valid;
struct shmTime *shm;
int shmid;
time_t dcf_time;
int ntpunit;
long int baudrate;
char *device;
int opt;

openlog(NULL, LOG_PID, LOG_DAEMON);
debug = 0;
ntpunit = 0;
baudrate = 9600;
device = "/dev/ttyUSB0";
while ((opt = getopt(argc, argv, "du:b:f:")) != -1) {
	switch (opt) {
	case 'd':
		debug = 1;
		break;
	case 'u':
		ntpunit = atoi(optarg);
		break;
	case 'b':
		baudrate = atol(optarg);
		break;
	case 'f':
		device = optarg;
		break;
	default: /* ? */
		usage(argv[0], 1);
	}
}
if (optind < argc) {
	usage(argv[0], 1);
}

eprint((shmid = shmget(0x4e545030 + ntpunit, sizeof (struct shmTime), IPC_CREAT|0600)), 
	"Cannot get shm segment");
if ((shm = shmat(shmid, NULL, 0)) == (void *)-1) {
	printf("Cannot attach shm segment\n");
	exit(1);
}
memset(shm, 0, sizeof (struct shmTime));
shm->mode = 1;
shm->valid = 0;

eprint((fd = open(device, O_RDONLY|O_NOCTTY)), "Cannot open device");
#if 0
eprint(ioctl(fd, TCGETS2, &tio2), "TCGETS2 failed");
tio2.c_cflag &= ~CBAUD;
tio2.c_cflag |= CBAUDEX;
tio2.c_ispeed = baudrate;
tio2.c_ospeed = baudrate;
eprint(ioctl(fd, TCSETS2, &tio2), "TCSETS2 failed");
#endif
eprint(ioctl(fd, TIOCGSERIAL, &ser_info), "TIOCGSERIAL failed");
ser_info.flags = (ser_info.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST | ASYNC_LOW_LATENCY; 
ser_info.custom_divisor = ser_info.baud_base / baudrate; 
eprint(ioctl(fd, TIOCSSERIAL, &ser_info), "TIOCSSERIAL failed");
eprint(tcgetattr(fd , &term), "tcgetattr failed");
memset(term.c_cc, _POSIX_VDISABLE, sizeof(term.c_cc));
term.c_cc[VMIN] = 1;
term.c_cflag = CS8|CREAD|CLOCAL|PARENB;
term.c_iflag = IGNPAR;
term.c_oflag = 0;
term.c_lflag = 0;
cfsetispeed(&term, B38400);
cfsetospeed(&term, B38400);
eprint(tcsetattr(fd, TCSANOW, &term), "tcsetattr failed");

eprint(ioctl(fd, TIOCMBIC, &on), "ioctl TIOCMBIC failed");
dprint("%s: Line initialised\n", argv[0]);

dcf_valid = 0;
dcf_time = 0;
dtime = NULL;
bitno = 0;
eprint(gettimeofday(&tl, NULL), "gettimeofday failed");
if (localtime(&tl.tv_sec)->tm_year + 1900 > pivot_year + 99) {
	printf("Assumptions in this programm only hold for 100 years, review source and recompile!\n");
	exit(1);
}
if (!debug) {
	int p;
	eprint(daemon(0, 0), "daemon() failed");
	eprint((p = open(PIDFILE, O_RDWR|O_CREAT|O_EXCL|O_NOCTTY, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH)), "Cannot open pidfile");
	eprint(dprintf(p, "%d\n", getpid()), "Cannot write pid");
	eprint(close(p), "Cannot close pidfile");
	lprint("started and detached");
} else {
	lprint("pivot year set to %d\n", pivot_year);
}
while (read(fd ,&c, 1) == 1 ) {
	eprint(gettimeofday(&t, NULL), "gettimeofday failed");
	td = (t.tv_sec - tl.tv_sec) * 1000000 + (t.tv_usec - tl.tv_usec) - 1000000;
	if (!debug && (strftime(time, 20, "%H:%M:%S", localtime(&t.tv_sec)) == 0)) {
		lprint("strftime failed\n");
		exit(1);
	}
	dprint("%s % 8ld: 0x%02x %s % 3.2i -> %i  %ld %ld\n", time, td, c, char_to_bin(c), bitno, char_to_bit(c), t.tv_sec, dcf_time);
	if (labs(td) > 500000) {
		dprint("New minute detected after %d bits\n", bitno);
		dtime = parsedcf(bitno, bits);
		if (dtime != NULL) {
			dcf_valid = 1;
			dcf_time = dcf_to_time(dtime);
			dprint("\nFlags: %02x, Mins %d, hours %d, days %d, dow %d, months %d, years %d\n",
			dtime->flags, dtime->minutes, dtime->hours, dtime->days, dtime->dow, dtime->months, dtime->years);
			if (labs(t.tv_sec - dcf_time) > 999) {
				dcf_valid = 0;
				lprint("time warp > 1000s ignored\n");
			}
		} else {
			dcf_valid = 0;
		}
		bitno = 0;
	}
	if (labs(td) > 2000000) {
		dprint("timed out\n");
		dcf_valid = 0;
	}
	if (bitno > 60) {
		lprint("Bit overflow within a minute\n");
		dcf_valid = 0;
	} else {
		if (dcf_valid) {
			shm->valid = 0;
			shm->count++;
			memory_barrier();
			shm->clockTimeStampSec = dcf_time++;
			shm->receiveTimeStampSec = t.tv_sec;
			shm->receiveTimeStampUSec = t.tv_usec;
			shm->leap = dtime->flags & DCF_LEAP;
			memory_barrier();
			shm->count++;
			shm->valid = 1;
		}
		bits[bitno++] = char_to_bit(c);
	}
	tl = t;
}

lprint("read failed\n");
close(fd);
exit(1);
}
