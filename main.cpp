// Motor

#include <cstdio>
#include <cstdlib>
#include <cstdbool>
#include <cstring>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <sys/errno.h>
#include <netdb.h>
#include <unistd.h>
#include <time.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "b3packet.h"

#include "c3ga.h"

using namespace c3ga;

#include "RotorIy.h"

#define	MYECHO_PORT 5790
#define MAXLINE sizeof(struct B3packet)

extern int errno;

#define SHOW_RAW_ACC     (1<<0)
#define SHOW_RAW_GYRO    (1<<1)
#define SHOW_MOTOR       (1<<6)
#define SHOW_ACCZ        (1<<7)
#define SHOW_LL          (1<<8)
#define SHOW_VIEWER      (1<<9)

static int show_flags;
static float filter_gain = 0.5f;

#if 0
static int
mymillis (void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

#define GRAVITY_MSS 9.80665f
#define GEPSILON 0.05f
#endif

void
dreck (int sockfd)
{
  int n;
  socklen_t clilen;
  struct sockaddr_in cli_addr;
  struct B3packet pkt;

  float gx, gy, gz;
  float ax, ay, az;
  int count = 0;


  const float epsilon = 0.000001;
  const float dt = 0.001;
  RotorIy RI (dt, filter_gain, epsilon);
  mv S;

  for (;;)
    {
      clilen = sizeof(cli_addr);
      bzero (&cli_addr, clilen);
      n = recvfrom(sockfd, &pkt, MAXLINE, 0,
		   (struct sockaddr *) &cli_addr, &clilen);
      if (n < 0) {
	fprintf (stderr, "server: recvfrom error");
	exit (1);
      }

      // printf ("%d %d\n", sizeof(cli_addr), clilen);
      // printf ("%d bytes %02x\n", n, pkt.head);
      if (pkt.head != 0xb3)
	continue;
      if (pkt.tos == TOS_IMU)
	{
	  union { float f; uint8_t bytes[sizeof(float)];} uax, uay, uaz;
	  union { float f; uint8_t bytes[sizeof(float)];} ugx, ugy, ugz;
	  memcpy (uax.bytes, &pkt.data[0], 4);
	  memcpy (uay.bytes, &pkt.data[4], 4);
	  memcpy (uaz.bytes, &pkt.data[8], 4);
	  memcpy (ugx.bytes, &pkt.data[12], 4);
	  memcpy (ugy.bytes, &pkt.data[16], 4);
	  memcpy (ugz.bytes, &pkt.data[20], 4);
	  ax = uax.f; ay = uay.f; az = uaz.f;
	  gx = ugx.f; gy = ugy.f; gz = ugz.f;
	  if (show_flags & SHOW_RAW_ACC)
	    printf("ax: %f ay: %f az: %f\n", ax, ay, az);
	  if (show_flags & SHOW_RAW_GYRO)
	    printf("gx: %f gy: %f gz: %f\n", gx, gy, gz);

	  // gyro offset calibration needed
	  gx -= 0.015; gy -= -0.024; gz -= 0.017;

	  S = RI.UpdateIMU (ax, ay, az, gx, gy, gz);
	}

      if (show_flags & SHOW_MOTOR)
	printf ("S = %s\n", S.c_str("%2.3f"));
      if (show_flags & SHOW_VIEWER)
	{
	  if ((count % 100) == 0)
	    printf ("S=%s;$\n", S.c_str("%2.3f"));
	  fflush (stdout);
	}
      count++;
    }
}

void
err_quit (const char *msg)
{
  fprintf (stderr, "%s\n", msg);
  exit (1);
}

int
main (int argc, char *argv[])
{
  int sockfd;
  int option;
  char *s;
  struct sockaddr_in serv_addr;

  while (--argc > 0 && (*++argv)[0] == '-')
    for (s = argv[0]+1; *s != '\0'; s++)
      switch (*s)
	{
	case 'h':
	  printf ("Usage:\n"
		  "   b3test [OPTION...]\n"
		  "\nHelp Options:\n"
		  "  -h	Show help options\n"
		  "\nReport options:\n"
		  "  -A Show raw acceralometer data\n"
		  "  -G Show raw gyroscope data\n"
		  "  -L Show longitude/lateral\n"
		  "  -M Show computed motor\n"
		  "  -Z Show computed vertical accelaration\n"
		  "\nFilter options:\n"
		  "  -g FLOAT_VALUE  Set filter gain to FLOAT_VALUE\n"
		  );
	  exit (1);

	case 'g':	/* next arg is gain */
	  if (--argc <=0)
	    err_quit("-g requires another argument");
	  filter_gain = atof(*++argv);
	  break;

	case 'A':
	  show_flags |= SHOW_RAW_ACC;
	  break;

	case 'G':
	  show_flags |= SHOW_RAW_GYRO;
	  break;

	case 'L':
	  show_flags |= SHOW_LL;
	  break;

	case 'M':
	  show_flags |= SHOW_MOTOR;
	  break;

	case 'V':
	  show_flags |= SHOW_VIEWER;
	  break;

	case 'Z':
	  show_flags |= SHOW_ACCZ;
	  break;

	default:
	  err_quit("illegal option. Try -h");
	}

  /*
   * Open a UDP socket (an Internet datagram socket).
   */

  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
      fprintf (stderr, "server: can't open datagram socket");
      exit (1);
    }

  option = 1;
  setsockopt (sockfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

  bzero ((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = htonl (INADDR_ANY);
  serv_addr.sin_port = htons (MYECHO_PORT);

  if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    {
      fprintf (stderr, "server: can't bind local address");
      exit (1);
    }

  dreck (sockfd);
  /* NOTREACHED */
}
