/*
 * An example SMR program.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

#define DATA_LOG_ROW 50000
#define DATA_LOG_COL 17

#define MAX_DELTA_VEL 0.005
#define MAX_DECELERATION 0.5

#define K_SPEED 0.0

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct
{
  double x, y, z, omega, phi, kappa, code, id, crc;
} gmk;
double visionpar[10];
double laserpar[10];

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

componentservertype lmssrv, camsrv;

symTableElement *
getinputref(const char *sym_name, symTableElement *tab)
{
  int i;
  for (i = 0; i < getSymbolTableSize('r'); i++)
    if (strcmp(tab[i].name, sym_name) == 0)
      return &tab[i];
  return 0;
}

symTableElement *
getoutputref(const char *sym_name, symTableElement *tab)
{
  int i;
  for (i = 0; i < getSymbolTableSize('w'); i++)
    if (strcmp(tab[i].name, sym_name) == 0)
      return &tab[i];
  return 0;
}

/*** ODOMETRY ***/

/*
#define WHEEL_SEPARATION 0.252
#define WHEEL_DIAMETER 0.067
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define CORRECTION 1.0
*/

/*
#define WHEEL_SEPARATION 0.26
#define DELTA_M 0.00010245
#define CORRECTION 1.0
*/

#define WHEEL_SEPARATION 0.264190
#define DELTA_M 0.000103
#define CORRECTION 0.997080

#define CL (DELTA_M / CORRECTION)
#define CR (DELTA_M * CORRECTION)

#define ROBOTPORT 24902

typedef struct
{                          //input signals
  int left_enc, right_enc; // encoderticks
  // parameters
  double w;      // wheel separation
  double cr, cl; // meters per encodertick
                 //output signals
  double right_pos, left_pos;
  // internal variables
  int left_enc_old, right_enc_old;
  // Odometry
  double th, th_ref, x, y;
} odotype;

void reset_odo(odotype *p);
void update_odo(odotype *p);

/********************************************
* Motion control
*/

typedef struct
{ //input
  int cmd;
  int curcmd;
  double speedcmd;
  double dist;
  double angle;
  double left_pos, right_pos;
  double th, th_ref;
  // parameters
  double w;
  //output
  double motorspeed_l, motorspeed_r;
  int finished;
  // internal variables
  double startpos, startth;
} motiontype;

enum
{
  mot_stop = 1,
  mot_move,
  mot_turn
};

void update_motcon(motiontype *p);

int fwd(double dist, double speed, int time);
int turn(double angle, double speed, int time);

typedef struct
{
  int state, oldstate;
  int time;
} smtype;

void sm_update(smtype *p);

// SMR input/output data

symTableElement *inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr, *resetmotorr, *resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum
{
  ms_init,
  ms_fwd,
  ms_turn,
  ms_end
};

int main()
{
  int running, arg, time = 0, index = 0, i, j, k;
  int n = 0;
  double dist = 0;
  //double vel=0;
  double angle = 0;
  double data_log[DATA_LOG_ROW][DATA_LOG_COL] = {{0}};
  FILE *data_file;

  /* Establish connection to robot sensors and actuators.
   */
  if (rhdConnect('w', "localhost", ROBOTPORT) != 'w')
  {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }

  printf("connected to robot \n");
  if ((inputtable = getSymbolTable('r')) == NULL)
  {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }
  if ((outputtable = getSymbolTable('w')) == NULL)
  {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }
  // connect to robot I/O variables
  lenc = getinputref("encl", inputtable);
  renc = getinputref("encr", inputtable);
  linesensor = getinputref("linesensor", inputtable);
  irsensor = getinputref("irsensor", inputtable);

  speedl = getoutputref("speedl", outputtable);
  speedr = getoutputref("speedr", outputtable);
  resetmotorr = getoutputref("resetmotorr", outputtable);
  resetmotorl = getoutputref("resetmotorl", outputtable);
  // **************************************************
  //  Camera server code initialization
  //

  /* Create endpoint */
  lmssrv.port = 24919;
  strcpy(lmssrv.host, "127.0.0.1");
  strcpy(lmssrv.name, "laserserver");
  lmssrv.status = 1;
  camsrv.port = 24920;
  strcpy(camsrv.host, "127.0.0.1");
  camsrv.config = 1;
  strcpy(camsrv.name, "cameraserver");
  camsrv.status = 1;

  if (camsrv.config)
  {
    int errno = 0;
    camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (camsrv.sockfd < 0)
    {
      perror(strerror(errno));
      fprintf(stderr, " Can not make  socket\n");
      exit(errno);
    }

    serverconnect(&camsrv);

    xmldata = xml_in_init(4096, 32);
    printf(" camera server xml initialized \n");
  }

  // **************************************************
  //  LMS server code initialization
  //

  /* Create endpoint */
  lmssrv.config = 1;
  if (lmssrv.config)
  {
    char buf[256];
    int errno = 0, len;
    lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (lmssrv.sockfd < 0)
    {
      perror(strerror(errno));
      fprintf(stderr, " Can not make  socket\n");
      exit(errno);
    }

    serverconnect(&lmssrv);
    if (lmssrv.connected)
    {
      xmllaser = xml_in_init(4096, 32);
      printf(" laserserver xml initialized \n");
      /*len = sprintf(buf, "push  t=0.2 cmd='mrcobst width=0.4'\n");*/
      len = sprintf(buf, "scanpush cmd='zoneobst'\n");
      send(lmssrv.sockfd, buf, len, 0);
    }
  }

  /* Read sensors and zero our position.
   */
  rhdSync();

  odo.w = WHEEL_SEPARATION;
  odo.cr = CR;
  odo.cl = CL;
  odo.left_enc = lenc->data[0];
  odo.right_enc = renc->data[0];
  reset_odo(&odo);
  printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
  mot.w = odo.w;
  mot.th_ref = odo.th_ref;
  running = 1;
  mission.state = ms_init;
  mission.oldstate = -1;
  while (running)
  {
    if (lmssrv.config && lmssrv.status && lmssrv.connected)
    {
      while ((xml_in_fd(xmllaser, lmssrv.sockfd) > 0))
        xml_proca(xmllaser);
    }

    if (camsrv.config && camsrv.status && camsrv.connected)
    {
      while ((xml_in_fd(xmldata, camsrv.sockfd) > 0))
        xml_proc(xmldata);
    }

    rhdSync();
    odo.left_enc = lenc->data[0];
    odo.right_enc = renc->data[0];
    update_odo(&odo);

    /****************************************
/ mission statemachine   
*/
    sm_update(&mission);

    switch (mission.state)
    {
    case ms_init:
      n = 4;
      dist = 1;
      angle = -M_PI / 2.0;
      mission.state = ms_fwd;
      break;

    case ms_fwd:
      if (fwd(dist, 0.3, mission.time))
        mission.state = ms_turn;
      break;

    case ms_turn:
      if (turn(angle, 0.3, mission.time))
      {
        n = n - 1;
        if (n == 0)
          mission.state = ms_end;
        else
          mission.state = ms_fwd;
      }
      break;

    case ms_end:
      mot.cmd = mot_stop;
      running = 0;
      break;
    }

    /*switch (mission.state) {
     
     case ms_init:
       dist=2.00; vel=0.2;
       mission.state= ms_fwd;      
     break;
     
     case ms_fwd:
       if (fwd(dist,vel,mission.time)) {mission.state=ms_end;}
     break;   
     
     case ms_end:
       mot.cmd=mot_stop;
       running=0;
     break;
     
   }*/

    data_log[index][0] = index;
    data_log[index][1] = mission.time;
    data_log[index][2] = mot.motorspeed_l;
    data_log[index][3] = mot.motorspeed_r;
    data_log[index][4] = odo.x;
    data_log[index][5] = odo.y;
    data_log[index][6] = odo.th;
    for (k = 0; k < 10; k++) {
      data_log[index][7+k] = laserpar[k];
    }
    index++;

    /*  end of mission  */

    mot.left_pos = odo.left_pos;
    mot.right_pos = odo.right_pos;
    mot.th = odo.th;
    update_motcon(&mot);
    speedl->data[0] = 100 * mot.motorspeed_l;
    speedl->updated = 1;
    speedr->data[0] = 100 * mot.motorspeed_r;
    speedr->updated = 1;
    if (time % 100 == 0)
      //    printf(" laser %f \n",laserpar[3]);
      time++;
    /* stop if keyboard is activated
*
*/
    ioctl(0, FIONREAD, &arg);
    if (arg != 0)
      running = 0;

  } /* end of main control loop */

  data_file = fopen("data_log.dat", "w");
  if (data_file == NULL)
  {
    printf("Error opening log file!\n");
    exit(1);
  }
  fprintf(data_file, "index,mission.time,mot.motorspeed_l,mot.motorspeed_r,odo.x,odo.y,odo.th,laserpar[0],laserpar[1],laserpar[2],laserpar[3],laserpar[4],laserpar[5],laserpar[6],laserpar[7],laserpar[8],laserpar[9]\n");
  for (i = 0; i < index; i++)
  {
    for (j = 0; j < DATA_LOG_COL; j++)
    {
      fprintf(data_file, "%g,", data_log[i][j]);
    }
    fprintf(data_file, "\n");
  }
  fclose(data_file);

  speedl->data[0] = 0;
  speedl->updated = 1;
  speedr->data[0] = 0;
  speedr->updated = 1;
  rhdSync();
  rhdDisconnect();
  exit(0);
}

/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */

void reset_odo(odotype *p)
{
  p->right_pos = 0.0;
  p->left_pos = 0.0;
  p->x = 0.0;
  p->y = 0.0;
  p->th = 0.0;
  p->th_ref = 0.0;
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;
}

void update_odo(odotype *p)
{
  int delta;
  double delta_right_pos, delta_left_pos;

  delta = p->right_enc - p->right_enc_old;
  if (delta > 0x8000)
    delta -= 0x10000;
  else if (delta < -0x8000)
    delta += 0x10000;
  p->right_enc_old = p->right_enc;
  delta_right_pos = delta * p->cr;
  p->right_pos += delta_right_pos;

  delta = p->left_enc - p->left_enc_old;
  if (delta > 0x8000)
    delta -= 0x10000;
  else if (delta < -0x8000)
    delta += 0x10000;
  p->left_enc_old = p->left_enc;
  delta_left_pos = delta * p->cl;
  p->left_pos += delta_left_pos;

  p->x += (delta_right_pos + delta_left_pos) * cos(p->th) / 2.0;

  p->y += (delta_right_pos + delta_left_pos) * sin(p->th) / 2.0;

  p->th += (delta_right_pos - delta_left_pos) / p->w;

  //if (p->th > 0) {p->th = fmod(p->th + M_PI, 2.0 * M_PI) - M_PI;} else {p->th = fmod(p->th - M_PI, 2.0 * M_PI) + M_PI;}

  //printf("(%f)(%f)(%f)\n", p->th, p->x, p->y);
}

void update_motcon(motiontype *p)
{
  
  double max_speed, delta_speed;

  if (p->cmd != 0)
  {

    p->finished = 0;
    switch (p->cmd)
    {
    case mot_stop:
      p->curcmd = mot_stop;
      break;
    case mot_move:
      p->startpos = (p->left_pos + p->right_pos) / 2;
      p->curcmd = mot_move;
      break;

    /*case mot_turn:
         if (p->angle > 0) 
	    p->startpos=p->right_pos;
	 else
	    p->startpos=p->left_pos;
         p->curcmd=mot_turn;
       break;*/

    case mot_turn:
      p->th_ref += p->angle; 
      p->startth = p->th;
      p->curcmd = mot_turn;
      break;
    }

    p->cmd = 0;
  }

  switch (p->curcmd)
  {
    
  case mot_stop:
    p->motorspeed_l = 0;
    p->motorspeed_r = 0;
    break;
    
  case mot_move:

    if ((p->right_pos + p->left_pos) / 2.0 - p->startpos > p->dist) {
      
      p->finished = 1;
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
      
    } else {

      if (p->speedcmd - p->motorspeed_l > MAX_DELTA_VEL) {p->motorspeed_l += MAX_DELTA_VEL;} else {p->motorspeed_l = p->speedcmd;}
      if (p->speedcmd - p->motorspeed_r > MAX_DELTA_VEL) {p->motorspeed_r += MAX_DELTA_VEL;} else {p->motorspeed_r = p->speedcmd;}

      max_speed = sqrt(2.0 * MAX_DECELERATION * (p->dist - (p->right_pos + p->left_pos) / 2.0 - p->startpos));
      if (p->motorspeed_l > max_speed) {p->motorspeed_l = max_speed;}
      if (p->motorspeed_r > max_speed) {p->motorspeed_r = max_speed;}
      
      delta_speed = K_SPEED * (p->th_ref - p->th);
      printf("(%f)(%f)\n", delta_speed, p->th_ref);
      p->motorspeed_l -= delta_speed;
      p->motorspeed_r += delta_speed;
      
    }
    
    break;

  case mot_turn:
    if (p->angle > 0)
    {
      if (p->th - p->startth < p->angle)
      {

        if (p->speedcmd + p->motorspeed_l > MAX_DELTA_VEL)
        {
          p->motorspeed_l -= MAX_DELTA_VEL;
        }
        else
        {
          p->motorspeed_l = -p->speedcmd;
        }
        if (p->speedcmd - p->motorspeed_r > MAX_DELTA_VEL)
        {
          p->motorspeed_r += MAX_DELTA_VEL;
        }
        else
        {
          p->motorspeed_r = p->speedcmd;
        }

        max_speed = sqrt(2.0 * MAX_DECELERATION * fabs(p->angle - (p->th - p->startth)) * p->w / 2.0);
        if (-p->motorspeed_l > max_speed)
        {
          p->motorspeed_l = -max_speed;
        }
        if (p->motorspeed_r > max_speed)
        {
          p->motorspeed_r = max_speed;
        }
      }
      else
      {
        printf("(%f)(%f)\n", p->th - p->startth, p->angle);
        p->motorspeed_r = 0;
        p->motorspeed_l = 0;
        p->finished = 1;
      }
    }
    else
    {
      if (p->th - p->startth > p->angle)
      {

        if (p->speedcmd - p->motorspeed_l > MAX_DELTA_VEL)
        {
          p->motorspeed_l += MAX_DELTA_VEL;
        }
        else
        {
          p->motorspeed_l = p->speedcmd;
        }
        if (p->speedcmd + p->motorspeed_r > MAX_DELTA_VEL)
        {
          p->motorspeed_r -= MAX_DELTA_VEL;
        }
        else
        {
          p->motorspeed_r = -p->speedcmd;
        }

        max_speed = sqrt(2.0 * MAX_DECELERATION * fabs(p->angle - (p->th - p->startth)) * p->w / 2.0);
        if (p->motorspeed_l > max_speed)
        {
          p->motorspeed_l = max_speed;
        }
        if (-p->motorspeed_r > max_speed)
        {
          p->motorspeed_r = -max_speed;
        }
      }
      else
      {
        p->motorspeed_r = 0;
        p->motorspeed_l = 0;
        p->finished = 1;
      }
    }
    break;
  }
}

int fwd(double dist, double speed, int time)
{
  if (time == 0)
  {
    mot.cmd = mot_move;
    mot.speedcmd = speed;
    mot.dist = dist;
    return 0;
  }
  else
    return mot.finished;
}

int turn(double angle, double speed, int time)
{
  if (time == 0)
  {
    mot.cmd = mot_turn;
    mot.speedcmd = speed;
    mot.angle = angle;
    return 0;
  }
  else
    return mot.finished;
}

void sm_update(smtype *p)
{
  if (p->state != p->oldstate)
  {
    p->time = 0;
    p->oldstate = p->state;
  }
  else
  {
    p->time++;
  }
}