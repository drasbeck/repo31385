/*** GROUP 13 SMR PROGRAM ***/

/* Includes */
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

/* Acceleration control parameters */
#define MAX_DELTA_VEL 0.005
#define MAX_DECELERATION 0.5

/* PD motion controller parameters */
#define KP_FWD 0.01
#define KP_FOLLOWLINE 0.2
#define KD_FOLLOWLINE 0.1
#define KP_FOLLOWWALL 0.5
#define KD_FOLLOWWALL 0.01

/* Parameters for line following */
#define FOLLOWLINE_OFFSET 1.0

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct {
  double x, y, z, omega, phi, kappa, code, id, crc;
} gmk;
double visionpar[10];
double laserpar[10];

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

componentservertype lmssrv, camsrv;

symTableElement *
getinputref(const char *sym_name, symTableElement *tab) {
  int i;
  for (i = 0; i < getSymbolTableSize('r'); i++)
    if (strcmp(tab[i].name, sym_name) == 0)
      return &tab[i];
  return 0;
}

symTableElement *
getoutputref(const char *sym_name, symTableElement *tab) {
  int i;
  for (i = 0; i < getSymbolTableSize('w'); i++)
    if (strcmp(tab[i].name, sym_name) == 0)
      return &tab[i];
  return 0;
}

/*** ODOMETRY ***/

/* Original odomotry parameters
#define WHEEL_SEPARATION 0.252
#define WHEEL_DIAMETER 0.067
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define CORRECTION 1.0 */

/* Simulator odometry parameters */
#define WHEEL_SEPARATION 0.26
#define DELTA_M 0.00010245
#define CORRECTION 1.0

/* Robot (SMR 9) parameters */
/*#define WHEEL_SEPARATION 0.261930
#define DELTA_M 0.000103
#define CORRECTION 1.000449*/

/* Odometry corrections */
#define CL (DELTA_M / CORRECTION)
#define CR (DELTA_M * CORRECTION)

/*** COMMUNICATION ***/

#define ROBOTPORT 24902

typedef struct {
  /* Input Signals */
  int left_enc, right_enc; // Encoder Ticks
  /* Parameters */
  double w; // Wheel separation
  double cr, cl; // Meters per encodertick
  /* Output Signals */
  double right_pos, left_pos;
  /* internal Variables */
  int left_enc_old, right_enc_old;
  /* Odometry */
  double th, th_ref, x, y;
} odotype;

void reset_odo(odotype *p);
void update_odo(odotype *p);

/*** MOTION CONTROL ***/

typedef struct {
  /* Inputs */
  int cmd;
  int curcmd;
  double speedcmd;
  double dist;
  double angle;
  double left_pos, right_pos;
  double th, th_ref;
  char line_type[2];
  int ir_number;
  /* Parameters */
  double w;
  /* Outputs */
  double motorspeed_l, motorspeed_r;
  int finished;
  /* Internal Variables */
  double startpos, startth;
  double ctrl_err, ctrl_err_last, ctrl_der, ctrl_out;
  double wall_dist;
  char line_colors[8];
} motiontype;

enum {
  mot_stop = 1,
  mot_move,
  mot_followline,
  mot_followwall,
  mot_turn
};

void update_motcon(motiontype *p);

int fwd(double dist, double speed, int time);
int followline(char line_type[], double dist, double speed, int time);
int followwall(int ir_number, double dist, double speed, int time);
int turn(double angle, double speed, int time);

double line_center(char line_color);
double ir_distance(int ir_number);
char line_color(int line_number);
int crossing_black_line(motiontype *p);

typedef struct {
  int state, oldstate;
  int time;
} smtype;

void sm_update(smtype *p);

/*** SMR INPUT AND OUTPUT DATA */

symTableElement *inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr, *resetmotorr, *resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum {
  ms_init,
  ms_fwd,
  ms_followline,
  ms_followwall,
  ms_turn,
  ms_end,
  /**/
  ms_measBox,
  ms_moveBoxTurn1,
  ms_moveBoxFwd1,
  ms_moveBoxFwd2,
  ms_moveBoxTurn2,
  ms_moveBoxFollow1,
  ms_moveBoxFwd3,
  ms_boxGateBwd1,
  ms_boxGateTurn1,
  ms_boxGateFwd1,
  ms_boxGateFwd2,
  ms_boxGateTurn2,
  ms_boxGateFwd3,
  ms_boxGateFwd4,
  ms_boxGateTurn3,
  ms_boxGateFwd5,
  ms_boxGateFwd6,
  ms_boxGateFwd7,
  ms_boxGateFwd8,
  ms_looseGateFwd1,
  ms_looseGateFwd2,
  ms_looseGateTurn1,
  ms_looseGateFwd3,
  ms_looseGateTurn2,
  ms_looseGateFwd4,
  ms_looseGateTurn3,
  ms_looseGateFwd5,
  ms_looseGateFwd6,
  ms_looseGateTurn4,
  ms_looseGateFwd7,
  ms_looseGateFwd8,
  ms_looseGateTurn5,
  ms_wallFwd1
};

int main()
{
  int running, arg, time = 0, i;
  int crossingblackline = 0;
  int n = 0;
  /*double angle = 0;*/
  /*double dist = 0;
  double vel=0;*/
  FILE *log_file_p;
  
  /*** OPENING LOG FILE ***/
  
  log_file_p = fopen("log.dat", "w");
  if (log_file_p == NULL) {
    printf("Error opening log file!\n");
    exit(1);
  }

  /*** CONNECTION TO SENSORS ANS ACTUATORS ***/
  
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

  /* READ SENSOR AND ZERO POSITION */
  
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
    
    crossingblackline = crossing_black_line(&mot);
    
    update_odo(&odo);

    /*** MISSION STATE MACHINE ***/
    
    sm_update(&mission);
    
    /* Track mission */
    switch (mission.state) {
      
      case ms_init:
	mission.state = ms_measBox;      
      break;
      
      case ms_measBox:
	if (followline("br",2.50,0.3,mission.time) || crossingblackline) {
	  mot.cmd = mot_stop;
	  mission.state=ms_moveBoxTurn1;
	  printf("Distance to box: %f\n", (fabs(odo.y) + 0.255 + laserpar[4]));
	}
      break;
      
      case ms_moveBoxTurn1:
	if (turn(M_PI/2.0, 0.3, mission.time)) {
	  n = 2;
	  mission.state=ms_moveBoxFwd1;
	}
      break;
      
      case ms_moveBoxFwd1:
	if (fwd(1.00,0.3,mission.time) || crossingblackline) {
	  mot.cmd = mot_stop;
	  mission.state=ms_moveBoxFwd2;
	}
      break;
      
      case ms_moveBoxFwd2:
	if (fwd(0.255,0.3,mission.time)) {
	  mot.cmd = mot_stop;
	  n--;
	  if (n == 0) {
	    mission.state=ms_moveBoxTurn2;
	  } else {
	    mission.state=ms_moveBoxFwd1;
	  }
	}
      break;
      
      case ms_moveBoxTurn2:
	if (turn(-M_PI/2.0, 0.3, mission.time)) {
	  mission.state=ms_moveBoxFollow1;
	}
      break;
      
      case ms_moveBoxFollow1:
	if (followline("bm",1.25,0.3,mission.time) || crossingblackline) {
	  mot.cmd = mot_stop;
	  mission.state=ms_moveBoxFwd3;
	}
      break;
      
      case ms_moveBoxFwd3:
	if (followline("bm",0.12,0.3,mission.time)) {
	  mot.cmd = mot_stop;
	  mission.state = ms_boxGateBwd1;
	}
      break;
      
      case ms_boxGateBwd1:
	if (fwd(1.10,-0.3,mission.time)) {
	  mot.cmd = mot_stop;
	  mission.state = ms_boxGateTurn1;
	}
      break;
      
      case ms_boxGateTurn1:
	if (turn(-M_PI/2.0, 0.3, mission.time)) {
	  mission.state=ms_boxGateFwd1;
	}
      break;
      
      case ms_boxGateFwd1:
	if (fwd(1.50,0.3,mission.time) || crossingblackline) {
	  mot.cmd = mot_stop;
	  mission.state=ms_boxGateFwd2;
	}
      break;
      
      case ms_boxGateFwd2:
	if (fwd(0.255,0.3,mission.time)) {
	  mot.cmd = mot_stop;
	  mission.state=ms_boxGateTurn2;
	}
      break;
      
      case ms_boxGateTurn2:
	if (turn(M_PI/2.0, 0.3, mission.time)) {
	  mission.state=ms_boxGateFwd3;
	}
      break;
      
      case ms_boxGateFwd3:
	if (followline("bm",1.50,0.3,mission.time) || crossingblackline) {
	  mot.cmd = mot_stop;
	  mission.state=ms_boxGateFwd4;
	}
      break;
      
      case ms_boxGateFwd4:
	if (fwd(0.255,0.3,mission.time)) {
	  mot.cmd = mot_stop;
	  mission.state=ms_boxGateTurn3;
	}
      break;
      
      case ms_boxGateTurn3:
	if (turn(M_PI/2.0, 0.3, mission.time)) {
	  mission.state=ms_boxGateFwd5;
	}
      break;
      
      case ms_boxGateFwd5:
	if (followline("bm",0.50,0.3,mission.time) || crossingblackline) {
	  mot.cmd = mot_stop;
	  mission.state=ms_boxGateFwd6;
	}
      break;
      
      case ms_boxGateFwd6:
	if (fwd(0.10,0.3,mission.time)) {
	  mot.cmd = mot_stop;
	  mission.state=ms_boxGateFwd7;
	}
      break;
      
      case ms_boxGateFwd7:
	if (followline("br",1.00,0.3,mission.time)) {
	  mot.cmd = mot_stop;
	  mission.state=ms_boxGateFwd8;
	}
      break;
      
      case ms_boxGateFwd8:
	if (followline("bm",1.00,0.3,mission.time) || crossingblackline) {
	  mot.cmd = mot_stop;
	  mission.state=ms_looseGateFwd1;
	}
      break;
      
      case ms_looseGateFwd1:
	if (followline("bm",2.00,0.3,mission.time) || laserpar[0] < 0.65) {
	  mot.cmd = mot_stop;
	  mission.state=ms_looseGateFwd2;
	}
      break;
      
      case ms_looseGateFwd2:
	if (followline("bm",0.65,0.3,mission.time)) {
	  mot.cmd = mot_stop;
	  mission.state=ms_looseGateTurn1;
	}
      break;
      
      case ms_looseGateTurn1:
	if (turn(M_PI/2.0, 0.3, mission.time)) {
	  mission.state=ms_looseGateFwd3;
	}
      break;
      
      case ms_looseGateFwd3:
	if (fwd(1.00,0.3,mission.time)) {
	  mot.cmd = mot_stop;
	  mission.state=ms_looseGateTurn2;
	}
      break;
      
      case ms_looseGateTurn2:
	if (turn(M_PI/2.0, 0.3, mission.time)) {
	  mission.state=ms_looseGateFwd4;
	}
      break;
      
      case ms_looseGateFwd4:
	if (fwd(0.70,0.3,mission.time)) {
	  mot.cmd = mot_stop;
	  mission.state=ms_looseGateTurn3;
	}
      break;
      
      case ms_looseGateTurn3:
	if (turn(M_PI/2.0, 0.3, mission.time)) {
	  mission.state=ms_looseGateFwd5;
	}
      break;
      
      case ms_looseGateFwd5:
	if (fwd(2.00,0.3,mission.time) || crossingblackline) {
	  mot.cmd = mot_stop;
	  mission.state=ms_looseGateFwd6;
	}
      break;
      
      case ms_looseGateFwd6:
	if (fwd(0.235,0.3,mission.time)) {
	  mot.cmd = mot_stop;
	  mission.state=ms_looseGateTurn4;
	}
      break;
      
      case ms_looseGateTurn4:
	if (turn(M_PI/2.0, 0.3, mission.time)) {
	  mission.state=ms_looseGateFwd7;
	}
      break;
      
      case ms_looseGateFwd7:
	if (followline("bm",1.00,0.3,mission.time) || crossingblackline) {
	  mot.cmd = mot_stop;
	  mission.state=ms_looseGateFwd8;
	}
      break;
      
      case ms_looseGateFwd8:
	if (fwd(0.235,0.3,mission.time)) {
	  mot.cmd = mot_stop;
	  mission.state=ms_looseGateTurn5;
	}
      break;
      
      case ms_looseGateTurn5:
	if (turn(M_PI/2.0, 0.3, mission.time)) {
	  mission.state=ms_wallFwd1;
	}
      break;
      
      case ms_wallFwd1:
	if (followline("bm",2.00,0.3,mission.time) || laserpar[0] < 0.30) {
	  mot.cmd = mot_stop;
	  mission.state=ms_end;
	}
      break;
      
      case ms_end:
	mot.cmd=mot_stop;
	running=0;
      break;
      
    }

    /*switch (mission.state)
    {
    case ms_init:
      n = 4;
      dist = 3.0;
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
    }*/

    /*switch (mission.state) {
     
     case ms_init:
       mission.state= ms_fwd;      
     break;
     
     case ms_fwd:
       if (fwd(1.00,0.3,mission.time)) {mission.state=ms_end;}
     break;   
     
     case ms_end:
       mot.cmd=mot_stop;
       running=0;
     break;
    
    }*/
    
    /*switch (mission.state) {
      
      case ms_init:
	mission.state= ms_followline;      
      break;
      
      case ms_followline:
	if (followline("br",2.00,0.3,mission.time)) {mission.state=ms_end;}
      break;   
      
      case ms_end:
	mot.cmd=mot_stop;
	running=0;
      break;
      
    }*/
    
    /*switch (mission.state) {
      
      case ms_init:
	mission.state= ms_followwall;      
      break;
      
      case ms_followwall:
	if (followwall(0,4.00,0.3,mission.time)) {mission.state=ms_end;}
      break;   
      
      case ms_end:
	mot.cmd=mot_stop;
	running=0;
      break;
      
    }*/
    
    /* Write to log file */
    fprintf(log_file_p, "%d,", mission.time);
    fprintf(log_file_p, "%g,", mot.motorspeed_l);
    fprintf(log_file_p, "%g,", mot.motorspeed_r);
    fprintf(log_file_p, "%g,", odo.x);
    fprintf(log_file_p, "%g,", odo.y);
    fprintf(log_file_p, "%g,", odo.th);
    for (i = 0; i < 10; i++) {
      fprintf(log_file_p, "%g,", laserpar[i]);
    }
    for (i = 0; i < 8; i++) {
      fprintf(log_file_p, "%g,", (double)(linesensor->data[i]));
    }
    for (i = 0; i < 5; i++) {
      fprintf(log_file_p, "%g,", ir_distance(i));
    }
    fprintf(log_file_p, "\n");
    
    /*for (i = 0; i < 8; i++) {
      printf("(%c)", line_color(i));
    }
    printf("[%d]", crossingblackline);*/ 
    /*for (i = 0; i < 10; i++) {
      printf("(%f)", laserpar[i]);
    }
    printf("\n");*/

    /*  END OF MISSION  */

    mot.left_pos = odo.left_pos;
    mot.right_pos = odo.right_pos;
    mot.th = odo.th;
    update_motcon(&mot);
    speedl->data[0] = 100 * mot.motorspeed_l;
    speedl->updated = 1;
    speedr->data[0] = 100 * mot.motorspeed_r;
    speedr->updated = 1;
    if (time % 100 == 0) {time++;}
    
    /* stop if keyboard is activated */
    ioctl(0, FIONREAD, &arg);
    if (arg != 0) {running = 0;}

    /* end of main control loop */
    
  }

  /* Close log file */
  fclose(log_file_p);

  /* Stop everything */
  speedl->data[0] = 0;
  speedl->updated = 1;
  speedr->data[0] = 0;
  speedr->updated = 1;
  rhdSync();
  rhdDisconnect();
  exit(0);
  
}

/*** ODOMETRY FUNCTIONS ***/

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
  if (delta > 0x8000) {delta -= 0x10000;} else if (delta < -0x8000) {delta += 0x10000;}
  p->right_enc_old = p->right_enc;
  delta_right_pos = delta * p->cr;
  p->right_pos += delta_right_pos;

  delta = p->left_enc - p->left_enc_old;
  if (delta > 0x8000) {delta -= 0x10000;} else if (delta < -0x8000) {delta += 0x10000;}
  p->left_enc_old = p->left_enc;
  delta_left_pos = delta * p->cl;
  p->left_pos += delta_left_pos;

  p->x += (delta_right_pos + delta_left_pos) * cos(p->th) / 2.0;
  p->y += (delta_right_pos + delta_left_pos) * sin(p->th) / 2.0;
  p->th += (delta_right_pos - delta_left_pos) / p->w;

}

/*** MOTION STATE MACHINE ***/

void update_motcon(motiontype *p) {

  double max_speed;

  if (p->cmd != 0) {

    p->finished = 0;

    switch (p->cmd) {

      case mot_stop:
        p->curcmd = mot_stop;
        break;

      case mot_move:
        p->startpos = (p->left_pos + p->right_pos) / 2;
        p->curcmd = mot_move;
        break;

      case mot_followline:
        p->startpos = (p->left_pos + p->right_pos) / 2;
        p->curcmd = mot_followline;
	p->ctrl_err = 0.0;
        break;

      case mot_followwall:
        p->startpos = (p->left_pos + p->right_pos) / 2;
        p->curcmd = mot_followwall;
	p->wall_dist = ir_distance(p->ir_number);
	p->ctrl_err = 0.0;
        break;

      case mot_turn:
        p->th_ref = p->th + p->angle; 
        p->startth = p->th;
        p->curcmd = mot_turn;
        break;

    }

    p->cmd = 0;

  }

  switch (p->curcmd) {
    
    case mot_stop:

      p->motorspeed_l = 0;
      p->motorspeed_r = 0;

      break;
      
    case mot_move:
      
      //printf("(%f)(%f)\n", (p->right_pos + p->left_pos) / 2.0 - p->startpos, p->dist);

      if (fabs((p->right_pos + p->left_pos) / 2.0 - p->startpos) > p->dist) {
        
        p->finished = 1;
        p->motorspeed_l = 0;
        p->motorspeed_r = 0;
        
      } else {

        if (p->speedcmd - p->motorspeed_l > MAX_DELTA_VEL) {p->motorspeed_l += MAX_DELTA_VEL;} else {p->motorspeed_l = p->speedcmd;}
        if (p->speedcmd - p->motorspeed_r > MAX_DELTA_VEL) {p->motorspeed_r += MAX_DELTA_VEL;} else {p->motorspeed_r = p->speedcmd;}

        max_speed = sqrt(2.0 * MAX_DECELERATION * (p->dist - (p->right_pos + p->left_pos) / 2.0 - p->startpos));
        if (p->motorspeed_l > max_speed) {p->motorspeed_l = max_speed;}
        if (p->motorspeed_r > max_speed) {p->motorspeed_r = max_speed;}
        
        p->ctrl_err = p->th_ref - p->th;
        p->ctrl_out = KP_FWD * p->ctrl_err;
        
        p->motorspeed_l -= p->ctrl_out;
        p->motorspeed_r += p->ctrl_out;
        
      }
      
      break;
      
    case mot_followline:

      if ((p->right_pos + p->left_pos) / 2.0 - p->startpos > p->dist) {
        
        p->finished = 1;
        p->motorspeed_l = 0;
        p->motorspeed_r = 0;
        
      } else {

	p->motorspeed_l = p->speedcmd;
	p->motorspeed_r = p->speedcmd;

	p->ctrl_err_last = p->ctrl_err;
	if (p->line_type[1] == 'l') {
	  p->ctrl_err = -FOLLOWLINE_OFFSET - line_center(p->line_type[0]);
	} else if (p->line_type[1] == 'r') {
	  p->ctrl_err = FOLLOWLINE_OFFSET - line_center(p->line_type[0]);
	} else {
	  p->ctrl_err = -line_center(p->line_type[0]);
	}
	p->ctrl_der = p->ctrl_err - p->ctrl_err_last;
	p->ctrl_out = KP_FOLLOWLINE * p->ctrl_err + KD_FOLLOWLINE * p->ctrl_der;
 
	p->motorspeed_l += p->ctrl_out;
	p->motorspeed_r -= p->ctrl_out;
        
      }
      
      break;
      
    case mot_followwall:

      if ((p->right_pos + p->left_pos) / 2.0 - p->startpos > p->dist) {
        
        p->finished = 1;
        p->motorspeed_l = 0;
        p->motorspeed_r = 0;
        
      } else {

	p->motorspeed_l = p->speedcmd;
	p->motorspeed_r = p->speedcmd;

	p->ctrl_err_last = p->ctrl_err;
	p->ctrl_err = p->wall_dist - ir_distance(p->ir_number);
	p->ctrl_der = p->ctrl_err - p->ctrl_err_last;
	p->ctrl_out = KP_FOLLOWWALL * p->ctrl_err + KD_FOLLOWWALL * p->ctrl_der;

	p->motorspeed_l += p->ctrl_out;
	p->motorspeed_r -= p->ctrl_out;
        
      }
      
      break;

    case mot_turn:

      if (p->angle > 0) {

        if (p->th - p->startth < p->angle) {

          if (p->speedcmd + p->motorspeed_l > MAX_DELTA_VEL) {p->motorspeed_l -= MAX_DELTA_VEL;} else {p->motorspeed_l = -p->speedcmd;}
          if (p->speedcmd - p->motorspeed_r > MAX_DELTA_VEL) {p->motorspeed_r += MAX_DELTA_VEL;} else {p->motorspeed_r = p->speedcmd;}

          max_speed = sqrt(2.0 * MAX_DECELERATION * fabs(p->angle - (p->th - p->startth)) * p->w / 2.0);
          if (-p->motorspeed_l > max_speed) {p->motorspeed_l = -max_speed;}
          if (p->motorspeed_r > max_speed) {p->motorspeed_r = max_speed;}

        } else {

          p->motorspeed_r = 0;
          p->motorspeed_l = 0;
          p->finished = 1;

        }

      } else {

        if (p->th - p->startth > p->angle) {

          if (p->speedcmd - p->motorspeed_l > MAX_DELTA_VEL) {p->motorspeed_l += MAX_DELTA_VEL;} else {p->motorspeed_l = p->speedcmd;}
          if (p->speedcmd + p->motorspeed_r > MAX_DELTA_VEL) {p->motorspeed_r -= MAX_DELTA_VEL;} else {p->motorspeed_r = -p->speedcmd;}

          max_speed = sqrt(2.0 * MAX_DECELERATION * fabs(p->angle - (p->th - p->startth)) * p->w / 2.0);
          if (p->motorspeed_l > max_speed) {p->motorspeed_l = max_speed;}
          if (-p->motorspeed_r > max_speed) {p->motorspeed_r = -max_speed;}

        } else {

          p->motorspeed_r = 0;
          p->motorspeed_l = 0;
          p->finished = 1;

        }

      }

      break;

  }

}

/*** MOTION CONTROL FUNCTIONS ***/

int fwd(double dist, double speed, int time) {
  if (time == 0) {
    mot.cmd = mot_move;
    mot.speedcmd = speed;
    mot.dist = dist;
    return 0;
  } else {
    return mot.finished;
  }
}

int followline(char line_type[], double dist, double speed, int time) {
  if (time == 0) {
    mot.cmd = mot_followline;
    mot.speedcmd = speed;
    mot.dist = dist;
    mot.line_type[0] = line_type[0];
    mot.line_type[1] = line_type[1];
    return 0;
  } else {
    return mot.finished;
  }
}

int followwall(int ir_number, double dist, double speed, int time) {
  if (time == 0) {
    mot.cmd = mot_followwall;
    mot.speedcmd = speed;
    mot.dist = dist;
    mot.ir_number = ir_number;
    return 0;
  } else {
    return mot.finished;
  }
}

int turn(double angle, double speed, int time) {
  if (time == 0) {
    mot.cmd = mot_turn;
    mot.speedcmd = speed;
    mot.angle = angle;
    return 0;
  } else {
    return mot.finished;
  }
}

void sm_update(smtype *p) {
  if (p->state != p->oldstate) {
    p->time = 0;
    p->oldstate = p->state;
  } else {
    p->time++;
  }
}

/*** LINE SENSOR FUNCTIONS ***/

double line_center(char line_color) {

  int i;
  double intensity;
  double num = 0.0, den = 0.0;
  
  /* Line sensor values for SMR 9 */
  /*const double min[8] = {49.7129, 50.0733, 50.1517, 49.8031, 49.5481, 49.8077, 49.6494, 52.5971};
  const double max[8] = {58.6394, 60.3545, 66.0494, 59.5392, 58.8640, 59.1455, 60.1955, 81.7896};*/
  
  /* Line sensor values for simulation */
  const double min[8] = {85.0000, 85.0000, 85.0000, 85.0000, 85.0000, 85.0000, 85.0000, 85.0000};
  const double max[8] = {255.0000, 255.0000, 255.0000, 255.0000, 255.0000, 255.0000, 255.0000, 255.0000};

  for (i = 0; i < 8; i++) {
    if (line_color == 'w') {
      intensity = ((double)(linesensor->data[i]) - min[i]) / (max[i] - min[i]);
    } else {
      intensity = 1.0 - ((double)(linesensor->data[i]) - min[i]) / (max[i] - min[i]);
    }
    num += (-3.5 + (double)(i)) * intensity;
    den += intensity;
  }

  return num / den;

}

char line_color(int line_number) {
  
  /* Line sensor limits for robot (SMR 9) */
  /*double wlim[8] = {58.2510, 59.2777, 62.9864, 58.7009, 58.0611, 58.1362, 59.1064, 73.4506}; // White-white*/
  /*double wlim[8] = {56.8812, 58.2390, 62.2036, 57.5726, 56.9525, 57.2897, 58.0907, 73.4007}; // Dirty-white
  double blim[8] = {53.2119, 54.4847, 57.6007, 53.7704, 53.1544, 53.5702, 54.0679, 66.6741};*/
  
  /* Line sensor limits for simulation */
  double wlim[8] = {198.3333, 198.3333, 198.3333, 198.3333, 198.3333, 198.3333, 198.3333, 198.3333};
  double blim[8] = {141.6667, 141.6667, 141.6667, 141.6667, 141.6667, 141.6667, 141.6667, 141.6667};
  
  
  if ((double)(linesensor->data[line_number]) > wlim[line_number]) {
    return 'w';
  } else if ((double)(linesensor->data[line_number]) < blim[line_number]) {
    return 'b';
  } else {
    return 'f';
  }
  
}

int crossing_black_line(motiontype *p) {
  
  int i, current, count = 0;
  
  for (i = 0; i < 8; i++) {
    current = line_color(i);
    if ((current == 'b') && (p->line_colors[i] == current)) {
      count++;
    }
    p->line_colors[i] = current;
  }
  
  if (count > 5) {
    return 1;
  } else {
    return 0;
  }

}

/*** IR SENSOR FUNCTIONS ***/

double ir_distance(int ir_number) {
  
  const double ka[5] = {14.2600, 13.5608, 13.4456, 14.0818, 13.5900};
  const double kb[5] = {57.4200, 38.1084, 64.3544, 55.2223, 72.6000};
  
  return ka[ir_number] / ((double)(irsensor->data[ir_number]) - kb[ir_number]);
  
}