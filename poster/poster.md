## Approach to solving the obstacles
### SMR-CL
Work on solving the obstacles of the course was divided into two components: *task* and *realrun*.
The *task* script was made for us to work with one obstacle at a time. When one solution was achieved and accepted, it would be appended to *realrun*. *task* would then be cleared and work on the next obstacle would commence. Not having to return to the start of the course for each test saved us a lot of time.
*realrun* is a linear run from the start of the track, that throughout the process implemented the solutions from *task* as they were finished. It was effectively used both to recheck the code of each task, but also the transitions between tasks.

This method worked well with both the simulator and the SMR.

### C
A state machine was used for solving the obstacles with a C program. Functions were created for every type of movement the SMR needed to do, like moving forward, turning, following line and so on. These functions were then used in the state machine, with a new mission for every type of movement. The obstacles were solved one by one and always tested from the start of the track to make sure everything was working and the transitions between obstacles were good.

## Why we won the odometry competition
### UMBmark
By calibrating the SMR with the UMBmark method, we were able to narrow our odometric errors to a total of 36 cm on a test of two 3 meter squares (CW and CCW). This landed us the first place* between the groups running UMBMark via a C program. 

*On the first CW run our SMR ran out of battery, which is why we had to do a rerun with the SMR connected to the charger.

## Follow line
### Center of mass
Using the center of mass theory we created an algorithm for following a line:

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
          p->ctrl_err = -FOLLOWLINE_OFFSET_L - line_center(p->line_type[0]);
        } else if (p->line_type[1] == 'r') {
          p->ctrl_err = FOLLOWLINE_OFFSET_R - line_center(p->line_type[0]);
        } else {
          p->ctrl_err = -line_center(p->line_type[0]);
        }
        
        p->ctrl_der = p->ctrl_err - p->ctrl_err_last;

        if (p->line_type[0] == 'w') {
          p->ctrl_out = KP_FOLLOWLINE_W * p->ctrl_err + KD_FOLLOWLINE_W * p->ctrl_der;
        } else {
          p->ctrl_out = KP_FOLLOWLINE_B * p->ctrl_err + KD_FOLLOWLINE_B * p->ctrl_der;
        }
 
        p->motorspeed_l += p->ctrl_out;
        p->motorspeed_r -= p->ctrl_out;

      }

    break;

## State machine
The state machine is implemented to ...
