/*************************************************************************/
/* File:        project5.c                                               */
/* Description: User project #5 - empty project directory for project    */
/*              developement                                             */
/* Date:        01-2015                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include "Xkw/Xkw.h"
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "RLToolKit.h"

/***********************************************************************/
/* default parameters for the state-action value table used to compose */
/* skills as sequential decision to engage primative actions           */
/*    all actions/skills return status values in {NOREF, !CONV, COV}   */
/*    where NOREF = 0, !CONV=1, CONV=2                                 */
#define NACTIONS 1
#define NSTATES  3    // 3^(NACTIONS)

// only used if using the RL toolkit
#define Q_TABLE_FILE "q_tables/....q_table_%d.txt"
#define CONVG_ACT 0
#define NO_OBJECT 100000
// end RLToolKit parameters
// Observation obs;
int average_red_pixel();


double proj_five_q_table[NSTATES][NACTIONS] = {0.0};
/**************************************************************/
// Observation stereo_observation(roger, obs, time)
// Robot* roger;
// Observation* obs; 
// double time;
// {
//   int ul_pixel = average_red_pixel(roger, LEFT, roger->image);
//   int ur_pixel = average_red_pixel(roger, RIGHT, roger->image);
//   // double ul = 0.0;
//   // double ur = 0.0; 
//   // double phi_l = 0.0, phi_r = 0.0;
//   // double gamma_l = 0.0 , gamma_r = 0.0;

//   if (ul_pixel == -1 || ur_pixel == -1) {
//     // roger -> eyes_setpoint[LEFT] = 0.0;
//     // roger -> eyes_setpoint[RIGHT] = 0.0;
//     // // printf("triggered");
//     obs->pos[0] = NO_OBJECT;
//     obs->pos[1] = NO_OBJECT;
//     obs->cov[0][0] = 0.0;
//     obs->cov[0][1] = 0.0;
//     obs->cov[1][0] = 0.0;
//     obs->cov[1][1] = 0.0;
//     obs->time = time;

//   }else {
//      // get the left and right value: ul and ur
//     double ul = (double)(NPIXELS/2 -  ul_pixel);
//     double ur = (double)(NPIXELS/2 -  ur_pixel);
//     double phi_l = atan2(ul, FOCAL_LENGTH);
//     double phi_r = atan2(ur, FOCAL_LENGTH);
//     // roger -> eyes_setpoint[LEFT] = roger -> eye_theta[LEFT] + phi_l;
//     // roger -> eyes_setpoint[RIGHT] = roger -> eye_theta[RIGHT] + phi_r;
//     double gamma_l = phi_l + roger->eye_theta[LEFT];
//     double gamma_r = phi_r + roger->eye_theta[RIGHT];

//     double wTb[4][4], ref_b[4], ref_w[4], J_b[2][2], J_w[2][2], J_wT[2][2], JJT[2][2], R[2][2];
//     construct_wTb(roger->base_position, wTb);

//     ref_b[0] = 2*BASELINE*cos(gamma_r)*cos(gamma_l)/sin(gamma_r - gamma_l + 0.001);
//     ref_b[1] = BASELINE + 2*BASELINE*cos(gamma_r)*sin(gamma_l)/sin(gamma_r - gamma_l + 0.001);
//     ref_b[2] = 0.0;
//     ref_b[3] = 1.0;

//     double scaling = 0.005;
//     double J_mult = 2*BASELINE/(sin(gamma_r - gamma_l + 0.001)*sin(gamma_r - gamma_l + 0.001));

//     J_b[0][0] = J_mult*cos(gamma_r)*cos(gamma_r);
//     J_b[0][1] = -1*J_mult*cos(gamma_l)*cos(gamma_l);
//     J_b[1][0] = J_mult*sin(gamma_r)*cos(gamma_r);
//     J_b[1][1] = -1*J_mult*sin(gamma_l)*cos(gamma_l);

//     R[0][0] = wTb[0][0];
//     R[0][1] = wTb[0][1];
//     R[1][0] = wTb[1][0];
//     R[1][1] = wTb[1][1];

//     matrix_mult(4, 4, wTb, 1, ref_b, ref_w);
//     printf("world position is: %f , %f \n", ref_w[0], ref_w[1]);
//     matrix_mult(2, 2, R, 2, J_b, J_w);

//     J_wT[0][0] = J_w[0][0];
//     J_wT[0][1] = J_w[1][0];
//     J_wT[1][0] = J_w[0][1];
//     J_wT[1][1] = J_w[1][1];

//     matrix_mult(2, 2, J_w, 2, J_wT, JJT);

//     obs->pos[0] = ref_w[0];
//     obs->pos[1] = ref_w[1];
//     obs->cov[0][0] = JJT[0][0]*scaling;
//     obs->cov[0][1] = JJT[0][1]*scaling;
//     obs->cov[1][0] = JJT[1][0]*scaling;
//     obs->cov[1][1] = JJT[1][1]*scaling;
//     obs->time = time;
//   }
// 	return obs;
// }


bool stereo_observation(roger, obs, time)
Robot* roger;
Observation* obs; 
double time;
{
  int ul_pixel = average_red_pixel(roger, LEFT, roger->image[LEFT]);
  int ur_pixel = average_red_pixel(roger, RIGHT, roger->image[RIGHT]);
  // double ul = 0.0;
  // double ur = 0.0; 
  // double phi_l = 0.0, phi_r = 0.0;
  // double gamma_l = 0.0 , gamma_r = 0.0;

  if (ul_pixel == -1 || ur_pixel == -1) {
    // roger -> eyes_setpoint[LEFT] = 0.0;
    // roger -> eyes_setpoint[RIGHT] = 0.0;
    // // printf("triggered");
    obs->pos[0] = NO_OBJECT;
    obs->pos[1] = NO_OBJECT;
    // obs->cov[0][0] = 0.0;
    // obs->cov[0][1] = 0.0;
    // obs->cov[1][0] = 0.0;
    // obs->cov[1][1] = 0.0;
    // obs->time = time;
    printf("No stereo vision in stereo_observation function\n");
    return false;

  }else {
     // get the left and right value: ul and ur
    double ul = ((double)NPIXELS/2 -  ul_pixel);
    double ur = ((double)NPIXELS/2 -  ur_pixel);
    double phi_l = atan2(ul, FOCAL_LENGTH);
    double phi_r = atan2(ur, FOCAL_LENGTH);
    // roger -> eyes_setpoint[LEFT] = roger -> eye_theta[LEFT] + phi_l;
    // roger -> eyes_setpoint[RIGHT] = roger -> eye_theta[RIGHT] + phi_r;
    double gamma_l = phi_l + roger->eye_theta[LEFT];
    double gamma_r = phi_r + roger->eye_theta[RIGHT];

    double wTb[4][4] = {0.0}, ref_b[4] = {0.0}, ref_w[4] = {0.0}, J_b[2][2] = {0.0}, J_w[2][2] = {0.0}, J_wT[2][2] = {0.0}, JJT[2][2] = {0.0}, R[2][2] = {0.0};
    construct_wTb(roger->base_position, wTb);

    ref_b[0] = 2*BASELINE*cos(gamma_r)*cos(gamma_l)/sin(gamma_r - gamma_l + 0.000001);
    ref_b[1] = BASELINE + 2*BASELINE*cos(gamma_r)*sin(gamma_l)/sin(gamma_r - gamma_l + 0.000001);
    ref_b[2] = 0.0;
    ref_b[3] = 1.0;

    // double scaling = 2*BASELINE/(sin(gamma_r - gamma_l)*sin(gamma_r - gamma_l));
    double J_mult = 2*BASELINE/(sin(gamma_r - gamma_l)*sin(gamma_r - gamma_l) + 0.00001);
    double scaling = 0.005;
    J_b[0][0] = J_mult*cos(gamma_r)*cos(gamma_r);
    J_b[0][1] = -J_mult*cos(gamma_l)*cos(gamma_l);
    J_b[1][0] = J_mult*sin(gamma_r)*cos(gamma_r);
    J_b[1][1] = -J_mult*sin(gamma_l)*cos(gamma_l);

    R[0][0] = wTb[0][0];
    R[0][1] = wTb[0][1];
    R[1][0] = wTb[1][0];
    R[1][1] = wTb[1][1];

    matrix_mult(4, 4, wTb, 1, ref_b, ref_w);
    matrix_mult(2, 2, R, 2, J_b, J_w);
    matrix_transpose(2, 2, J_w, J_wT);
    matrix_mult(2, 2, J_w, 2, J_wT, JJT);

    obs->pos[0] = ref_w[0];
    obs->pos[1] = ref_w[1];
    obs->cov[0][0] = JJT[0][0]*scaling;
    obs->cov[0][1] = JJT[0][1]*scaling;
    obs->cov[1][0] = JJT[1][0]*scaling;
    obs->cov[1][1] = JJT[1][1]*scaling;
    obs->time = time;
  }
	return true;
}



void project5_control(roger, time)
Robot* roger;
double time;
{ 
  Observation obs;
  stereo_observation(roger, &obs, time);
}

/************************************************************************/
void project5_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project5_enter_params() 
{
  printf("Project 5 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project5_visualize(roger)
Robot* roger;
{
  Observation obs;
	draw_observation(roger, obs); /* defined in update.c */
}
