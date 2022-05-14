/*************************************************************************/
/* File:        project3.c                                               */
/* Description: User project #3 - empty project directory for project    */
/*              development                                             */
/* Date:        01-2015                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
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
#define NACTIONS 2
#define NSTATES  9    // 3^(NACTIONS)

// only used if using the RL toolkit
#define Q_TABLE_FILE ""
#define CONVG_ACT 0
// end RLToolKit parameters


int average_red_pixel(Robot* roger, int index, int image[NPIXELS][NPRIMARY_COLORS]);
double proj_three_q_table[NSTATES][NACTIONS] = {0.0};
/**************************************************************/

// /* 
// * Primitive Skill prototype function.
// */
// int Action_X(roger, errors, time)
// Robot* roger;
// double errors[NUM_DOF];
// double time;
// {
// 	int i;
// 	static int return_status = NO_REFERENCE;
// 	// first, do no harm
//   	for (i=0; i<NUM_DOF; ++i) {
//   		errors[i] = 0.0;
//   	}
// 	// DO STUFF
// 	return(return_status);
// }

void project3_control(roger, time)
Robot* roger;
double time;
{
	// starter code for testing primitive actions
	// double errors[NUM_DOF];

	// Action_X(roger, errors, time);
	// submit_errors(roger, errors);

  int ul = average_red_pixel(roger, LEFT, roger->image[LEFT]);
  int ur = average_red_pixel(roger, RIGHT, roger->image[RIGHT]);
  // printf("ul is %d \n", ul);
  // printf("ur is %d \n", ur);
  if (ul != -1 && ur != -1) {
    roger -> eyes_setpoint[LEFT] = roger -> eye_theta[LEFT] - atan2((double) (NPIXELS/2 - ul), FOCAL_LENGTH);
    roger -> eyes_setpoint[RIGHT] = roger -> eye_theta[RIGHT] - atan2((double) (NPIXELS/2 - ur), FOCAL_LENGTH);
  }else if (ul == -1 || ur == -1) {
    roger->eyes_setpoint[LEFT] = 0;
		roger->eyes_setpoint[RIGHT] = 0;
  }

}

/************************************************************************/
void project3_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project3_enter_params()
{
  printf("Project 3 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project3_visualize(roger)
Robot* roger;
{ }

int average_red_pixel(roger, index, image)
Robot* roger;
int index;
int image[NPIXELS][NPRIMARY_COLORS];
{

  int found = 0;
  int start = -1, end = -1;
  for (int i = 0; i < NPIXELS; i++) {
    if (image[i][RED_CHANNEL] == 255) {
      if (found == 0) {
        found = 1;
        start = i;
      }
        end = i;
    }
  }


 if (found != 0) {
   return (start + end)/2;
 }

 return -1;
}

// double average_red_pixel(roger, index, image)
// Robot* roger;
// int index;
// int image[NPIXELS][NPRIMARY_COLORS];
// {
//     int found = 0;
//     int start = -1, end = -1;
//     double start_pos = 0.0, end_pos = 0.0;
//     double red_pixels_length = 0.0;
//     for (int i = 0; i < NPIXELS; i++) {
//       if (image[i][RED_CHANNEL] == 255) {
//         if (found == 0) {
//           found = 1;
//           start = i;
//           start_pos = (double) start;
//         }
//           end = i;
//           end_pos = (double) end;
//       }
//     }


//   if (found != 0) {
//     return start_pos + (end_pos - start_pos + 1.0)/2.0;
//   }

//   return -1.0;
// }

