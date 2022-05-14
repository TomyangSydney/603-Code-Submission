/*************************************************************************/
/* File:        project4.c                                               */
/* Description: User project #4 - empty project directory for project    */
/*              developement                                             */
/* Date:        01-2015                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "../include/Xkw/Xkw.h"
#include "../include/roger.h"
#include "../include/simulate.h"
#include "../include/control.h"
#include "../include/modes.h"
#include "../include/RLToolKit.h"
// #include "../project3-Vision/project3.c"

/***********************************************************************/
/* default parameters for the state-action value table used to compose */
/* skills as sequential decision to engage primative actions           */
/*    all actions/skills return status values in {NOREF, !CONV, COV}   */
/*    where NOREF = 0, !CONV=1, CONV=2                                 */
#define NACTIONS_4 2
#define NSTATES_4  9    // 3^(NACTIONS_4)

// only used if using the RL toolkit
#define Q_TABLE_FILE "q_tables/....q_table_%d.txt"
#define CONVG_ACT 0
// end RLToolKit parameters

double proj_four_q_table[NSTATES_4][NACTIONS_4];
/**************************************************************/
void print_errors();
int average_red_pixel();
int sample_gaze_direction();
int Search(Robot* roger, double errors[], double time);
int Track(Robot* roger, double errors[], double time);
int SearchTrack(Robot* roger, double errors[], double time);
// /* 
// * Primitive Skill prototype function.
// */


int Search(roger, errors, time)
Robot* roger;
double errors[NDOF];
double time;
{
	// printf("Hello \n");
	printf("Search called, errors passed in is:  \n");
	print_errors(errors);
	int i;
	int search_status = NO_REFERENCE;
	static double heading;
	int sample_result = 0;
	double base_error = 0.0; // roger->base_setpoint[THETA] - roger->base_position[THETA];
	// printf("base set point is: %f \n", roger->base_setpoint[THETA]);
	// printf("heading is: %f \n", heading);
	double eye_error = 0.0;

	// still in the transient to the last sample direction
	// if (((heading - roger->base_position[THETA]) >=  0.01) || ((heading - roger->base_position[THETA]) <= -0.01)) {
	if (fabs(roger->base_setpoint[THETA] - roger->base_position[THETA]) > 0.1) {
		printf("Search in transient. \n");
		search_status = TRANSIENT;
		base_error = heading - roger->base_position[THETA]; // roger->base_setpoint[THETA] - roger->base_position[THETA];
		printf("In transient, sample heading is: %f   error is: %f \n", heading, base_error);
		while (base_error > M_PI) {
			base_error -= 2*M_PI;
		}

		while (base_error < -M_PI) {
			base_error += 2*M_PI;
		}
		eye_error = base_error/2;
		errors[1] = base_error;
		errors[2] = eye_error;
		errors[3] = eye_error;
		printf("In search: error array is: \n");
		print_errors(errors); 
		return search_status;

	// if the last search finished, set the setpoint and prepare to do a new search
	}else {
		// printf("here, heading error is: %f \n", abs(heading - roger->base_position[THETA]));
		double* headingPtr = &heading;
		sample_result = sample_gaze_direction(headingPtr);
		if (sample_result == FALSE) {
			printf("Something wrong in sampe_gaze_direction. \n");
		}
		printf("In Search, do a sampling. \n");
		base_error = *headingPtr - roger->base_position[THETA];
		while (base_error > M_PI) {
			base_error -= 2*M_PI;
		}

		while (base_error < -M_PI) {
			base_error += 2*M_PI;
		}
		errors[1] = base_error;
		errors[2] = base_error/2;
		errors[3] = base_error/2;
		search_status = CONVERGED; 
	}
	// printf("In search: error array is: \n");
	// print_errors(errors); 

	printf("Search finished, errors obtained is: . \n");
	print_errors(errors); 

	// DO STUFF
	return search_status;
}

// int Search(roger, errors, time)
// Robot* roger;
// double errors[NDOF];
// double time;
// {
// 	int search_status = NO_REFERENCE;
// 	printf("Search called, errors passed in is:  \n");
// 	print_errors(errors);
// 	double heading = 0.0;
// 	int sample_result = 0;
// 	double base_error = roger->base_setpoint[THETA] - roger->base_position[THETA];
// 	double base_diff = 0.0;

// 	// ensure the base error is in the range.
// 	while (base_error > M_PI) {
// 		base_error -= 2*M_PI;
// 	}

// 	while (base_error < -M_PI) {
// 		base_error += 2*M_PI;
// 	}

// 	errors[1] = base_error;
// 	errors[2] = base_error/2;
// 	errors[3] = base_error/2;


// 	// if the base error is less or equal to 0.01
// 	// treat it as converge
// 	if (fabs(base_error) <= 0.01) {
// 		printf("In Search function, Search converged! Do a new sampling. \n");
// 		double* headingPtr = &heading;
// 		sample_result = sample_gaze_direction(headingPtr);
// 		if (sample_result == FALSE) {
// 			printf("Something wrong in sampe_gaze_direction. \n");
// 		}
// 		printf("In Search, do a sampling. \n");
// 		base_error = *headingPtr - roger->base_position[THETA];

		
// 		search_status = CONVERGED;
// 	}

// 	if (search_status == CONVERGED) {

// 	}
// }



int Track(roger, errors, time) 
Robot* roger;
double errors[NDOF];
double time;
{
	printf("Track called, errors passed in is: \n");
	print_errors(errors);
	static int track_status = NO_REFERENCE;
	double base_error = 0.0;
	int ul = -1, ur = -1;
	ul = average_red_pixel(roger, LEFT, roger->image[LEFT]);
	ur = average_red_pixel(roger, RIGHT, roger->image[RIGHT]);
	printf("ul is: %d,   ur is: %d  \n", ul, ur);
	if (ul == -1 || ur == -1) {
		// printf("track no reference \n");
		// printf("In track: error array is: \n");
		// print_errors(errors); 
		return NO_REFERENCE;
	}

	double left_diff_angle = atan2((double)(NPIXELS/2 - ul), FOCAL_LENGTH);
	double right_diff_angle = atan2((double)(NPIXELS/2 - ur), FOCAL_LENGTH);
	// errors[1] = (roger->eyes_setpoint[LEFT] + roger->eyes_setpoint[RIGHT])/2 - roger->base_position[THETA];
	// printf("left and right eye error is: %f, %f \n", left_diff_angle, right_diff_angle);
	errors[2] = -left_diff_angle;
	errors[3] = -right_diff_angle;

	double base_adjustment = 0.0;
	// if the eyes have been moved to the desired position (almost there) 
	// if ((errors[2] >= -0.1 && errors[2] <= 0.1) || (errors[3] >= -0.1 && errors[3] <= 0.1)) {
	// printf("Left eye angle : %f   Right eye angle: %f \n", roger->eye_theta[LEFT], roger->eye_theta[RIGHT]);
	// printf("Roger heading is: %f \n", roger->base_position[THETA]);

	// 	base_adjustment = -roger->eye_theta[LEFT];

	// 	// if (roger->base_position[THETA] >= 0 && roger->base_position[THETA] <= M_PI) {
	// 	// 	base_adjustment = -roger->eye_theta[LEFT];
	// 	// }else if (roger->base_position[THETA] >= -M_PI && roger->base_position[THETA] < 0) {
	// 	// 	base_adjustment = -roger->eye_theta[LEFT];
	// 	// }
	// 	// base_adjustment = roger->eye_theta[LEFT];
	// }

	if (roger->base_position[THETA] >= 0 && roger->base_position[THETA] <= M_PI) {
		base_adjustment = roger->eye_theta[LEFT];
	}else if (roger->base_position[THETA] >= -M_PI && roger->base_position[THETA] < 0) {
		base_adjustment = -roger->eye_theta[LEFT];
	}
	

	// double next_base_direction = roger->base_position[THETA] + base_adjustment;
	// base_error = next_base_direction - roger->base_position[THETA] + (roger->theta[LEFT] + roger->theta[RIGHT])/2;
	base_error = (roger->eye_theta[LEFT] + roger->eye_theta[RIGHT])/2;
	double next_base_direction = roger->base_position[THETA] + base_error; 

	// while (next_base_direction > M_PI) {
	// 	next_base_direction -= 2*M_PI;
	// }

	// while (next_base_direction < -M_PI) {
	// 	next_base_direction += 2*M_PI;
	// }


	// base_error = next_base_direction - roger->base_position[THETA];

	while (base_error < -M_PI) {
		base_error += 2*M_PI;
	}

	while (base_error > M_PI) {
		base_error -= 2*M_PI;
	}
	errors[1] = base_error;
	// printf("base orientation is: %f \n", roger->base_position[THETA]);
	// roger->eyes_setpoint[LEFT] = roger->eye_theta[LEFT] - left_diff_angle;
	// roger->eyes_setpoint[RIGHT] = roger->eye_theta[RIGHT] - right_diff_angle;
	// roger->base_setpoint[THETA] = roger->base_position[THETA] + (left_diff_angle + right_diff_angle)/2;

	// if (roger->eyes_setpoint[LEFT] != roger->eye_theta[LEFT]) {
	// 	return TRANSIENT;
	// }else if (roger->eyes_setpoint[RIGHT] != roger->eye_theta[RIGHT]) {
	// 	return TRANSIENT;
	// }else if (roger->base_setpoint[THETA] != roger->eye_theta[THETA]) {
	// 	return TRANSIENT;
	// }

	if (fabs(errors[2]) <= 0.01 && fabs(errors[3]) <= 0.01) {
		// printf("track converged. \n");
		track_status = CONVERGED;
	}else {
		// printf("track in transient. \n");
		track_status = TRANSIENT;
	}

	printf("Track finished, errors array is: \n");
	print_errors(errors); 

	return track_status;
}

/********************************** Behavorial-Build File Function Prototypes ****************************/
/* 
* Learned/Hand-Designed Policy Execution Action Function Prototype
*     use your learned or hand-designed policy array in this function
*     for future functions to call.
*/
int SearchTrack(roger, errors, time)
Robot* roger;
double errors[NDOF];
double time;
{
	printf("In SearchTrack, errors passed in is:  \n");
	print_errors(errors);
	int i, state, return_state, internal_state[NACTIONS_4]; 
	// // double search_errors[NDOF], track_errors[NDOF];

	// action_errors[0] for track error, action_errors[1] for search error
	double action_errors[NACTIONS_4][NDOF] = {0.0};
	static int initialized = 0;
	int selected_action;

	// arrays to hold the action and reward functions (rewards are still tracked for debugging)
	static int (*actions[NACTIONS_4])(Robot* roger, double errors[NDOF], double time);

	if (initialized == 0) {
		// need to assign actions to the array indicies 
		//    NOTE: order matters w.r.t. how state is calculated
		actions[0] = Track;
		actions[1] = Search;
  		// ...
  		// Load the learned q-table - if skill was learned through RL....
		// LoadLearnedQTable(..., proj_X_q_table, NSTATES_4, NACTIONS_4, Q_TABLE_FILE);
     
  		// ... OR - define the q-table by hand
		// proj_four_q_table[j][0] for track, proj_four_q_table[j][1] for search

		// state = 0, do search
		proj_four_q_table[0][0] = 0.0;
		proj_four_q_table[0][1] = 1.0;

		// state = 1 and 2, do track
		proj_four_q_table[1][0] = 1.0;
		proj_four_q_table[1][1] = 0.0;
		proj_four_q_table[2][0] = 1.0;
		proj_four_q_table[2][1] = 0.0;

		// state = 3, search
		proj_four_q_table[3][0] = 0.0;
		proj_four_q_table[3][1] = 1.0;

		// state = 4, 5; track
		proj_four_q_table[4][0] = 1.0;
		proj_four_q_table[4][1] = 0.0;
		proj_four_q_table[5][0] = 1.0;
		proj_four_q_table[5][1] = 0.0;

		// state = 6; search
		proj_four_q_table[6][0] = 0.0;
		proj_four_q_table[6][1] = 1.0;

		// state = 7,8; track
		proj_four_q_table[7][0] = 1.0;
		proj_four_q_table[7][1] = 0.0;
		proj_four_q_table[8][0] = 1.0;
		proj_four_q_table[8][1] = 0.0;
     	// ...
     	// total_reward_exploit = 0.0;
		initialized = 1;
	}
	
	// calculate the current state based on the avaliable actions
	state = 0;
    // create a defualt set of action parameters
	// printf("action number is: %d\n", NACTIONS_4);
	for (i=0; i<NACTIONS_4; ++i) {
		// printf("i is: %d\n", i);
		internal_state[i] = actions[i](roger, action_errors[i], time);
		// printf("Internal state is: %d \n", internal_state[i]);
		state += pow(3, i)*internal_state[i];
	}

	if (state == 6) {
		// printf("State is: %d \n", state);
	}

	printf("In SearchTrack, State is: %d \n", state);
	// printf("current state is: %d \n", state);
	// get the greedy (largest q-value) action to perform based on the q-table
	selected_action = GetActionGreedy(state, proj_four_q_table, NACTIONS_4);
	printf("In SearchTrack, action index chosen is: %d \n", selected_action);
	copy_errors(action_errors[selected_action], errors);
	// actions[selected_action];

	printf("In SearchTrack, the erros finally obtainbed is: \n");
	print_errors(errors);
	// handle setting the return state
    // TODO: define your own skill's return status based on the state
	if (state == 1 || state == 4 || state == 7) {
		return(TRANSIENT); 
	} else if (state == 2 || state == 5 || state == 8) {
		return(CONVERGED);
	}
	// // ... 
	// printf("In SearchTrack, Searching track finished. \n");
	return(NO_REFERENCE);
}
/********************************************************************************************************/


void project4_control(roger, time)
Robot* roger;
double time;
{ 

	// searchTrack(roger);
	/******** Code outline for testing primitive actions ********/
	// double errors[NDOF] = {0.0};
	// // Action_X(roger, errors, time);
	// Search(roger, errors, time);
	// // Track(roger, errors, time);
	// print_errors(errors);
	// submit_errors(roger, errors);
	/************************************************************/


	/******** Code outline for testing HAND-DESIGNED composite actions ********/
	double errors[NDOF] = {0.0};
	int result = SearchTrack(roger, errors, time);
	if (result == NO_REFERENCE) {
		printf("Search Track in No REF \n");
	}else if (result == TRANSIENT) {
		printf("Search Track in Transient \n");
	}else {
		printf("Search Track in Converged \n");
	}
	// printf("After composite action: error array is: \n");
	// print_errors(errors); 
	submit_errors(roger, errors);
	// printf("errors submitted. \n");
	/************************************************************/

	/******** Code outline for LEARNING/TESTING composite actions ********/
	// NOTE: See programming exercises document, section 7, for detailed explanation of below function and
	//     example reward anf reset functions
	// int (*actions[NACTIONS_4])(Robot* roger, double errors[NDOF], double time);
    // double (*rewards[...])(int state, int previous_state, int previous_action, int internal_state[NSTATES_4]);
    // double alpha=..., gamma=...;
    // int reward_num = ...;
    // double default_reward = ...;
	// double conv_reward = 100;
	// double conv_penalty = -100;             

    // // important note, final eps values is the SUM of eps_min and eps_max
	// //     i.e. eps_min = 0.2 and eps_max = 0.6 results in a final eps value of 0.8
	// //     eps is the probability of a greedy action
    // double eps_min = ..., eps_max = ...;
    // int time_per_episode = ; // ms
    // int num_episodes = ; 

    // // learn a policy
	// // NOTE, DOES NOT construct empty folder inside of q_tables/. If you want to put data in a sub-folder
	// //     create the sub-folder first!	
	// Q_Learning(roger, time, alpha, gamma, actions, rewards, default_reward, NACTIONS_4, NSTATES_4, 
	// 	reward_num, proj_X_q_table, time_per_episode, num_episodes, eps_min, eps_max, CONVG_ACT, conv_reward, conv_penalty, Q_TABLE_FILE,
	// 	..reset_func..);
	/************************************************************/
	
    /******** Code outline for loading and testing previously learned composite actions ********/
  	// static int policy_loaded = 0;
	// int q_table_num = ...;
	
    // // load a learned policy
    // if (policy_loaded == 0) {
	// 	LoadLearnedQTable(q_table_num, proj_X_q_table, NSTATES_4, NACTIONS_4, Q_TABLE_FILE);
	// 	policy_loaded = 1;
	// }

    // // exploit the learned policy
	// Run_Policy(roger, time, actions, rewards, default_reward, NACTIONS_4, NSTATES_4, reward_num, proj_X_q_table);

	// alternatively, you can load the learned file into the above SearchTrack template, create the approperiatre action/reward arrays
	//     and run the action is displayed above for HAND-DESIGNED composite actions
	/************************************************************/

	/******** Code outline for collecting performance data for learned composite actions ********/
	// static int action_calls[NACTIONS_4];

	// // load a learned policy
    // if (policy_loaded == 0) {
	// 	LoadLearnedQTable(q_table_num, proj_X_q_table, NSTATES_4, NACTIONS_4, Q_TABLE_FILE);
	// 	policy_loaded = 1;
	// }

	// // NOTE, DOES NOT construct empty folder inside of perf_data. If you want to pput data in a sub-folder
	// //     create the sub-folder first!
	// generatePerformanceData(roger, time, actions, rewards, default_reward, NACTIONS_4, NSTATES_4, reward_num, proj_X_q_table,
	// "perf_data/TODO/TODO_%s", action_calls, 100, CONVG_ACT, -100, 100, time_per_episode, reset_X); 

	/************************************************************/
}

/************************************************************************/
void project4_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project4_enter_params() 
{
  printf("Project 4 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project4_visualize(roger)
Robot* roger;
{ }



