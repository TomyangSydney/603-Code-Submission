/*************************************************************************/
/* File:        project7.c                                               */
/* Description: User project #7 - empty project directory for project    */
/*              development                                             */
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
#define NACTIONS 2
#define NSTATES  9    // 3^(NACTIONS)

// only used if using the RL toolkit
#define Q_TABLE_FILE "q_tables/....q_table_%d.txt"
#define CONVG_ACT 0
#define NO_OBJECT 100000
// end RLToolKit parameters
Observation obs;
int SearchTrack();
bool stereo_observation();
int Chase(Robot* roger, double errors[], double time);
int Touch(Robot* roger, double errors[], double time);
void home_arms(), print_errors(), copy_errors(), add_error_arrays();
int inv_arm_kinematics_errors();
void construct_wTb(), matrix_mult(), fwd_arm_kinematics();
double proj_seven_q_table[NSTATES][NACTIONS] = {0.0};

/**************************************************************/

/********************************** Behavorial-Build File Function Prototypes ****************************/
// /* 
// * Learned/Hand-Designed Policy Execution Action Function Prototype
// *     use your learned or hand-designed policy array in this function
// *     for future functions to call.
// */
// int CompositeAction_X(roger, errors, time)
// Robot* roger;
// double errors[NDOF];
// double time;
// {
// 	int i, state, return_state, internal_state[NACTIONS]; 
// 	// double search_errors[NDOF], track_errors[NDOF];
// 	double action_errors[NACTIONS][NDOF];
// 	static int initialized = 0;
// 	int selected_action;

// 	// arrays to hold the action and reward functions (rewards are still tracked for debugging)
// 	static int (*actions[NACTIONS])(Robot* roger, double errors[NDOF], double time);

// 	if (initialized == 0) {
// 		// need to assign actions to the array indicies 
// 		//    NOTE: order matters w.r.t. how state is calculated
// 		actions[0] = Action_X;
// 		actions[1] = ...;
//   		...
//   		// Load the learned q-table - if skill was learned through RL....
// 		LoadLearnedQTable(..., proj_X_q_table, NSTATES, NACTIONS, Q_TABLE_FILE);
     
//   		// ... OR - define the q-table by hand
//      	proj_X_q_table[0][0] = 1.0;
// 		proj_X_q_table[1][1] = 1.0;
// 		proj_X_q_table[2][0] = 1.0;
//      	...
//      	total_reward_exploit = 0.0;
// 		initialized = 1;
// 	}
	
// 	// first do no harm
// 	for (i=0; i<NDOF; ++i) {
// 		errors[i] = 0.0;
// 	}

// 	// calculate the current state based on the avaliable actions
// 	state = 0;
//     // create a defualt set of action parameters
// 	for (i=0; i<NACTIONS; ++i) {
// 		internal_state[i] = actions[i](roger, action_errors[i], time);
// 		state += pow(3, i)*internal_state[i];
// 	}

// 	// get the greedy (largest q-value) action to perform based on the q-table
// 	selected_action = GetActionGreedy(state, proj_..._q_table, NACTIONS);

// 	copy_errors(action_errors[selected_action], errors);

// 	// handle setting the return state
//     // TODO: define your own skill's return status based on the state
// 	if (state >= 1 && ...) {
// 		return(TRANSIENT); 
// 	} else if (...) {
// 		return(CONVERGED);
// 	}
// 	... 
// 	return(NO_REFERENCE);
// }
/********************************************************************************************************/


// /* 
// * Primitive Skill prototype function.
// */
// int Action_X(roger, errors, time)
// Robot* roger;
// double errors[NDOF];
// double time;
// {
// 	int i;
// 	static int return_status = NO_REFERENCE;
// 	// first, do no harm
//   	for (i=0; i<NDOF; ++i) {
//   		errors[i] = 0.0;
//   	}
// 	// DO STUFF
// 	return(return_status);
// }


int Chase(roger, errors, time) 
Robot* roger;
double errors[NDOF];
double time;
{

	printf("Chase called, errors passed in is: .\n");
	// print_errors(errors);

	// for (int i = 0; i < NDOF; i++) {
	// 	errors[i] = 0.0;
	// }
	// printf("SearchTrack called in Chase. \n");
	int search_track_result = SearchTrack(roger, errors, time);
	// printf("In Chase, after searchTrack:  Translation error:  %f     Rotation error:  %f     \n", errors[0], errors[1]);
	// printf("After search track, error is: \n");
	// print_errors(errors);
	
	if ( search_track_result != NO_REFERENCE) {

		if (search_track_result == NO_REFERENCE) {
			// printf("Search Track in No REF \n");
		}else if (search_track_result == TRANSIENT) {
			// printf("Search Track in Transient \n");
		}else {
			// printf("Search Track in Converged \n");
		}
		// printf("In Chase, red ball detected\n");

		// printf("stereo called in Chase. \n");
		if (stereo_observation(roger, &obs, time)) {
			// printf("found ball in Chase. \n");
			double ball_x = obs.pos[X];
			double ball_y = obs.pos[Y];
			
			double error_x = obs.pos[X] - roger->base_position[X];
			// printf("World ball position is:  %f %f\n", obs.pos[X], obs.pos[Y]);
			// printf("World roger position is: %f %f\n", roger->base_position[X], roger->base_position[Y]);
			// printf("error_x: %f \n", error_x);
			double error_y = obs.pos[Y] - roger->base_position[Y];
			// printf("error_y: %f \n", error_y);
			double trans_error = 0.0;
			// double trans_error = error_x*cos(roger->base_position[THETA]) +
			//   error_y*sin(roger->base_position[THETA]);

			// trans_error = error_x*cos(roger->base_position[THETA]) + error_y*sin(roger->base_position[THETA]);


			// if (fabs(error_x) <= fabs(error_y)) {
			// 	trans_error = error_x;
			// }else {
			// 	trans_error = error_y;
			// }

			// double trans_vel = roger->base_velocity[X]*cos(roger->base_position[THETA]) +
			// roger->base_velocity[Y]*sin(roger->base_position[THETA]);
			// double ball_heading = atan2(error_y, error_x);
			// double ball_roger_deading_diff = ball_heading - roger -> base_position[THETA];



			double distance_to_ball = sqrt(error_x*error_x + error_y*error_y);  // trans_error; // sqrt(error_x*error_x + error_y*error_y); 
			printf("In Chase, distance to ball is: %f\n", distance_to_ball); 
			printf("In Chase, angle differnce is: %f \n", fabs(roger->base_setpoint[THETA] - roger->base_position[THETA]));
			if ((distance_to_ball < 1.0) && (fabs(roger->base_setpoint[THETA] - roger->base_position[THETA]) <= 1.0)) {
				printf("In Chase, in CONVERGED.\n");
				errors[0] = 0.0;
				errors[1] = roger->base_setpoint[THETA] - roger->base_position[THETA];
				return CONVERGED;
			}else {
				double angle_diff = atan2(error_y, error_x + 0.001);
				double chasing_x_diff = (distance_to_ball)*cos(angle_diff);
				double chasing_y_diff = (distance_to_ball)*sin(angle_diff);
				errors[0] = sqrt(chasing_x_diff * chasing_x_diff + chasing_y_diff * chasing_y_diff);
				home_arms(roger, errors, time);
				// printf("In Chase, calculate translational error. \n");
			}
			// rotational error
			// roger->base_setpoint[THETA] = atan2(error_y, error_x + 0.0001);
			// printf("base setpoint is: %f \n", roger->base_setpoint[THETA]);
			// double theta_error = roger->base_setpoint[THETA] - roger->base_position[THETA];
			// // theta_error = roger->base_setpoint[THETA] - roger->base_position[THETA];
			// double theta_dot_error = 0.0 - roger->base_velocity[THETA];
			// while (theta_error > M_PI) theta_error -= 2.0 * M_PI;
			// while (theta_error < -M_PI) theta_error += 2.0 * M_PI;
			// printf("theta_error is: %f \n", theta_error);
			// errors[1] = theta_error;

			// if (errors[0] <= 0.2) {
			// 	printf("In chase, converged. \n");
			// 	return CONVERGED;
			// }
		}else {
			// printf("No sterero vision in Chase, errors is \n");
			// print_errors(errors);
			// printf("Eyes see: %f %f \n", obs.pos[X], obs.pos[Y]);
		}
	}else {
		home_arms(roger, errors, time);
		// submit_errors(errors);
		// printf("In Chase, Ball not detected, still searching it, errors obtained is \n");
		// print_errors(errors);
		return NO_REFERENCE;
	}

	return TRANSIENT;
}

int Touch(roger, errors, time)
Robot* roger;
double errors[NDOF];
double time;
{
	printf("Touch called, error passed in is: \n");
	// print_errors(errors);
	static int touch_status;

	// for (int i = 0; i < NDOF; i++) {
	// 	errors[i] = 0.0;
	// }

	// printf("SearchTrack called in Touch. \n");
	int track_result =  SearchTrack(roger, errors, time);
	// printf("After search track, error is: \n");
	// print_errors(errors);
	
	

	if (track_result != NO_REFERENCE) {
		// try to get the ball position in world frame
		// refer the implementation from project 9
		// printf("stereo called in Touch. \n");
		bool obs_result = stereo_observation(roger, &obs, time);
		
		double wTb[4][4] = {0.0}, ref_b_l[4] = {0.0}, ref_w_l[4] = {0.0}, ref_b_r[4] = {0.0}, ref_w_r[4] = {0.0};
		double leftarm_errors[NDOF] = {0.0}, rightarm_errors[NDOF] = {0.0}, botharms_errors[NDOF] = {0.0};
		double errors_copy[NDOF] = {0.0};
		if (obs_result) {
			// check if both arms are in the range
			// printf("Track called in Touch. \n");
			// int track_result = Track(roger, errors, time);

			
			// calculate inv arm kinematics ansd pass it to errors array
			int is_left_in_range = inv_arm_kinematics_errors(roger, LEFT, obs.pos[X], obs.pos[Y], leftarm_errors);
			int is_right_in_range = inv_arm_kinematics_errors(roger, RIGHT, obs.pos[X], obs.pos[Y], rightarm_errors);
			add_error_arrays(leftarm_errors, rightarm_errors, botharms_errors);
			add_error_arrays(botharms_errors, errors, errors);
			// copy_errors(botharms_errors, errors);

			
			if (!is_left_in_range && !is_right_in_range) {
				// printf("In touch, arm out of range. \n");
				// home arms
				home_arms(roger, errors, time);
				touch_status = NO_REFERENCE;
				return NO_REFERENCE;
			}

			touch_status = TRANSIENT;
			construct_wTb(roger->base_position, wTb);

			// convert the base position to world position using HT
			double left_x, left_y, right_x, right_y;
			fwd_arm_kinematics(roger, LEFT, &left_x, &left_y);
			left_y += ARM_OFFSET;
			ref_b_l[0] = left_x;
			ref_b_l[1] = left_y;
			ref_b_l[2] = 0.0;
			ref_b_l[3] = 1.0;
			matrix_mult(4, 4, wTb, 1, ref_b_l, ref_w_l);

			fwd_arm_kinematics(roger, RIGHT, &right_x, &right_y);
			right_y -= ARM_OFFSET;
			ref_b_r[0] = right_x;
			ref_b_r[1] = right_y;
			ref_b_r[2] = 0.0;
			ref_b_r[3] = 1.0;
			matrix_mult(4, 4, wTb, 1, ref_b_r, ref_w_r);
			
			// get the distance between the hand and the ball
			double left_x_diff = ref_w_l[0] - obs.pos[X];
			double left_y_diff = ref_w_l[1] - obs.pos[Y];
			double right_x_diff = ref_w_r[0] - obs.pos[X];
			double right_y_diff = ref_w_r[1] - obs.pos[Y];
			double dist_to_ball_l = sqrt(left_x_diff*left_x_diff + left_y_diff*left_y_diff);
			double dist_to_ball_r = sqrt(right_x_diff*right_x_diff + right_y_diff*right_y_diff);

			// errors[4] = roger->arm_setpoint[LEFT][0] - roger->arm_theta[LEFT][0];
			// errors[5] = roger->arm_setpoint[LEFT][1] - roger->arm_theta[LEFT][1];
			// errors[6] = roger->arm_setpoint[RIGHT][0] - roger->arm_theta[RIGHT][0];
			// errors[7] = roger->arm_setpoint[RIGHT][1] - roger->arm_theta[RIGHT][1];

			// for some reason, two hands might be touched
			// if that case happends, that's not converged.
			double roger_hands_x_diff = left_x - right_x;
			double roger_hands_y_diff = left_y - right_y;
			double hands_distance = sqrt(roger_hands_x_diff*roger_hands_x_diff + roger_hands_y_diff*roger_hands_y_diff);

			// calculate the difference between the roger base and the ball 

			double ball_x = obs.pos[X];
			double ball_y = obs.pos[Y];
			double error_x = obs.pos[X] - roger->base_position[X];
			// printf("World ball position is:  %f %f\n", obs.pos[X], obs.pos[Y]);
			// printf("World roger position is: %f %f\n", roger->base_position[X], roger->base_position[Y]);
			// printf("error_x: %f \n", error_x);
			double error_y = obs.pos[Y] - roger->base_position[Y];
			// printf("error_y: %f \n", error_y);
			double distance_to_ball = sqrt(error_x*error_x + error_y*error_y); 

			if (((fabs(roger->ext_force[LEFT][0]) > 0) || (fabs(roger->ext_force[LEFT][1]) > 0)) && dist_to_ball_l < 0.3){
				touch_status = CONVERGED;
				// printf("Touch converged. \n");
				// if (hands_distance > 0.28) {
				// 	printf("Touch converged. \n");
					
				// }else {
				// 	printf("Left hand touched right hand. \n");
				// }
				
			}else if ((fabs(roger->ext_force[RIGHT][0]) > 0 || fabs(roger->ext_force[RIGHT][1]) > 0) && dist_to_ball_r < 0.3) {
				touch_status = CONVERGED;
				// printf("Touch converged. \n");
				// if (hands_distance > 0.28) {
				// 	printf("Touch converged. \n");
				// 	touch_status = CONVERGED;
				// }else {
				// 	printf("Right hand touched left hand. \n");
				// }
			}
		}else {
			// printf("No sterero vision in Touch, errors is \n");
			// print_errors(errors);
			// printf("Eyes see: %d %d \n", obs.pos[X], obs.pos[Y]);
			home_arms(roger, errors, time);
		}
		
	}else {
		// printf("In touch, can't see the red ball\n");
		home_arms(roger, errors, time);
		// printf("In Touch, Ball not detected, still searching it, errors obtained is \n");
		// print_errors(errors);
	}
	// printf("In Touch part, error is: \n");
	// print_errors(errors);
	return touch_status;
}


int ChaseTouch(roger, errors, time)
Robot* roger;
double errors[NDOF];
double time;
{

	// printf("ChaseTouch called, error passed in is: \n");
	// print_errors(errors);
	int i, state, return_state, internal_state[NACTIONS]; 
	// // double search_errors[NDOF], track_errors[NDOF];

	// action_errors[0] for track error, action_errors[1] for search error
	double action_errors[NACTIONS][NDOF] = {0.0};
	static int initialized = 0;
	int selected_action;

	// arrays to hold the action and reward functions (rewards are still tracked for debugging)
	static int (*actions[NACTIONS])(Robot* roger, double errors[NDOF], double time);

	if (initialized == 0) {
		// need to assign actions to the array indicies 
		//    NOTE: order matters w.r.t. how state is calculated
		actions[0] = Touch;
		actions[1] = Chase;
  		// ...
  		// Load the learned q-table - if skill was learned through RL....
		// LoadLearnedQTable(..., proj_X_q_table, NSTATES_4, NACTIONS_4, Q_TABLE_FILE);
     
  		// ... OR - define the q-table by hand
		// proj_four_q_table[j][0] for track, proj_four_q_table[j][1] for search

		// state = 0, do Chase
		proj_seven_q_table[0][0] = 0.0;
		proj_seven_q_table[0][1] = 1.0;

		// state = 1 and 2, Chase 
		proj_seven_q_table[1][0] = 0.0;
		proj_seven_q_table[1][1] = 1.0;
		proj_seven_q_table[2][0] = 0.0;
		proj_seven_q_table[2][1] = 1.0;

		// state = 3, Chase
		proj_seven_q_table[3][0] = 0.0;
		proj_seven_q_table[3][1] = 1.0;

		// state = 4 Touch
		proj_seven_q_table[4][0] = 0.0;
		proj_seven_q_table[4][1] = 1.0;

		// state = 5, Chase
		proj_seven_q_table[5][0] = 0.0;
		proj_seven_q_table[5][1] = 1.0;

		// state = 6; Chase
		proj_seven_q_table[6][0] = 1.0;
		proj_seven_q_table[6][1] = 0.0;

		// state = 7, Touch
		proj_seven_q_table[7][0] = 1.0;
		proj_seven_q_table[7][1] = 0.0;

		// state = 8, Touch
		proj_seven_q_table[8][0] = 1.0;
		proj_seven_q_table[8][1] = 0.0;
     	// // ...
     	// // total_reward_exploit = 0.0;
		initialized = 1;
	}
	
	// calculate the current state based on the avaliable actions
	state = 0;
    // create a defualt set of action parameters
	// printf("action number is: %d\n", NACTIONS_4);
	for (i=0; i<NACTIONS; ++i) {
		// printf("i is: %d\n", i);
		internal_state[i] = actions[i](roger, action_errors[i], time);
		// printf("Internal state is: %d \n", internal_state[i]);
		state += pow(3, i)*internal_state[i];
	}

	// printf("In ChaseTouch, the State is: %d \n", state);
	// if (state == 6) {
	// 	// printf("State is: %d \n", state);
	// }
	// printf("current state is: %d \n", state);
	// get the greedy (largest q-value) action to perform based on the q-table
	selected_action = GetActionGreedy(state, proj_seven_q_table, NACTIONS);

	// if (selected_action == 1) {
	// 	printf("In ChaseTouch, action chosen is Chase, index chosen is: %d \n", selected_action);
	// }else {
	// 	printf("In ChaseTouch, action chosen is Touch, index chosen is: %d \n", selected_action);
	// }

	// printf("In ChaseTouch, errors obtained from selected action is: \n");
	// print_errors(action_errors[selected_action]);
	// printf("In ChaseTouch, action index chosen is: %d \n", selected_action);
	copy_errors(action_errors[selected_action], errors);
	// actions[selected_action];

	// handle setting the return state
    // TODO: define your own skill's return status based on the state
	if (state == 3 || state == 4 || state == 5 || state == 7) {
		// printf("In ChaseTouch, ChaseTouch is TRANSIENT. \n");
		return(TRANSIENT); 
	} else if (state == 8) {
		// printf("ChaseTouch is CONVERGED. \n");
		return(CONVERGED);
	}
	// // ... 
	// printf("ChaseTouch NO_REFERENCE. \n");
	return(NO_REFERENCE);
}



void project7_control(roger, time)
Robot* roger;
double time;
{
	/******** Code outline for testing primitive actions ********/
	double errors[NDOF] = {0.0};
	ChaseTouch(roger, errors, time);
	//printf("After chase:  Translation error:  %f     Rotation error:  %f     \n", errors[0], errors[1]);
	submit_errors(roger, errors);
	print_errors(errors) ;
	/************************************************************/


	/******** Code outline for testing HAND-DESIGNED composite actions ********/
	// double errors[NDOF];

	// CompositeAction_X(roger, errors, time);
	// submit_errors(roger, errors);
	/************************************************************/

	/******** Code outline for LEARNING/TESTING composite actions ********/
	// NOTE: See programming exercises document, section 7, for detailed explanation of below function and
	//     example reward anf reset functions
	// int (*actions[NACTIONS])(Robot* roger, double errors[NDOF], double time);
    // double (*rewards[...])(int state, int previous_state, int previous_action, int internal_state[NSTATES]);
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
	// Q_Learning(roger, time, alpha, gamma, actions, rewards, default_reward, NACTIONS, NSTATES, 
	// 	reward_num, proj_X_q_table, time_per_episode, num_episodes, eps_min, eps_max, CONVG_ACT, conv_reward, conv_penalty, Q_TABLE_FILE,
	// 	..reset_func..);
	/************************************************************/
	
    /******** Code outline for loading and testing previously learned composite actions ********/
  	// static int policy_loaded = 0;
	// int q_table_num = ...;
	
    // // load a learned policy
    // if (policy_loaded == 0) {
	// 	LoadLearnedQTable(q_table_num, proj_X_q_table, NSTATES, NACTIONS, Q_TABLE_FILE);
	// 	policy_loaded = 1;
	// }

    // // exploit the learned policy
	// Run_Policy(roger, time, actions, rewards, default_reward, NACTIONS, NSTATES, reward_num, proj_X_q_table);

	// alternatively, you can load the learned file into the above CompositeAction_X template, create the approperiatre action/reward arrays
	//     and run the action is displayed above for HAND-DESIGNED composite actions
	/************************************************************/

	/******** Code outline for collecting performance data for learned composite actions ********/
	// static int action_calls[NACTIONS];

	// // load a learned policy
    // if (policy_loaded == 0) {
	// 	LoadLearnedQTable(q_table_num, proj_X_q_table, NSTATES, NACTIONS, Q_TABLE_FILE);
	// 	policy_loaded = 1;
	// }

	// // NOTE, DOES NOT construct empty folder inside of perf_data. If you want to pput data in a sub-folder
	// //     create the sub-folder first!
	// generatePerformanceData(roger, time, actions, rewards, default_reward, NACTIONS, NSTATES, reward_num, proj_X_q_table,
	// "perf_data/TODO/TODO_%s", action_calls, 100, CONVG_ACT, -100, 100, time_per_episode, reset_X); 

	/************************************************************/
}

/************************************************************************/
void project7_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project7_enter_params() 
{
  printf("Project 7 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project7_visualize(roger)
Robot* roger;
{
	draw_observation(roger, obs); 
 }