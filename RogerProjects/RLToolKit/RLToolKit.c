/*************************************************************************/
/* File:        RLToolKit.c                                              */
/* Description: Set of Functions to Perform\Evaluate Q-Learning problems */
/*                  for the Roger-the-Crab robot simulator               */
/* Date:        06-2021                                                  */  
/* author:      Oscar Youngquist                                         */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"
#include <stdio.h>
#include <stdlib.h>

#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "RLToolKit.h"

#define NACTIONS 2
void submit_errors();
int reset_ball_training();
void print_errors();
/* 
* Helper function to check if a file exists before opening it
*/
int file_exists(const char *filename)
{
    FILE *fp = fopen(filename, "r");
    int is_exist = 0;
    if (fp != NULL)
    {
        is_exist = 1;
        fclose(fp); // close the file
    }
    return is_exist;
}


/* 
* Helper function to saves the current policy to
*     POLICY_PATH.txt
*/
void SaveQTable(int fileNum, double *q_table, int numStates, int num_actions, char *fileName) {
	int i, j; 
	char buf[100];
	memset(buf, '\0', sizeof(buf));
	// for testing
	sprintf(buf, fileName, fileNum);
	printf("Saving Q-table: %s\n", buf);
    FILE* fp = fopen(buf, "w");
	if (!fp) {
		printf("Error saving policy to file%s\n", buf);
	} else {
		for (i = 0; i < numStates; ++i) {
			for (j = 0; j < num_actions-1; ++j) {
				fprintf(fp, "%.4f, ", q_table[i*num_actions + j]);
			}
			fprintf(fp, "%.4f\n", q_table[i*num_actions + (num_actions-1)]);
		}
		fclose(fp);
	}
}

/* 
* Helper function to saves the current policy to
*     POLICY_PATH.txt
*/
void SavePolicy(int fileNum, int policy[], int numStates, char *fileName) {
	int i; 
	char buf[100];
	memset(buf, '\0', sizeof(buf));
	sprintf(buf, fileName, fileNum);
	printf("Saving Policy: %s\n", buf);
    FILE* fp = fopen(buf, "w");
	if (!fp) {
		printf("Error saving policy to file%s\n", buf);
	} else {
		for (i = 0; i < numStates; i++) {
			fprintf(fp, "%d ", policy[i]);
		}
		fclose(fp);
	}
}

/* 
* Helper function to load the specified policy
*     
*/
void LoadLearnedPolicy(int fileNum, int policy[], int numStates, char *fileName) {
	int i; 
	char buf[sizeof(fileName) + 50];
	memset(buf, '\0', sizeof(buf));
	sprintf(buf, fileName, fileNum);
	printf("Opening Policy: %s\n", buf);
	FILE* fp = fopen(buf, "r");
	if (!fp) {
		printf("Error loading policy file");
	} else {
		for (i = 0; i < numStates; i++) {
            fscanf(fp, "%d ", &policy[i]);
			printf("policy[%d]: %d\n", i, policy[i]);
		}

		fclose(fp);
	}
}


/* 
* Helper function to load the specified learned q-table
*     
*/
void LoadLearnedQTable(int fileNum, double *q_table, int numStates, int num_actions, char *fileName) {
	int i, j; 
	char buf[sizeof(fileName) + 50];
	memset(buf, '\0', sizeof(buf));
	sprintf(buf, fileName, fileNum);
	printf("Opening Policy: %s\n", buf);
	FILE* fp = fopen(buf, "r");

	
	if (!fp) {
		printf("Error loading policy file");
	} else {
		
		
		for (i = 0; i < numStates; i++) {

			char line[4098];
			fgets(line, 4098, fp);

			// remove the newline
			line[strcspn(line, "\n")] = 0;

			// printf("%s\n", line);

			char* token = strtok(line, ","); 
			for (j=0; j < num_actions; j++) {
				// printf("%s\n", token);
				if (token != NULL) {
					q_table[i*num_actions + j] = atof(token);
					printf("q_table[%d][%d]: %.4f\n", i, j, q_table[i*num_actions + j]);
				}
				token = strtok(NULL, ", ");
			}
		}
		fclose(fp);
	}
}

/* 
* Helper function to generate a random integer between [0, max_num]
*/
int RandomInt(int max_num) {
	int random_int = 0;
	double random_double = 0;

	// printf("inside of randomint\n");
	// printf("%d\n", max_num);

	if (max_num > 0) {
		random_double = ((double)rand() / (double)RAND_MAX) * (double)(max_num + 1);
		random_int = floor(random_double);

		if (random_int > max_num) {
			random_int = max_num;
		}
	}
	return random_int;
}

/* 
* Helper function to initialize Q_function to all zero
*/
void InitQFunc(double *q_table, int num_states, int num_actions) {
	int i, j;
	for (i=0; i < num_states; i++) {
		for (j=0; j < num_actions; j++) {
			q_table[i*num_actions + j] = 0.0;
		}
	}
}

/* 
* Helper function to initialize learned_policy to all zero (first action)
*/
void InitPolicy(int policy[], int num_states) {
	int i;
	for (i=0; i < num_states; i++) {
		policy[i] = 0;
	}
}

/* 
* Helper function to generate a learned greedy policy from Q_function
*/
void GeneratePolicy(int policy[], double *q_table, int num_states, int num_actions)
{
	int i, j;
	for (i = 0; i < num_states; ++i) {
		policy[i] = 0;
		for (j = 0; j < num_actions; ++j) {
			// access as if a 1-D array and fix-up the indicies
			if (q_table[i*num_actions + j] > q_table[i*num_actions + policy[i]]) {
				policy[i] = j;
			}
		}
	}
}

/* 
* Greedy Action Selection
*/
int GetActionGreedy(state, q_table, num_actions)
int state;
double *q_table;
int num_actions;
{
	int i;
	int selected_action = 0;

	// printf("inside get action greedy!!\n");
	
	for (i = 1; i < num_actions; ++i) {
		if (q_table[state*num_actions + i] > q_table[state*num_actions + selected_action]) {
			selected_action = i;
		}
	}

	return selected_action;
}


/* 
* Select the next action using epsilon-greedy acction selection algorithm
*/
int GetActionEpsilonGreedy(state, counter, q_table, eps_min, eps_max, itermax, num_actions)
int state;
int counter;
double *q_table;
double eps_min;
double eps_max;
int itermax;
int num_actions;
{
	int i;
	int selected_action = 0;
	
    double epsilon = eps_min + eps_max * (counter / itermax);
	double rand_num = (double) rand() / (double)RAND_MAX;

    if (rand_num > epsilon) {
        selected_action = RandomInt(num_actions - 1);
    } else {
        selected_action = GetActionGreedy(state, q_table, num_actions);
    }

	return selected_action;
}

/* 
* Select the next action using epsilon-greedy acction selection algorithm
*/
int GetActionEpsilonGreedy_TimeBased(state, counter, q_table, eps_min, eps_max, num_episodes, time_per_eps, num_actions)
int state;
int counter;
double *q_table;
double eps_min;
double eps_max;
int num_episodes;
int time_per_eps;
int num_actions;
{
	int i;
	int selected_action = 0;
	
    double epsilon = eps_min + eps_max * (counter / (double) (num_episodes * time_per_eps));
	double rand_num = (double) rand() / (double)RAND_MAX;

	// printf("%.4f\n", (counter / (num_episodes * time_per_eps)));

	printf("epsilon: %.4f\n", epsilon);
	printf("rand_num: %.4f\n", rand_num);

    if (rand_num > epsilon) {
        selected_action = RandomInt(num_actions - 1);
		// printf("selected_action: %d\n", selected_action);
    } else {
        selected_action = GetActionGreedy(state, q_table, num_actions);
    }

	return selected_action;
}


/*
*
*  Runs a given action from array of actions
*
*/
void RunAction(roger, actions, selected_action, time) 
Robot* roger;
int (*actions[])(Robot* roger, double errors[NDOF], double time);
int selected_action;
double time;
{
    double errors[NDOF] = {0.0};
	printf("\n");
	printf("In RunAction function, run the selected action:  %d\n", selected_action);
	if (selected_action == 0) {
		printf("Selected ChaseTouch. \n");
	}else if(selected_action == 1) {
		printf("Selected VFClosure. \n");
	}
    // run action
    (*actions[selected_action])(roger, errors, time);

	printf("In RunAction function, errors to submit is:  \n");
	print_errors(errors);
	printf("\n");
    // submit selected actions errors
    submit_errors(roger, errors);
}

/*
*
*  Implmentation of one-step tabular q-learning
*
*/
void Q_Learning(roger, time, alpha, gamma, actions, rewards, default_reward, num_actions,
	num_states, reward_num, q_table, episode_time_limit, num_episodes, eps_min, eps_max, convergence_action, 
	convergence_reward, convergence_penalty, qtable_path, reset_func)
Robot* roger;
double time;
double alpha;
double gamma;
int (*actions[])(Robot* roger, double errors[NDOF], double time);
double (*rewards[])(int state, int previous_state, int previous_action, int internal_state[]);
double default_reward;
int num_actions;
int num_states;
int reward_num;
double *q_table;
int episode_time_limit;
int num_episodes;
double eps_min;
double eps_max;
int convergence_action;
int convergence_reward;
int convergence_penalty;
char* qtable_path;
// int (*reset_func)(Robot* roger, double time, int state, int internal_state[], int previous_action, int counter, int episode_len);
int (*reset_func)(Robot* roger, double time, int state, int internal_state[], int previous_action, int counter, int episode_len);
{
    // variables that we need to persist between calls
    int internal_state[num_states];
	static int return_state = NO_REFERENCE;
	static int initialized = 0;
	static int previous_state;
	static int previous_action;
	static int counter = 0;
	static int last_saved_policy = 0;
    static int explore = 1;
	static int action_counter = 0;
	static int num_expisodes_completed = 0;
	static int reset_episode = 0;
	static int select_new_action = 0;
	int policy[num_states];
	// printf("\n");
	// printf("In Q Learning function. \n");
	if (counter % 1000 == 0) {
		printf("counter: %d ms\n", counter);
	}
	

    // initialize the policy and quality function variables
	if (!initialized) {
		printf("Initliazing Q_Func and policy!\n");
		// InitPolicy_Random(policy, num_states, action_counter);
		InitQFunc(q_table, num_states, num_actions);
		initialized = 1;
		previous_state = 0;
		previous_action = 0;
		printf("num_episodes: %d\n", num_episodes);
	}

    // variables that are calculated fresh each call 
	int selected_action = 0;
    double rewards_sum = 0;
	double update;
    double temp_errors[NDOF] = {0.0};
    int state, i, j;

	// printf("calculating state\n");
    // iterate over each action in the passed actions array
    //     and save off the returned state, and calculate
    //     the internal state of Q_Learning
	// printf("Number of action is: %d\n", num_actions);
	state = 0;
	/// printf("Internal_state loop in Q_Learning starts. \n");
    for (i = 0; i < num_actions; ++i) {
        internal_state[i] = (*actions[i])(roger, temp_errors, time);
		// submit_errors(roger, temp_errors);
        state += pow(3, i)*internal_state[i];
		
		// printf("\tinternal_state[%d] = %d\n", i, internal_state[i]);
		// printf("In Q learning, the error is:    \n");
		// print_errors(temp_errors);

    } 

	// printf("State: %d, previous_state: %d\n", state, previous_state);

    // explore vs. exploit loops
    if (explore) {
		printf("Inside explore loop\n");
        // if we are finished learning, generate the last policy and then switch to exploit mode
		if (num_expisodes_completed >= num_episodes) {
			GeneratePolicy(policy, q_table, num_states, num_actions);
			counter = 0;
			explore = 0;
            rewards_sum = 0.0;
			printf("Generated Final Policy!\n");
		}

		// printf("internal_state[previous_action] = %d\n", internal_state[previous_action]);
		// printf("previous_state is: %d , current state is: %d \n", previous_state, state);
        // we only learn on state changes and when previously selected action has nothing to do
        if (previous_state != state || select_new_action == 1) {
			select_new_action = 0;
			printf("State Change!\n");
            rewards_sum = default_reward;

            // iterate over all the passed rewards
            for (i=0; i<reward_num; ++i) {
				// printf("Processing reward[%d]\n", i);
                rewards_sum += (*rewards[i])(state, previous_state, previous_action, internal_state);
            }


            // perform the Q_learning update
			// printf("Performing Q_Learning update\n");
            
            // extract the maxQvalue of this (current) state as a results of
            //     the previous state x action pair
			double maxQ = q_table[state*num_actions + 0]; // access using speical code
			for (i = 1; i < num_actions; ++i) {
				// printf("i: %d, state: %d\n", i, state);
				if (q_table[state*num_actions + i] > maxQ) {
					maxQ = q_table[state*num_actions + i];
				}
			}

			// printf("Updating Q_function\n");
            // calculate the update value
			update = alpha*(rewards_sum + gamma*maxQ - q_table[previous_state * num_actions + previous_action]);

            // update the Q_function
			q_table[previous_state*num_actions + previous_action] += update;

            // print off relavent information and increment the counter
            // printf("State: %d, prev_state: %d, prev act: %d, QMAX: %.4f, reward: %.4f, update: %.4f, counter: %d\n", state, previous_state, previous_action, maxQ, rewards_sum, update, counter);
			// get the next action
        	// printf("selecting next actions!!!!\n");
			selected_action = GetActionEpsilonGreedy_TimeBased(state, counter, q_table, eps_min, eps_max, num_episodes, episode_time_limit, num_actions);
			// printf("Get action greedily. \n");
			
			// printf("Selected_action: %d, previous_action: %d\n", selected_action, previous_action);
			action_counter = 0;
        } else { // state change if
			// printf("change selected action\n");
			selected_action = previous_action;
		} 
    } else {       // if not exploring
		printf("In exploiting. \n");
        selected_action = GetActionGreedy(state, q_table, num_actions);
		// printf("state: %d, selected action: %d, counter: %d\n", state, selected_action, counter);
		if (previous_state != state) {
			rewards_sum += default_reward;
			
            // iterate over the rewards and acumulate
            for (i=0; i<reward_num; ++i) {
                rewards_sum += (*rewards[i])(state, previous_state, previous_action, internal_state);
            }			
		}
    }

	// printf("check if need to reset the ball\n");
	// check if we should reset the ball
	reset_func = reset_ball_training;
	int reset_result = (*reset_func)(roger, time, state, internal_state, previous_action, counter, episode_time_limit);
	// printf("finish checking \n");
	if (reset_result && explore) {
		reset_episode = 1;
	}

	// Save off policies every-so-often for the shake of comparison
	if (((counter % episode_time_limit == 0 && last_saved_policy != counter) || reset_episode) && explore) {
		// increment the episode counter
		num_expisodes_completed += 1;
		select_new_action = 1;

		printf("num_expisodes_completed: %d\n", num_expisodes_completed);

		// extract the maxQvalue of this (current) state as a results of
		//     the previous state x action pair
		double maxQ = q_table[state*num_actions + 0]; // access using speical code
		for (i = 1; i < num_actions; ++i) {
			// printf("i: %d, state: %d\n", i, state);
			if (q_table[state*num_actions + i] > maxQ) {
				maxQ = q_table[state*num_actions + i];
			}
		}

		double temp_num = last_saved_policy/episode_time_limit;
		double final_reward;

		// not converged, penalty
		if (internal_state[convergence_action] != CONVERGED) {
			final_reward = default_reward +  convergence_penalty;
			printf("adding final convergence penalty: %.4f\n", final_reward);
			// update the Q_function
			q_table[previous_state*num_actions + previous_action] += alpha*(final_reward + gamma*maxQ - q_table[previous_state * num_actions + previous_action]);

			// last_saved_policy = counter;

		} else { // converged, reward
			final_reward = default_reward +  convergence_reward;
			printf("Adding final convergence reward:  %.4f\n", final_reward);
			// update the Q_function
			q_table[previous_state*num_actions + previous_action] += alpha*(final_reward + gamma*maxQ - q_table[previous_state * num_actions + previous_action]);	
			
			// last_saved_policy = (temp_num +1) * episode_time_limit;
			// counter = last_saved_policy;
		}

		if (counter % episode_time_limit == 0) {
			last_saved_policy = counter;
		} else {
			last_saved_policy = (temp_num +1) * episode_time_limit;
			counter = last_saved_policy;
		}

		// printf("State: %d, prev_state: %d, prev act: %d, QMAX: %.4f, reward: %.4f, counter: %d\n", state, previous_state, previous_action, maxQ, final_reward, counter);

		
		// char buff[sizeof(policy_path) + 100];
		// double temp_num = last_saved_policy/episode_time_limit;
		// if (reset_episode) {
		// 	last_saved_policy = (temp_num +1) * episode_time_limit;
		// 	counter = last_saved_policy - 1;
		// } else {
		// 	last_saved_policy = counter;
		// }
		
		printf("Iteration num %d\n", counter);
		// GeneratePolicy(policy, q_table, num_states, num_actions);
		// SavePolicy(temp_num, policy, num_states, policy_path);
		qtable_path  = "./q_tables/qtable_9.txt";
		SaveQTable(temp_num, q_table, num_states, num_actions, qtable_path);
		printf("policy %f saved\n", temp_num);
		reset_episode = 0;
	} else {
		// run selected action
		// printf("Running selected action!!!\n");
		RunAction(roger, actions, selected_action, time);
		previous_state = state;
		previous_action = selected_action;
	}
	counter++; // increment counter on every time this loop is executed (every time-step which in Roger is 1ms)
}


/*
*
*  Implmentation of one-step tabular q-learning
*
*/
void Run_Policy(roger, time, actions, rewards, default_reward, num_actions, num_states, num_rewards, q_table)
Robot* roger;
double time;
int (*actions[])(Robot* roger, double errors[NDOF], double time);
double (*rewards[])(int state, int previous_state, int previous_action, int internal_state[]);
double default_reward;
int num_actions;
int num_states;
int num_rewards;
double *q_table;
{
	// variable to hold the selected option
	static int selected_action;
	// state/action variables
	static int initialized = 0;
	static int previous_state = 0;
	static int previous_action = 0;
	int state = 0;
	// counter variable
	static int counter = 0;
	// variable to trak the reward gained
	static double total_reward_exploit = 0.0;
	// array to hold the internal state of the various actions
	int internal_state[num_states];
	// general use variables
	int i;
	double temp_errors[NDOF];

	// get the current state
	state = 0;
    for (i = 0; i < num_actions; ++i) {
        internal_state[i] = (*actions[i])(roger, temp_errors, time);
        state += pow(3, i)*internal_state[i];
		// printf("\tinternal_state[%d] = %d\n", i, internal_state[i]);
    } 

	selected_action = GetActionGreedy(state, q_table, num_actions);
	printf("state: %d, selected action: %d, counter: %d\n", state, selected_action, counter);
	if (previous_state != state) {
		total_reward_exploit += default_reward;
		// iterate over all the passed rewards
		for (i=0; i<num_rewards; ++i) {
			// printf("Processing reward[%d]\n", i);
			total_reward_exploit += (*rewards[i])(state, previous_state, previous_action, internal_state);
		}
	}

	// run selected action
    RunAction(roger, actions, selected_action, time);
    previous_state = state;
    previous_action = selected_action;
}

/*
*
*  Function used to track the statistics of running various policies trained for a specific task, a specified number of times
*      tracks: total_reward, 
*              num_actions_taken  : number of actions taken, 
*			   num_timesteps_used : number of timesteps taken,
*			   convergence_status : binary variable indicating if the function timeed-out (0) or converged (1)
*  
*      args: roger                : Robot                    : instance of the Robot structure
*	       	 time                 : double                   : The usual time-step passed to Roger functions
*	   		 actions              : int *actions[]           : an array of actions that can be performed
*	   		 reward               : double (#rewards)        : an array of reward functions
*	   		 default_reward       : double                   : the default reward awarded at every action-selection
*	   		 num_actions          : int                      : the number of actions avalible for the problem
*	   		 num_states           : int                      : the number of states in this problem (3^num_actions)
*	   		 num_rewards          : int                      : the number of reward functions used in this problem
*	   		 q_table              : int[num_states]          : a loaded (learned) policy ready to execute
*	   		 output_file_path     : char*                    : file_path for the output statistics csv (needs to end in _%s)
*            action_calls         : static int[num_actions]  : static array that counts how many times each action is called 
*	   		 num_runs             : int                      : number of times we want to run the data collection loop
*	   		 max_actions          : int                      : number of actions that Roger can perform for this task before timing out
*	   		 convergence_function : *function_ptr            : pointer to the function whose status is used to track task convergence 
*	   		 time_steps_converged : int                      : number of time steps convergence_function must be converged before the problem is converged
*            convergence_reward   : double                   : reward for the task converging (awarded when convergence function is converged for time_steps_converged)
*                                 :                          :     do NOT include reward function for Convergence, instead use this value to reward convergence
*            prempt_time_steps    : int                      : number of time steps allowed between state/change before declaring the problem "unsolvable"
*/
int generatePerformanceData(roger, time, actions, rewards, default_reward, num_actions, num_states, num_rewards, q_table,
	output_file_path, action_calls, num_runs, convergence_function, episode_penalty, convergence_reward,
	episode_time_limit, reset_func) 
Robot* roger;
double time;
int (*actions[])(Robot* roger, double errors[NDOF], double time);
double (*rewards[])(int state, int previous_state, int previous_action, int internal_state[]);
double default_reward;
int num_actions;
int num_states;
int num_rewards;
// int policy[];
double *q_table;
char *output_file_path;
int action_calls[];
int num_runs;
int convergence_function;
// int time_steps_converged;
int episode_penalty;
int convergence_reward;
int episode_time_limit;
int (*reset_func)(Robot* roger, double time, int state, int internal_state[],
	int previous_action, int num_timesteps_used, int episode_time_limit);
{
	// state/action variables
	static int initialized = 0;
	static double total_reward_exploit = 0.0;
	static int previous_state = 0;
	static int previous_action = 0;
	static int selected_action = 0;
	int internal_state[num_states]; // array to hold the internal state of the various actions
	int state = 0;

	// general use variables
	static int ep_num;              // number of time the task has been run
	static FILE * fp;               // file pointer used to save off the relevent data
	double temp_errors[NDOF]; // dummy variable used for calculating state
	int i, j;                       // general use index/counter
	int save_and_restart = 0;       // used to track if we need to save data or not

	// variables to track performace
	static int num_actions_taken = 0;
	static int num_timesteps_used = 0;
	// static int num_time_converged = 0;
	static int convergence_status = 0;
	// static int time_steps_wo_statechange = 0;

	
	// create the file that we will save results too
	if (initialized == 0) {

		// zero out action_calls array to be safe
		for (i = 0; i < num_actions; i++) {
			action_calls[i] = 0.0;
		}

		initialized = 1;
		char buf[sizeof(output_file_path) + 50];
		memset(buf, '\0', sizeof(buf));
		sprintf(buf, output_file_path, "performance_data.csv");
		printf("Saving Data to file: %s\n", buf);
		fp = fopen(buf, "w+");
		if (!fp) {
			printf("Error loading policy file");
			exit(1);
		} else {
			char *action_str = "action_%d_calls, ";
			fprintf(fp, "trail_num, total_reward, num_actions, num_timesteps, convergence_status, ");
			for (i = 0; i < num_actions-1; i++) {
				memset(buf, '\0', sizeof(buf));
				sprintf(buf, action_str, i);
				fprintf(fp, "%s",  buf);
			}
			action_str = "action_%d_calls\n";
			memset(buf, '\0', sizeof(buf));
			sprintf(buf, action_str, (num_actions-1));
			fprintf(fp, "%s",  buf);
			fflush(fp);
		}
	}


	// if we have not done all the trail runs we want
	if (ep_num < num_runs) {
		save_and_restart = 0;

		// run policy
		// get the current state
		// printf("about to check the state\n");
		state = 0;
		for (i = 0; i < num_actions; ++i) {
			internal_state[i] = (*actions[i])(roger, temp_errors, time);
			state += pow(3, i)*internal_state[i];
			// printf("\tinternal_state[%d] = %d\n", i, internal_state[i]);
		} 

		// select the next action given the policy and state
		selected_action = GetActionGreedy(state, q_table, num_actions);
		if (previous_state != state) {
			printf("state: %d, selected action: %d, num_actions_taken: %d\n", state, selected_action, num_actions_taken);
			total_reward_exploit += default_reward;
			// iterate over all the passed rewards
			for (i=0; i<num_rewards; ++i) {
				// printf("Processing reward[%d]\n", i);
				total_reward_exploit += (*rewards[i])(state, previous_state, previous_action, internal_state);
			}
			// printf("made it past rewards\n");
			// if the state changes we are selected a "new" action
			num_actions_taken += 1;
			action_calls[selected_action] += 1;
			// time_steps_wo_statechange = 0;
			// printf("action_calls[%d]: %d\n", selected_action, action_calls[selected_action]);
		} 
		
		// else {
		// 	time_steps_wo_statechange += 1;

		// 	printf("time_steps_wo_statechange: %d\n", time_steps_wo_statechange);
			
		// 	// if we have gone "prempt_time_steps" time-steps w/o a state change
		// 	//     end the problem with a large negative reward
		// 	if (time_steps_wo_statechange > prempt_time_steps) {
		// 		save_and_restart = 1;
		// 		time_steps_wo_statechange = 0;
		// 		total_reward_exploit += -100;
		// 	}
		// }

		// run selected action
		RunAction(roger, actions, selected_action, time);
		// printf("made it past running the action\n");
		previous_state = state;
		previous_action = selected_action;

		// // check if the function has converged
		// if ((*convergence_function)(roger, temp_errors, time) == CONVERGED) {
		// 	num_time_converged += 1;
		// 	printf("num_time_converged: %d\n", num_time_converged);
		// 	// we want the function to be converged for a minimum amount of time
		// 	if (num_time_converged > time_steps_converged) {
		// 		// open file, write, and save
		// 		total_reward_exploit += convergence_reward;
		// 		convergence_status = 1;
		// 		save_and_restart = 1;
		// 	}
		// 	// printf("made it past checking convergence\n");
		// } else {
		// 	// printf("made it past checking convergence\n");
		// 	num_time_converged = 0;
		// }

		// check if we should reset the ball
		int reset_result = (*reset_func)(roger, time, state, internal_state, previous_action, num_timesteps_used, episode_time_limit);

		// // check if the function has run out of actions
		// if (num_actions_taken > max_actions) {
		// 	save_and_restart = 1;
		// 	total_reward_exploit += -100;
		// }

		// printf("save_and_restart: %d\n", save_and_restart);

		if (num_timesteps_used % 1000 == 0) {
			printf("num_timesteps_used: %d ms\n", num_timesteps_used);
		}
		// everytime we run this function it is one time-step
		num_timesteps_used += 1;

		// if we need to restart for whatever reason 
		if (reset_result == 1) {

			if (internal_state[convergence_function] != CONVERGED) {
				total_reward_exploit +=  episode_penalty;
				convergence_status = FALSE;
			} else { // converged, reward
				total_reward_exploit += convergence_reward;
				convergence_status = TRUE;
			}

			// save off the stats from this run
			fprintf(fp, "%d, %.6f, %d, %d, %d, ", ep_num, total_reward_exploit, num_actions_taken, num_timesteps_used, convergence_status);
			
			char *action_str = "%d, ";
			char buf[5];
			for (i = 0; i < num_actions-1; i++) {
				memset(buf, '\0', sizeof(buf));
				sprintf(buf, action_str, action_calls[i]);
				fprintf(fp, "%s",  buf);
			}

			// fixme plz

			action_str = "%d\n";
			memset(buf, '\0', sizeof(buf));
			sprintf(buf, action_str, action_calls[(num_actions-1)]);
			fprintf(fp, "%s", buf);
			fflush(fp);

			// clear stats variables
			for (i = 0; i < num_actions; i++) {
				action_calls[i] = 0.0;
			}
			convergence_status = 0;
			// num_time_converged = 0;
			total_reward_exploit = 0.0;
			num_actions_taken = 0;
			num_timesteps_used = 0;
			ep_num += 1;
			// (*reset_func)(roger, time, state);
		}
		return 0;
	} else {
		// close the stats output file
		fclose(fp);
		return 1;
	}
}