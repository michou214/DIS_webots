/**************************************************************************************************
 * File:        flock_controller.c
 * Authors:     Zeki Doruk Erden, Michael Perret, Mickaël Salamin
 * Date:        Fall 2018
 * Description: Controller for DIS project.
 * Version:		Finale
 *************************************************************************************************/

/*** Specific libraries ***/

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/compass.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>

/*** Standard libraries ***/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <time.h>

/*** Symbolic constants ***/

#define WORLD_CROSSING 1

#define TIME_STEP  64 // Step duration [ms]
#define FLOCK_SIZE 5  // Number of robots in flock

#define NB_SENSORS 8    // Number of distance sensors
#define MIN_SENS   350  // Minimum sensibility value
#define MAX_SENS   4096 // Maximum sensibility value

#define MAX_SPEED       1000    // Maximum speed for differential drive [mm/s]
#define MAX_SPEED_MOTOR 6.28    // Maximum speed for rotational motor [rad/s]
#define AXLE_LENGTH     0.052   // Distance between wheels of robot [m]
#define SPEED_UNIT_RADS 0.00628 // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS    0.0205  // Wheel radius [m]
#define DELTA_T         0.064   // Timestep [s]

#define NEIGHBORHOOD_THRESHOLD 0.25      // Maximum neighborhood radius [m]
#define RULE1_THRESHOLD        0.0       // Threshold to activate aggregation rule (default 0.20)
#define RULE2_THRESHOLD        0.05      // Threshold to activate dispersion rule (default 0.15)
#define RULE1_WEIGHT           (1.5/10)  // Weight of aggregation rule (default 0.6/10)
#define RULE2_WEIGHT           (0.5/10)  // Weight of dispersion rule (default 0.02/10)
#define RULE3_WEIGHT           (1.0/10)  // Weight of consistency rule (default 1.0/10)
#define MIGRATION_WEIGHT       (0.02/10) // Weight of attraction towards the common goal (default 0.02/10)
#define MIGRATORY_URGE         1         // If the robots should just go forward or move towards a specific migratory direction

/*** Symbolic macros ***/

#define ABS(x) ((x >= 0) ? (x) : -(x))

/*** Global constants ***/

//static const int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // Weights for obstacle avoidance
static const float e_puck_matrix[16] = { 19, 6, 4,  0,  0, -4,-6,-22,  -22,-12,-6,  0, 0, 6, 12, 19}; // Maze
                                  //  R: 0   1   2   3   4    5    6    7|L: 0    1    2   3   4   5   6   7
#if WORLD_CROSSING
static const float migration[2][2] = {{-50, 0}, {50, 0}}; // Migration vector for world crossing {team0, team1}
#else
static const float migration[1][2] = {{50, 0}}; // Migration vector for world obstacles {team0}
#endif

/*** Global variables ***/

static char*        robot_name; // Robot's unique identification name
static unsigned int robot_team; // Robot's unique identification team
static unsigned int robot_id;   // Robot's unique identification number

static WbDeviceTag left_motor, right_motor; // Rotational motors
static WbDeviceTag emitter, receiver;       // Radio sensors
static WbDeviceTag sensors[NB_SENSORS];     // Distance sensors
static WbDeviceTag compass;                 // Orientation sensor

static float      my_position[3];              // X, Z, Theta of the current robot
static float prev_my_position[3];              // X, Z, Theta of the current robot at the previous time step
static float      relative_pos[FLOCK_SIZE][3]; // Relative X, Z, Theta of all robots
static float prev_relative_pos[FLOCK_SIZE][3]; // Relative X, Z, Theta of all robots at the previous time step
static float          speed[FLOCK_SIZE][2];    // Speed of all robots
static float relative_speed[FLOCK_SIZE][2];    // Speed of all robots relative to the current robot
static int in_neighborhood[FLOCK_SIZE];        // Indication wheter the other robots are in the neighborhood of the current robot

/*** Functions declaration ***/

void init(void);
void limit(int *number, const int limit);
void limitf(float *number, const float limit);
void compute_wheel_speeds(int *msl, int *msr);
void update_self_motion(const int msl, const int msr);
void send_ping(void);
void process_received_ping_messages(void);
void reynolds_rules(void);

/*** Functions implementation ***/

/*
 * Initialize the controller.
 */
void init(void)
{
	// Unique identifier of the robot
	robot_name = (char*) wb_robot_get_name();
	sscanf(robot_name, "epuck%d", &robot_id);
	robot_team = robot_id/FLOCK_SIZE;
	robot_id  %= FLOCK_SIZE; // Normalize between 0 and FLOCK_SIZE-1
	
	// Radio emitter and receiver
	emitter  = wb_robot_get_device("emitter");
	receiver = wb_robot_get_device("receiver");
	wb_receiver_enable(receiver, TIME_STEP);
	
    // Compass
    compass = wb_robot_get_device("compass");
	if (compass) {
		wb_compass_enable(compass, TIME_STEP);
	}
	
	// Wheel motors and encoders
    left_motor  = wb_robot_get_device("left  wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position( left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);
	wb_motor_set_velocity( left_motor, 0);
	wb_motor_set_velocity(right_motor, 0);
	
	// Distance sensors
	char sensor_name[] = "ps0";
	for (unsigned int i = 0; i < NB_SENSORS; i++) {
		sensors[i] = wb_robot_get_device(sensor_name);
		wb_distance_sensor_enable(sensors[i], TIME_STEP);
		sensor_name[2]++;
	}
}

/*
 * Keep given integer within interval [-limit, +limit].
 */
void limit(int *number, const int limit)
{
	if (*number >  limit) 	*number =  limit;
	if (*number < -limit) 	*number = -limit;
}
void limitf(float *number, const float limit)
{
	if (*number >  limit) 	*number =  limit;
	if (*number < -limit) 	*number = -limit;
}

/*
 * Compute wheel speeds from a given (X,Z) speed.
 */
void compute_wheel_speeds(int *msl, int *msr)
{
	// Compute wanted position from Reynolds speed and current location
	float x =  speed[robot_id][0]*cosf(my_position[2]) + speed[robot_id][1]*sinf(my_position[2]); // x in robot coordinates
	float z = -speed[robot_id][0]*sinf(my_position[2]) + speed[robot_id][1]*cosf(my_position[2]); // z in robot coordinates
	//printf("[%s] x = %f, y = %f\n", robot_name, x, z);
	
	// Proportional controller on tangential and angular speeds
	float Ku = 0.2; // Forward control coefficient
	float Kw = 1.0; // Rotational control coefficient
	float range = sqrtf(x*x + z*z); // Distance to the wanted position
	float bearing = atan2(z, x); // Orientation of the wanted position
	
	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;
	
	// Convert to wheel speeds
	*msl = (u - AXLE_LENGTH*w/2.0) * (1000.0/WHEEL_RADIUS);
	*msr = (u + AXLE_LENGTH*w/2.0) * (1000.0/WHEEL_RADIUS);
	limit(msl, MAX_SPEED);
	limit(msr, MAX_SPEED);
	//printf("[%s] bearing = %f, u = %f, w = %f, msl = %d, msr = %d\n", robot_name, bearing, u, w, *msl, *msr);
}

/*
 * Update robot's position with wheel speeds.
 */
void update_self_motion(const int msl, const int msr)
{
	// Previous orientation of the robot
	const float theta = my_position[2];
	
	// Compute deltas of the robot
	const float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	const float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	const float du = (dr + dl)/2.0;
	const float dtheta = (dr - dl)/AXLE_LENGTH;
	
	// Compute deltas in the environment
	const float dx = du * cosf(theta);
	const float dz = du * sinf(theta);

	
	// Update position
	my_position[0] += dx;
	my_position[1] += dz;
	my_position[2] += dtheta;
	if (compass) {
		const double *compass_heading = wb_compass_get_values(compass);
		my_position[2] = -atan2(compass_heading[0], compass_heading[2]) + M_PI;
	}
	
	// Keep orientation within [0, 2pi]
	if (my_position[2] > 2*M_PI) 	my_position[2] -= 2*M_PI;
	if (my_position[2] < 0     ) 	my_position[2] += 2*M_PI;
	//printf("[%s] orientation = %f°\n", robot_name, my_position[2]*180/M_PI);
}

/*
 * Each robot sends a ping message, so that the other robots can measure the relative range and bearing to the sender.
 * The message contains the robot's name.
 * The range and bearing will be measured directly out of message RSSI and direction.
 */
void send_ping(void)
{
	char out[10];
	strcpy(out, robot_name); // In the ping message we send the name of the robot
	wb_emitter_send(emitter, out, strlen(out)+1); 
}

/*
 * Process all the received ping messages, and calculate range and bearing to the other robots.
 * The message contains the robot's name.
 * The range and bearing are measured directly out of message RSSI and direction.
 */
void process_received_ping_messages(void)
{
	unsigned int i; // Loop counter
	char *inbuffer;	// Buffer for the receiver node
	const double *message_direction; // Direction of the emitter with respect to the receiver's coordinate system
	double message_rssi; // Received Signal Strength Indicator
	double theta;
	double range;
	unsigned int other_robot_team, other_robot_id;
	
	// Reset the neighborhood
	for (i = 0; i < FLOCK_SIZE; i++) {
		in_neighborhood[i] = 0;
	}
	
	while (wb_receiver_get_queue_length(receiver) > 0) {
		// Retrieve information from message
		inbuffer = (char*) wb_receiver_get_data(receiver);
		message_direction = wb_receiver_get_emitter_direction(receiver);
		message_rssi = wb_receiver_get_signal_strength(receiver);
		const double x = message_direction[0]; // Relative X in Webots reference frame
		const double z = message_direction[2]; // Relative Z in Webots reference frame

		// Compute orientation and distance from teammate to robot
		theta = -atan2(z, x) - M_PI/2.0; // Reference frame of e-puck is rotated -90° from Webots reference frame
		theta = theta + my_position[2];  // Absolute theta
		range = sqrt((1/message_rssi));

		// Teammate's ID is retrieved from the robot's name contained in the message
		sscanf(inbuffer, "epuck%d", &other_robot_id);
		other_robot_team = other_robot_id/FLOCK_SIZE;
		other_robot_id %= FLOCK_SIZE; // Normalize between 0 and FLOCK_SIZE-1
		
		// If robots are in the same neighborhood
		if (range < NEIGHBORHOOD_THRESHOLD) {
			in_neighborhood[other_robot_id] = 1;
		
			// If robots are in the same team
			if (robot_team == other_robot_team) {
				
				/* Position update */
				prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
				prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];
				
				relative_pos[other_robot_id][0] =  range*cos(theta); // Relative x-coordinate
				//printf("[%d], [%d], rel_pos_X=%f \n", robot_id, other_robot_id, relative_pos[other_robot_id][0]);
				relative_pos[other_robot_id][1] = -range*sin(theta); // Relative z-coordinate
				//printf("[%d], [%d], rel_pos_Z=%f \n", robot_id, other_robot_id, relative_pos[other_robot_id][1]);
				
				//printf("[%s] from %s, x = %f, y = %f, theta = %f, my_theta = %f\n", robot_name, inbuffer, relative_pos[other_robot_id][0], relative_pos[other_robot_id][1], -atan2(y,x)*180.0/3.141592, my_position[2]*180.0/3.141592);
				
				/* Speed update */
				relative_speed[other_robot_id][0] = (relative_pos[other_robot_id][0]-prev_relative_pos[other_robot_id][0])/DELTA_T;
				relative_speed[other_robot_id][1] = (relative_pos[other_robot_id][1]-prev_relative_pos[other_robot_id][1])/DELTA_T;
			}
  		}
		wb_receiver_next_packet(receiver);
	}
}

/*
 * Update speed according to Reynolds' rules.
 */
void reynolds_rules(void)
{
	unsigned int i, j;  	        // Loop counters
	unsigned int n_robots = 1;      // Number of neighbors
	float rel_avg_loc  [2] = {0,0}; // Flock average relative positions
	float rel_avg_speed[2] = {0,0}; // Flock average relative speeds
	float cohesion     [2] = {0,0}; // Aggregation behavior
	float dispersion   [2] = {0,0}; // Separation behavior
	float consistency  [2] = {0,0}; // Alignment behavior
	
	/* Compute averages over the flockmates in the local neighborhood */
	for (i = 0; i < FLOCK_SIZE; i++) {
		if (i != robot_id) { // Loop on flockmates only
			// If robot i is in the local neighborhood
			if (in_neighborhood[i]) {
				for (j = 0; j < 2; j++) {
					rel_avg_loc  [j] += relative_pos  [i][j];
					rel_avg_speed[j] += relative_speed[i][j];
				}
				n_robots++;
			}
		}
	}
	for (j = 0; j < 2; j++) {
		if (n_robots > 1) {
			rel_avg_loc  [j] /= n_robots-1;
			rel_avg_speed[j] /= n_robots-1;
		} else {
			rel_avg_loc  [j] = 0;
			rel_avg_speed[j] = 0;
		}
	}

	/* Reynold's rules */	

	/* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
	// If center of mass is too far (Euclidean distance)
	if (sqrt(pow(rel_avg_loc[0],2)+pow(rel_avg_loc[1],2)) > RULE1_THRESHOLD) {
		for (j = 0; j < 2; j++) {
			cohesion[j] = rel_avg_loc[j]; // Average relative position
		}
	}
	
	/* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
	for (i = 0; i < FLOCK_SIZE; i++) {
		if (i != robot_id) { // Loop on flockmates only
			// If neighbor i is too close (Euclidean distance)
			if (sqrt(pow(relative_pos[i][0],2)+pow(relative_pos[i][1],2)) < RULE2_THRESHOLD) {
				for (j = 0; j < 2; j++) {
					if (relative_pos[i][j] != 0) {
						dispersion[j] -= 1/relative_pos[i][j]; // Relative distance to neighbor i
					}
				}
			}
		}
	}
	
	/* Rule 3 - Consistency/Alignment: match the speeds of flockmates */
	for (j = 0; j < 2; j++) {
		consistency[j] = rel_avg_speed[j]; // Average relative speed
	}
	
	/* Aggregation of all behaviors with relative influence determined by weights */
	for (j = 0; j < 2; j++) {
		speed[robot_id][j]  = cohesion   [j] * RULE1_WEIGHT;
		speed[robot_id][j] += dispersion [j] * RULE2_WEIGHT;
		speed[robot_id][j] += consistency[j] * RULE3_WEIGHT;
	}
	speed[robot_id][1] *= -1; // y-axis of Webots is inverted
	//printf("[%d] Cohesion: %f/%f, Dispersion = %f/%f, Consistency = %f/%f \n", robot_id, cohesion[0] * RULE1_WEIGHT,cohesion[1] * RULE1_WEIGHT, dispersion [0] * RULE2_WEIGHT,dispersion [1] * RULE2_WEIGHT, consistency[0] * RULE3_WEIGHT,consistency[1] * RULE3_WEIGHT);
	
	/* Move the robot according to some migration rule */
	if (MIGRATORY_URGE == 0) {
		speed[robot_id][0] += 0.01*cos(my_position[2] + M_PI/2);
		speed[robot_id][1] += 0.01*sin(my_position[2] + M_PI/2);
	} else {            	
		speed[robot_id][0] += (migration[robot_team][0]-my_position[0]) * MIGRATION_WEIGHT;
		speed[robot_id][1] -= (migration[robot_team][1]-my_position[1]) * MIGRATION_WEIGHT; // y-axis of Webots is inverted
    	//printf("[%d] mypos0 = %f, mypos1 = %f \n", robot_id, rel_avg_loc[0], rel_avg_loc[1]);
	}
}

/*
 * Main function.
 */
int main(int argc, char *args[])
{
	// Variables declaration
	unsigned int i;            // Loop counter
	int msl, msr;              // Wheel speeds for differential drive
	float msl_w, msr_w;        // Wheel speeds for rotational motors
	int bmsl, bmsr;            // Braitenberg parameters
	int distances[NB_SENSORS]; // Array for the distance sensor readings
	int sum_sensors;           // Sum of all distance sensor readings
	
	// Initialization
	wb_robot_init();
	init();
	
	// Variables initialization
	my_position[0] = 0; my_position[1] = 0; my_position[2] = 0;
	msl = 0; msr = 0;
	
	// Simulation loop
	while (wb_robot_step(TIME_STEP) != -1) {
		bmsl = 0; bmsr = 0;
		sum_sensors = 0;
		
		/* Braitenberg obstacle avoidance */	
		for (i = 0; i < NB_SENSORS; i++) {
			distances[i] = wb_distance_sensor_get_value(sensors[i]); // Read sensor values
			sum_sensors += distances[i]; // Add up sensor values
			
			// Weighted sum of distance sensor values for Braitenberg vehicle
			bmsr += e_puck_matrix[i]            * (distances[i]/(float)MAX_SENS) * MAX_SPEED;
			bmsl += e_puck_matrix[i+NB_SENSORS] * (distances[i]/(float)MAX_SENS) * MAX_SPEED;
		}
		
		/* Compute self position */
		prev_my_position[0] = my_position[0];
		prev_my_position[1] = my_position[1];
		
		update_self_motion(msl, msr);
		
		speed[robot_id][0] = (my_position[0]-prev_my_position[0])/DELTA_T;
		speed[robot_id][1] = (my_position[1]-prev_my_position[1])/DELTA_T;
		
		/* Send and get information */
		// Sending a ping to other robots, so they can measure their distance to this robot
		send_ping();
		
		// Receiving ping from other robots, so this robot can measure its distance to them
		process_received_ping_messages();
		

		/* Compute self speed */
		// Reynolds' rules with all previous info (updates the speed[][] table)
		reynolds_rules();
		
		// Compute wheels speed from Reynolds speed
		compute_wheel_speeds(&msl, &msr);
		
		// Adapt speed instinct to distance sensor values
		if (sum_sensors > MIN_SENS) {
			msl = 0.5*MAX_SPEED;
			msr = 0.5*MAX_SPEED;
		}
		
		// Add Braitenberg
		msl += bmsl;
		msr += bmsr;
		//printf("[%s] msl = %d, msr = %d\n", robot_name, msl, msr);
		
		// Compute wheel speed
		msl_w = msl*MAX_SPEED_MOTOR/1000.0;
		msr_w = msr*MAX_SPEED_MOTOR/1000.0;
		
		// Saturate output command (floating point comparison problem -> little correction to avoid saturation warnings)
		limitf(&msl_w, MAX_SPEED_MOTOR-0.001);
		limitf(&msr_w, MAX_SPEED_MOTOR-0.001);
		printf("[%s] msr_w = %f, msr_w = %f\n", robot_name, msr_w, msr_w);
		
		// Set speed
		wb_motor_set_velocity( left_motor, msl_w);
		wb_motor_set_velocity(right_motor, msr_w);
	}
	
    // Clean up and exit
    wb_robot_cleanup();
    return 0;
}
