/**************************************************************************************************
 * File:        flock_super.c
 * Authors:     Zeki Doruk Erden, Michael Perret, Mickaël Salamin
 * Date:        Fall 2018
 * Description: Supervisor for DIS project.
 *************************************************************************************************/

/*** Specific libraries ***/

#include <webots/supervisor.h>
#include <webots/robot.h>
#include <webots/nodes.h>
#include <webots/emitter.h>

/*** Standard libraries ***/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <time.h>

/*** Symbolic constants ***/

#define TIME_STEP  64 // Step duration [ms]
#define FLOCK_SIZE 5  // Number of robots in flock

#define VERBOSE 1 // If messages should be printed

/*** Global constants ***/

static const char directory_results[]   = "../../results";       // Directory for performance results
static const char folder_perf_instant[] = "performance_instant"; // Folder name for instant performance results
static const char folder_perf_overall[] = "performance_overall"; // Folder name for overall performance results

/*** Global variables ***/

static char* super_name;    // Supervisor's unique identification name
static unsigned int offset; // Offset to the supervisor's flock

static WbDeviceTag emitter;                       // Radio node
static WbNodeRef  robots            [FLOCK_SIZE]; // Robots nodes
static WbFieldRef robots_translation[FLOCK_SIZE]; // Robots translation fields
static WbFieldRef robots_rotation   [FLOCK_SIZE]; // Robots rotation fields
static float loc[FLOCK_SIZE][3];		          // Robots location in the flock

static float perf_orientation; // Performance metric for orientation
static float perf_cohesion;    // Performance metric for cohesion
static float perf_velocity;    // Performance metric for velocity
static float perf_instant;     // Performance instant
static float perf_overall;     // Performance overall

/*** Data types definition ***/

/*** Functions declaration ***/

/*** Functions implementation ***/

/*
 * Robert Jenkins' 96 bit Mix Function
 * http://web.archive.org/web/20070111091013/http://www.concentric.net/~Ttwang/tech/inthash.htm
 * http://web.archive.org/web/20070110173527/http://burtleburtle.net:80/bob/hash/doobs.html
 */
unsigned long mix(unsigned long a, unsigned long b, unsigned long c)
{
    a=a-b;  a=a-c;  a=a^(c >> 13);
    b=b-c;  b=b-a;  b=b^(a << 8);
    c=c-a;  c=c-b;  c=c^(b >> 13);
    a=a-b;  a=a-c;  a=a^(c >> 12);
    b=b-c;  b=b-a;  b=b^(a << 16);
    c=c-a;  c=c-b;  c=c^(b >> 5);
    a=a-b;  a=a-c;  a=a^(c >> 3);
    b=b-c;  b=b-a;  b=b^(a << 10);
    c=c-a;  c=c-b;  c=c^(b >> 15);
    return c;
}

/*
 * Initialize the controller.
 */
void init(void)
{
	// Seed the random number generator
	const unsigned long seed = mix(clock(), time(NULL), getpid());
	srand(seed);
	
	// Unique identifier of the supervisor
	super_name = (char*) wb_robot_get_name();
	
	// Offset to the first robot in the team corresponding to this supervisor
	WbNodeRef root = wb_supervisor_node_get_root();
	WbFieldRef children = wb_supervisor_node_get_field(root, "children");
	unsigned int n = wb_supervisor_field_get_count(children);
	unsigned int nb_robots = 0;
	for (unsigned int i = 0; i < n; i++) {
		WbNodeRef node = wb_supervisor_field_get_mf_node(children, i);
		if (wb_supervisor_node_get_type(node) == WB_NODE_ROBOT) {
			nb_robots++;
		}
	}
	if (nb_robots > FLOCK_SIZE) {
		sscanf(super_name, "super%d", &offset);
		offset *= FLOCK_SIZE;
	} else {
		offset = 0;
	}
	
	// Radio emitter node
	emitter = wb_robot_get_device("emitter");
	
	// Robots position fields
	char robot_name[7];
	for (unsigned int i = 0; i < FLOCK_SIZE; i++) {
		sprintf(robot_name, "epuck%d", i+offset);
		robots[i] = wb_supervisor_node_get_from_def(robot_name);
		robots_translation[i] = wb_supervisor_node_get_field(robots[i], "translation");
		robots_rotation[i] = wb_supervisor_node_get_field(robots[i], "rotation");
	}
}

/*
 * Get the robots location.
 */
void location(void)
{
	for (unsigned int i = 0; i < FLOCK_SIZE; i++) {
		loc[i][0] = wb_supervisor_field_get_sf_vec3f(robots_translation[i])[0]; // X
		loc[i][1] = wb_supervisor_field_get_sf_vec3f(robots_translation[i])[2]; // Z
		loc[i][2] = wb_supervisor_field_get_sf_rotation(robots_rotation[i])[3]; // Theta
	}
}

/*
 * Compute the flock performance.
 */
void performance(void)
{
	perf_orientation = 0;
	perf_cohesion    = 0;
	perf_velocity    = 0;
	
	for (unsigned int i = 0; i < FLOCK_SIZE; i++) {
		perf_orientation += 0;
		perf_cohesion    += 0;
		perf_velocity    += 0;
	}
	
	perf_orientation = perf_orientation/FLOCK_SIZE;
	perf_cohesion    = 1/(1+perf_cohesion/FLOCK_SIZE);
	perf_velocity    = 0;
	
	perf_instant = perf_orientation*perf_cohesion*perf_velocity;
	perf_overall += perf_instant;
	
	if (VERBOSE) {
		printf("[%s] performance = %f\n", super_name, perf_instant);
	}
}

/*
 * Erase log files.
 */
void log_clear(void)
{
	char filename[256];
	FILE* file;
	
	sprintf(filename, "%s/%s/%s.txt", directory_results, folder_perf_instant, super_name);
	file = fopen(filename, "w");
	fclose(file);
	
	sprintf(filename, "%s/%s/%s.txt", directory_results, folder_perf_overall, super_name);
	file = fopen(filename, "w");
	fclose(file);
}

/*
 * Store the flock instant performance.
 */
void log_perf_instant(void)
{
	char filename[256];
	sprintf(filename, "%s/%s/%s.txt", directory_results, folder_perf_instant, super_name);
	FILE* file = fopen(filename, "a");
	fprintf(file, "%f %f %f %f\n", perf_orientation, perf_cohesion, perf_velocity, perf_instant);
	fflush(file);
	fclose(file);
}

/*
 * Store the flock overall performance.
 */
void log_perf_overall(void)
{
	char filename[256];
	sprintf(filename, "%s/%s/%s.txt", directory_results, folder_perf_overall, super_name);
	FILE* file = fopen(filename, "w");
	fprintf(file, "%f\n", perf_overall);
	fflush(file);
	fclose(file);
}

/*
 * Main function.
 */
int main(int argc, char *args[])
{
	// Variables declaration
	unsigned int time_steps; // Iterations counter
	
	// Initialization
	wb_robot_init();
	init();
	
	// Variables initialization
	time_steps = 0;
	perf_overall = 0;
	
	// Storage reset
	log_clear();
	
	// Simulation loop
	while (wb_robot_step(TIME_STEP) != -1) {
		// Update robots location
		location();
		// Compute performance metrics
		performance();
		// Store performance metrics
		log_perf_instant();
		// Increase iterations counter
		time_steps++;
	}
	
	// Overall performance
	perf_overall /= time_steps;
	if (VERBOSE) {
		printf("[%s] overall performance = %f\n", super_name, perf_overall);
	}
	
	// Result storage
	log_perf_overall();
	
    // Clean up and exit
    wb_robot_cleanup();
    return 0;
}
