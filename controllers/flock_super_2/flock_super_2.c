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

#define WORLD_CROSSING 0

#define TIME_STEP  64 // Step duration [ms]
#define FLOCK_SIZE 5  // Number of robots in the flock

#define VERBOSE 1 // If messages should be printed

#define DELTA_T         0.064   // Timestep [s]

/*** Symbolic macros ***/

#define MAX(a,b) ((a < b) ? b : a)

/*** Global constants ***/

#if WORLD_CROSSING
static const float migration[2][2] = {{-100, 0}, {100, 0}}; // Migration vector for world crossing {team0, team1}
#else
static const float migration[1][2] = {{25, 0}}; // Migration vector for world obstacles {team0}
#endif

static const float velocity_max = 0.12874; // Maximum speed of a robot [m/s]

static const char directory_results[]   = "../../results";       // Directory for performance results
static const char folder_perf_instant[] = "performance_instant"; // Folder name for instant performance results
static const char folder_perf_overall[] = "performance_overall"; // Folder name for overall performance results

/*** Global variables ***/

static char*        super_name; // Supervisor's unique identification name
static unsigned int super_team; // Supervisor's unique identification team
static unsigned int offset;     // Offset to the supervisor's flock

static WbDeviceTag emitter;                       // Radio node
static WbNodeRef  robots            [FLOCK_SIZE]; // Robots nodes
static WbFieldRef robots_translation[FLOCK_SIZE]; // Robots translation fields
static WbFieldRef robots_rotation   [FLOCK_SIZE]; // Robots rotation fields
static float position[FLOCK_SIZE][3];		      // Robots position in the flock

static float      center_of_mass[2]; // Center of mass of the flock
static float prev_center_of_mass[2]; // Center of mass of the flock at the previous time step

static float perf_orientation; // Performance metric for orientation
static float perf_cohesion;    // Performance metric for cohesion
static float perf_velocity;    // Performance metric for velocity
static float perf_instant;     // Performance instant
static float perf_overall;     // Performance overall

/*** Data types definition ***/

/*** Functions declaration ***/

unsigned long mix(unsigned long a, unsigned long b, unsigned long c);
void init(void);
void location(void);
void performance(void);
void log_clear(void);
void log_perf_instant(void);
void log_perf_overall(void);

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
		super_team = offset;
		offset *= FLOCK_SIZE;
	} else {
		super_team = 0;
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
		position[i][0] = wb_supervisor_field_get_sf_vec3f(robots_translation[i])[0]; // X
		position[i][1] = wb_supervisor_field_get_sf_vec3f(robots_translation[i])[2]; // Z
		position[i][2] = wb_supervisor_field_get_sf_rotation(robots_rotation[i])[3]; // Theta
	}
}

/*
 * Compute the flock performance.
 */
void performance(void)
{
	unsigned int i, j;    // Loop counters
	float migratory_urge; // Migration orientation
	
	float temp_perf_orientation_vec[2]; // Temporary performance metric for orientation vector
	float temp_perf_orientation;        // Temporary performance metric for orientation
	float temp_perf_cohesion;           // Temporary performance metric for cohesion
	float temp_perf_velocity;           // Temporary performance metric for velocity
	
	// Reset performance metrics
	temp_perf_orientation_vec[0] = 0;
	temp_perf_orientation_vec[1] = 0;
	temp_perf_orientation = 0;
	temp_perf_cohesion    = 0;
	temp_perf_velocity    = 0;
	perf_orientation = 0;
	perf_cohesion    = 0;
	perf_velocity    = 0;
	
	// Reset center of mass
	for (j = 0; j < 2; j++) {
		prev_center_of_mass[j] = center_of_mass[j];
		center_of_mass[j] = 0;
	}
	// Compute center of mass
	for (i = 0; i < FLOCK_SIZE; i++) {
		for (j = 0; j < 2; j++) {
			center_of_mass[j] += position[i][j];
		}
	}
	for (j = 0; j < 2; j++) {
		center_of_mass[j] /= FLOCK_SIZE;
	}
	
	// Compute migration orientation
	migratory_urge = atan2f(migration[super_team][1]-prev_center_of_mass[1], migration[super_team][0]-prev_center_of_mass[0]);
	// Keep migration orientation within [0, 2pi]
	if (migratory_urge > 2*M_PI)
		migratory_urge -= 2*M_PI;
	if (migratory_urge < 0)
		migratory_urge += 2*M_PI;
	
	// Compute performance metrics
	for (i = 0; i < FLOCK_SIZE; i++) {
		temp_perf_orientation_vec[0] += cosf(position[i][2]);
		temp_perf_orientation_vec[1] += sinf(position[i][2]);
		temp_perf_cohesion           += sqrtf(powf(position[i][0]-center_of_mass[0],2) + powf(position[i][1]-center_of_mass[1],2));
	}
	temp_perf_orientation = sqrtf(powf(temp_perf_orientation_vec[0],2) + powf(temp_perf_orientation_vec[1],2));
	temp_perf_velocity    = MAX((center_of_mass[0]-prev_center_of_mass[0])/DELTA_T*cosf(migratory_urge) + (center_of_mass[1]-prev_center_of_mass[1])/DELTA_T*sinf(migratory_urge), 0);
	
	// Normalize performance metrics
	perf_orientation = temp_perf_orientation/FLOCK_SIZE;
	perf_cohesion    = 1/(1+temp_perf_cohesion/FLOCK_SIZE);
	perf_velocity    = temp_perf_velocity/velocity_max;
	
	if (VERBOSE) {
		printf("[%s] Orientation = %f || Cohesion = %f || Velocity = %f \n", super_name, perf_orientation, perf_cohesion, perf_velocity);
	}
	
	// Update general performance
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
