/**************************************************************************************************
 * File:        flock_super.c
 * Authors:     Zeki Doruk Erden, Michael Perret, Mickaël Salamin
 * Date:        Fall 2018
 * Description: Supervisor for DIS project.
 *************************************************************************************************/

/*** Specific libraries ***/

#include <webots/supervisor.h>
#include <webots/robot.h>
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

/*** Global constants ***/

/*** Global variables ***/

static char* super_name;
static unsigned int offset;
static WbDeviceTag emitter;
static WbNodeRef  robots            [FLOCK_SIZE]; // Robots nodes
static WbFieldRef robots_translation[FLOCK_SIZE]; // Robots translation fields
static WbFieldRef robots_rotation   [FLOCK_SIZE]; // Robots rotation fields

/*** Data types definition ***/

/*** Functions declaration ***/

/*** Functions implementation ***/

// Robert Jenkins' 96 bit Mix Function
// http://web.archive.org/web/20070111091013/http://www.concentric.net/~Ttwang/tech/inthash.htm
// http://web.archive.org/web/20070110173527/http://burtleburtle.net:80/bob/hash/doobs.html
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
	sscanf(super_name, "super%d", &offset);
	offset *= FLOCK_SIZE;
	
	// Radio emitter
	emitter = wb_robot_get_device("emitter");
	
	// Robots position
	char robot_name[7];
	for (unsigned int i = 0; i < FLOCK_SIZE; i++) {
		sprintf(robot_name, "epuck%d", i+offset);
		robots[i] = wb_supervisor_node_get_from_def(robot_name);
		robots_translation[i] = wb_supervisor_node_get_field(robots[i], "translation");
		robots_rotation[i] = wb_supervisor_node_get_field(robots[i], "rotation");
	}
}

/*
 * Main function.
 */
int main(int argc, char *args[])
{
	// Initialization
	wb_robot_init();
	init();
	
	// Simulation loop
	while (wb_robot_step(TIME_STEP) != -1) {
		
	}
	
    // Clean up and exit
    wb_robot_cleanup();
    return 0;
}
