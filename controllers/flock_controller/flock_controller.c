/**************************************************************************************************
 * File:        flock_controller.c
 * Authors:     Zeki Doruk Erden, Michael Perret, Mickaël Salamin
 * Date:        Fall 2018
 * Description: Controller for DIS project.
 *************************************************************************************************/

/*** Specific libraries ***/

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
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

#define TIME_STEP  64 // Step duration [ms]
#define NB_SENSORS 8  // Number of distance sensors

/*** Global constants ***/

/*** Global variables ***/

static char* robot_name;
static unsigned int robot_id;
static WbDeviceTag left_motor, right_motor;
static WbDeviceTag left_encoder, right_encoder;
static WbDeviceTag emitter, receiver;
static WbDeviceTag sensors[NB_SENSORS];

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
	//Seed the random number generator
	const unsigned long seed = mix(clock(), time(NULL), getpid());
	srand(seed);
	
	//Unique identifier of the robot
	robot_name = (char*) wb_robot_get_name();
	sscanf(robot_name, "epuck%d", &robot_id);
	
	//Radio emitter and receiver
	emitter = wb_robot_get_device("emitter");
	receiver = wb_robot_get_device("receiver");
	wb_receiver_enable(receiver, TIME_STEP);
	
	//Wheel motors and encoders
    left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	left_encoder = wb_robot_get_device("left wheel sensor");
	right_encoder = wb_robot_get_device("right wheel sensor");
    wb_position_sensor_enable( left_encoder, TIME_STEP);
    wb_position_sensor_enable(right_encoder, TIME_STEP);
	wb_motor_set_position( left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);
	wb_motor_set_velocity( left_motor, 0);
	wb_motor_set_velocity(right_motor, 0);
	
	//Distance sensors
	char sensor_name[] = "ps0";
	for (unsigned int i = 0; i < NB_SENSORS; i++) {
		sensors[i] = wb_robot_get_device(sensor_name);
		wb_distance_sensor_enable(sensors[i], TIME_STEP);
		sensor_name[2]++;
	}
}

/*
 * Main function.
 */
int main(int argc, char *args[])
{
	//Initialization
	wb_robot_init();
	init();
	
	//Simulation loop
	while (wb_robot_step(TIME_STEP) != -1) {
		
	}
	
    //Clean up and exit
    wb_robot_cleanup();
    return 0;
}
