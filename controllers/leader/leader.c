/*****************************************************************************/
/* File:         leader.cc                                                   */
/* Version:      1.1 -> New double key recognition                           */
/* Date:         12-Oct-15                                                   */
/* Description:  Allows to remote control a robot using the arrow keys       */
/*                                                                           */
/* Author: 	 22-Oct-04 by nikolaus.correll@epfl.ch                       */
/* Last revision:12-oct-15 by Florian Maushart				     */
/*****************************************************************************/

#include <stdio.h>
#include <math.h>

#include <webots/robot.h>
/*Webots 2018b*/
#include <webots/motor.h>
/*Webots 2018b*/
#include <webots/keyboard.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>


#define NB_SENSORS           8
#define BIAS_SPEED           200
#define DEL_SPEED            BIAS_SPEED/2
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/

/*Webots 2018b*/
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/

int Interconn[16] = {-5,-15,-20,6,4,6,3,5,4,4,6,-18,-15,-5,5,3};	
WbDeviceTag ds[NB_SENSORS];           // Handle for the infrared distance sensors
WbDeviceTag emitter;                  // Handle for the emitter node
int robot_id;                         // Unique robot ID

static void reset(void) {
  int i;
  
  wb_robot_init();

  emitter = wb_robot_get_device("emitter"); 
  
  /*Webots 2018b*/
  //get motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  /*Webots 2018b*/   

  char s[4]="ps0";
  for(i=0; i<NB_SENSORS;i++) {
    ds[i]=wb_robot_get_device(s);      // the device name is specified in the world file
    s[2]++;                            // increases the device number
  }
  char* robot_name; robot_name=(char*) wb_robot_get_name(); 

  wb_keyboard_enable(64);
  sscanf(robot_name,"epuck%d",&robot_id);            // read robot id from the robot's name
  printf("Reset: robot %d\n",robot_id);
}


int main(){
  
  int msl,msr;                      // motor speed left and right
  /*Webots 2018b*/
  float msl_w, msr_w;
  /*Webots 2018b*/
  int distances[NB_SENSORS];        // array keeping the distance sensor readings
  char outbuffer[255];              // buffer for the emitter node
  
  int i;

  reset();
  
  for(i=0;i<NB_SENSORS;i++)
    wb_distance_sensor_enable(ds[i],64);

  

  for(;;){
    
    msl=0; msr=0;                  
    
    int sensor_nb;
    // read sensor values and calculate motor speeds
    for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++){  
      distances[sensor_nb]=wb_distance_sensor_get_value(ds[sensor_nb]);
      
      msr += distances[sensor_nb] * Interconn[sensor_nb];
      msl += distances[sensor_nb] * Interconn[sensor_nb + NB_SENSORS];
    }

    msl /= 400; msr /= 400;        // Normalizing speeds
   
    // Adapted version of the original code that is now able to detect left/right movements while the
    // forward arrow is still pressed
    
    int key = 0; 				// key that is used to determine how to adapt the speed
    int key1 = wb_keyboard_get_key();	// key that is currently detected
    //int key2 = wb_keyboard_get_key();	// key that might be currently detected as well
    int key2 = key1;
    int sign = 1;
    
    sign = 1; // used to invert left and right when robot goes backwards
    
    if(key2)  // if a second key is detected -> use the second one
    {
       key=key2; 
       if(key1 == 317) sign = -1; // if going backwards set sign accordingly
    }
    else      // if no second key is detected -> use the first one
    {
      key = key1;
      if(key1 == 317) sign = -1; // if going backwards set sign accordingly
    }
    
    // adapt speed to key pressed
    if(key){
      switch (key)
      {
      case 314 : {
        msl += BIAS_SPEED-sign*DEL_SPEED;
        msr += BIAS_SPEED+sign*DEL_SPEED;
        break;}
      case 316 : {
        msl += BIAS_SPEED+sign*DEL_SPEED;
        msr += BIAS_SPEED-sign*DEL_SPEED;
        break;}	
      case 315 : {
        msl += BIAS_SPEED;
        msr += BIAS_SPEED;
        break;}	
      case 317 : {
        msl -= BIAS_SPEED;
        msr -= BIAS_SPEED;
        break;}
      } 
      //printf("Key: %d\n",key);
    }
      
      
    /*Webots 2018b*/
    // Set speed
    msl_w = msl*MAX_SPEED_WEB/1000;
    msr_w = msr*MAX_SPEED_WEB/1000;
    wb_motor_set_velocity(left_motor, msl_w);
    wb_motor_set_velocity(right_motor, msr_w);
    //wb_differential_wheels_set_speed(msl,msr);
    /*Webots 2018b*/
		
    wb_robot_step(64); // Executing the simulation for 64ms
  }
  
  wb_robot_cleanup();
}  
  
