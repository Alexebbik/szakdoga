//the directorys we'll need
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include <stdio.h>
#include <stdlib.h> 
#include <math.h>

//basic timestep for the world
#define TIME_STEP 64




//initializing the sensors and motors
WbDeviceTag wheels[2];

// wheel naming
const char* wheel_names[2] = {"wheel_fl", "wheel_fr"};
enum WheelDef{
  W_FL = 0,
  W_FR = 1
};



//setting up the devices for the controller
void init_devices(){
  int i;
//the motors 
  for (i=0;i<2;i++){
    wheels[i] = wb_robot_get_device(wheel_names[i]);
    wb_motor_set_velocity(wheels[i], 0);
    wb_motor_set_position(wheels[i], INFINITY);
  }
}


//getting the values of the sensors


//setting up the motors speeds
void set_differential_drive(double left, double right){
//these variables prevents the motors to get more than 100 velocity
  if(left > 100) left = 100;
  if(left < -100) left = -100;
  if(right > 100) right = 100;
  if(right < -100) right = -100;
  wb_motor_set_velocity(wheels[W_FL], left);
  wb_motor_set_velocity(wheels[W_FR], right);
}

int main(int argc, char **argv) {

//initialize divices and get starting target coordinates
  wb_robot_init();
  init_devices();


  while (wb_robot_step(TIME_STEP) != -1) {

    
    //the forward and turning speed
    double left = -50;
    double right = -50;
    

    
    set_differential_drive(left, right);
    
    double balkerek = wb_motor_get_velocity(wheels[W_FL]);
    double jobbkerek = wb_motor_get_velocity(wheels[W_FR]);
    
    printf("bal %f, jobb:%f\n",balkerek,jobbkerek);
  };

  

  wb_robot_cleanup();

  return 0;
}
