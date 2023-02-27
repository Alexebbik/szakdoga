//the directorys we'll need
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/vehicle/driver.h>
#include <webots/vehicle/car.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include <stdio.h>
#include <stdlib.h> 
#include <math.h>

//basic timestep for the world
#define TIME_STEP 64




//graph structure creating and walkthrough
// define the maximum of vertices in the graph
#define N 9

//data structure to store a graph object
struct Graph{

  //an array of pointers to node to represent an adjacency list
  struct Node* head[N];
};

//data structure to store adjacency list nodes of the graph
struct Node{
  int dest;
  struct Node* next;
};

//data structure to store a graph edge
struct Edge{
  int src, dest;
};

//function to create an adjacency list from specified edges
struct Graph* createGraph(struct Edge edges[],int n){

  //allocate storage for the graph data structure
  struct Graph* graph = (struct Graph*)malloc(sizeof(struct Graph));
  
  //initialize head pointer for all vertices
  for (int i = 0; i < N; i++){
    graph->head[i] = NULL;
  }
  
  //add edges to the directed graph one by one
  for (int i = 0; i < n; i++){
  
    //get the source of the destination vertex
    int src = edges[i].src;
    int dest = edges[i].dest;
    
    //allocate a new node of adjacency list from src to dest
    struct Node* newNode = (struct Node*)malloc(sizeof(struct Node));
    newNode->dest = dest;
    
    //point new node to the current head
    newNode->next = graph->head[src];
    
    //point head pointer to the new node
    graph->head[src] = newNode;
    
    //allocate a new node of adjacency list from dest to src
    newNode = (struct Node*)malloc(sizeof(struct Node));
    newNode->dest = src;
    
    //point new node to the current head
    newNode->next = graph->head[dest];
    
    //point head pointer to the new node
    graph->head[dest] = newNode;
    
  }
  return graph;
}

//function to print adjacency list representation of graph
void printGraph(struct Graph* graph){

  for (int i = 0; i < N; i++){
    //print current vertex and all its neighbours
    struct Node* ptr = graph->head[i];
    while (ptr != NULL){
      printf("(%d -> %d)\t", i, ptr->dest);
      ptr = ptr->next;
    }
    printf("\n");
  }
}





//initializing the sensors and motors
WbDeviceTag ds[3];
WbDeviceTag wheels[2];
WbDeviceTag gps;
WbDeviceTag comp;

// wheel naming
const char* wheel_names[4] = {"wheel_fl", "wheel_fr"};
enum WheelDef{
  W_FL = 0,
  W_FR = 1
};

// distance sensor naming
const char* ds_names[3] = {"ds_left", "ds_right","ds_mid"};
enum DistanceSensorDef{
  DS_LEFT = 0,
  DS_RIGHT = 1,
  DS_MID =2
};

//setting up the devices for the controller
void init_devices(){
  int i;
 
//the distance sensors
  for (i=0;i<3;++i){
    ds[i] = wb_robot_get_device(ds_names[i]);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }
  
//the motors 
  for (i=0;i<2;++i){
    wheels[i] = wb_robot_get_device(wheel_names[i]);
    wb_motor_set_velocity(wheels[i], 0);
    wb_motor_set_position(wheels[i], INFINITY);
  }
  
//the gps
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

//the compass
  comp = wb_robot_get_device("compass");
  wb_compass_enable(comp, TIME_STEP); 
}




double target_x;
double target_y;
/*

//x and y coordinates of the targets
double xTarget[5] = {0.5, 1, 4.5, 2, 2.5};
double yTarget[5] = {0.5, 2, 0.5, 4.5, 2.5};

//the before variable prevents the function to give the same target again
int before = 0;
//setting up the target location and the
void set_target(){
  int random_number = rand() % 5;
  if (random_number==before)
  {
    random_number = rand() % 5;
  }
  target_x = xTarget[random_number];
  target_y = yTarget[random_number];
  printf("Target:%d, its coordinates are: %gx / %gy\n",random_number+1,target_x,target_y);
  before=random_number;
}*/

void set_target(){
    target_x = 4.5;
    target_y = 1;
}

//current position coordinates
double pos_x = 0;
double pos_y = 0;
//the target's direction
double target_bearing;
//the values of the distance sensors
double ds_values[3];
//the radius which we count as successful landing
double target_radius = 0.01;

//getting the values of the sensors
void update_sensor_readings(){
  int i;

//the values of the gps
  const double *gps_values = wb_gps_get_values(gps);
  pos_x = gps_values[0];
  pos_y = gps_values[2];

//the values of the distance sensors  
  for (i=0;i<3;++i){
    ds_values[i] = wb_distance_sensor_get_value(ds[i]);
  }
  
//helper variable for counting
  double rad = 0;
//the value of the compass
  const double* north = wb_compass_get_values(comp);
  rad = atan2(north[2],north[0]);
  //counting out the targets direction in degrees
  target_bearing = atan2(-target_y + pos_y,target_x - pos_x);
  target_bearing -= rad;
  target_bearing *= 180 / M_PI;
  if(target_bearing < -180)
  {
    target_bearing += 360;
  }
  if(target_bearing > 180)
  {
    target_bearing -= 360;
  }
  //0 elore, -90 jobbra, 90 balra, 180 mogotte
  target_bearing = target_bearing+90;
}

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
  set_target();
  
  //the time when we landed on the target
  double stop_time = 0;
  //this helps counting the time
  int helper = 0;
  //this tells whether we have to turn or we are facing in the right direction
  int turn = 1;
  //this tells if we are on the target
  int stop = 0;
  //this counts the successful landings
  int counter = 0;
  
  
  
  //directed graph implementation in C
  //input array containing edges of the graph
  //(x,y) pair in the array represents and edge from x to y
  struct Edge edges[] =
  {
    {1,3},{1,5},{1,7},{1,0},{0,2},{2,4},{2,6},{2,8}
  };
  
  //calculate the total number of edges
  int n = sizeof(edges)/sizeof(edges[0]);
  
  //construct a graph from the given edges
  struct Graph *graph = createGraph(edges, n);
  
  //function to print adjacency list representation of a graph
  printGraph(graph);
  
  

  while (wb_robot_step(TIME_STEP) != -1) {
    
    //getting the sensor values every cycle
    update_sensor_readings();
    //the current time of the simulation
    double time = 0.0;
    time=wb_robot_get_time(); 
    
    //the forward and turning speed
    double left = 20;
    double right = 20;
    
    //how far we are from the target
    double target_distance = fabs(pos_x - target_x) + fabs(pos_y - target_y);

    
    
    if (time >= 5)
      {left = 0;
      right = 0;}
    
    set_differential_drive(left, right);
  };

  

  wb_robot_cleanup();

  return 0;
}
