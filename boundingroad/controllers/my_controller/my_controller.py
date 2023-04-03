"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import driver, Motor, DistanceSensor
from vehicle import Driver
import math
import sys
import random
from controller import DistanceSensor, Compass, GPS, Compass
from collections import deque
import pickle

with open(r'..\myGraph.pickle', 'rb') as handle:
    graph = pickle.load(handle)
with open(r'..\roadIDs.pickle', 'rb') as handle:
    road_ids = pickle.load(handle)
with open(r'..\graphCoords.pickle', 'rb') as handle:
    graph_coords = pickle.load(handle)
with open(r'..\roadCoords.pickle', 'rb') as handle:
    road_coords = pickle.load(handle)
with open(r'..\roadAngles.pickle', 'rb') as handle:
    road_angle = pickle.load(handle)
with open(r'..\roadLengths.pickle', 'rb') as handle:
    road_lengths = pickle.load(handle)
road_ids = list(map(int, road_ids))
for key in graph:
    graph[key] = list(map(int, graph[key]))

#print (road_lengths)
#print (graph_coords)
#print (road_coords)
#print (road_angle)
# create the driver instance.
driver = Driver()
# get the time step of the current world.
timestep = int(driver.getBasicTimeStep())

# Example graph represented as a dictionary where each vertex is a key and its value is a list of adjacent vertices
#graph = {0: [1, 2], 1: [0, 3, 5, 7], 2: [0, 4, 6, 8], 3: [1, 10], 4: [2, 11], 5: [1], 6: [2, 14], 7: [1], 8: [2, 16], 9: [10, 11], 10: [3, 9, 13], 11: [4, 9, 12], 12: [11, 13], 13: [10, 12], 14: [6, 15], 15: [14, 16], 16: [8, 15]}
#graph_coords = {0: [16, 10], 1: [10, 10], 2: [22.2, 10], 3: [10, 16], 4: [22.2, 16], 5: [4, 10], 6: [28.4, 10], 7: [10, 4], 8: [22.2, 4], 9: [16, 22], 10: [10, 22], 11: [22.2, 22.2], 12: [20.5, 29.5], 13: [11.5, 29.5], 14: [38, 9], 15: [39, -7], 16: [23, -6]}
#graph_coords = {0: [10, 10], 1: [22.2, 3.9], 2: [10, 22.2], 3: [22.2, 16.1], 4: [3.9, 10], 5: [10, 3.9], 6: [16.1, 10], 7: [10, 16.1], 8: [28.3, 10], 9: [11.5, 29.5], 10: [16.1, 22.2], 11: [22.2, 22.2], 12: [22.2, 10], 13: [20.5, 29.5], 14: [38, 9], 15: [39, -7], 16: [23, -6]}

def pathfinder(graph, start, end):
    # Queue to store the nodes to be visited
    queue = deque()
    # Dictionary to keep track of visited nodes and their parent
    visited = {start: None}
    # Enqueue the starting node
    queue.append(start)
    while queue:
        current_node = queue.popleft()
        # Check if we reached the end node
        if current_node == end:
            # Reconstruct the path by backtracking from the end node to the start node
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = visited[current_node]
            # Reverse the path to get it in the correct order (from start to end)
            path.reverse()
            return path
        # Visit the adjacent nodes
        for neighbor in graph[current_node]:
            if neighbor not in visited:
                visited[neighbor] = current_node
                queue.append(neighbor)
    # If we reached here, it means there is no path from start to end node
    return None


ds_names = ["dsRearLeft","dsLeft","dsFrontLeft","dsFrontRight","dsRight","dsRearRight"]
ds_list = []

for i in range(len(ds_names)):
    ds_list.append(DistanceSensor(ds_names[i]))
    ds_list[i].enable(timestep)
compass = Compass("compass")
compass.enable(timestep)
gps = GPS("gps")
gps.enable(timestep)


def getTargetBearing():
    north = compass.getValues()
    rad = math.atan2(north[1],north[0])
    target_bearing = math.atan2(-target_y + current_y, target_x - current_x)
    target_bearing -= rad
    target_bearing *= 180/math.pi
    if target_bearing < -90.0:
        target_bearing = target_bearing + 360
    target_bearing += 90
    if target_bearing >= 360.0:
        target_bearing = target_bearing - 360
    return target_bearing

target_x = 0
target_y = 0
path = []

start = 4
end = 17
small_target = 2.5
big_target = 3.5
target_radius = 2
close_range = 5
front_left_max = 359
front_left_min = 270
rear = 180
front_right_max = 90
front_right_min = 1
max_right_angle = 0.75
min_right_angle = 0.1
max_left_angle = -0.75
min_left_angle = -0.1
max_speed = 50
turn_speed = 10
min_speed = 1
reverse_speed = -2
max_ds = 1
min_ds = 0
millisec = 0.001

nonzero_length = []
for i in range(len(road_lengths)):
    if road_lengths[i] != 0:
        nonzero_length.append(road_lengths[i])
#print (nonzero_length)
# You should insert a getDevice-like function in order to get the
# instance of a device of the driver. Something like:
#  motor = driver.getDevice('motorname')
#  ds = driver.getDevice('dsname')
#  ds.enable(timestep)

#maximalis jobb driver.setSteeringAngle(0.75)
#maximalis bal driver.setSteeringAngle(-0.75)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
stop = 0
#turn = 1
time_helper = 0
path_helper = 1
counter = 0
angle = 0
speed = 10
end_coords = []
#print (road_ids)
#print (graph)
while driver.step() != -1:

    
    
    #print ("brake:",driver.getBrakeIntensity())
    current_x, current_y, z = gps.getValues()
    current_time = driver.getTime()
    target_bearing = getTargetBearing()
    
    if current_time / timestep == millisec:
        print("Lets start with getting a path!")
    
    if not path and path_helper == 1:
        #end = random.randint(0,11)
        while end == start:
            end = random.randint(0,len(graph)-1)
        #print("start:",start)
        #print("end:",end)
        path = pathfinder(graph, start, end)
        print("The new path is:",path)
        path.reverse()
        #print("reversed path:",path)
        end_target = path[0]
        end_coords = graph_coords[end_target]
        #print("end:",end_coords)
        current_target = path.pop()
        #print("The current target is point:",current_target)
        target_coords = graph_coords[current_target]
        target_x = target_coords[0]
        target_y = target_coords[1]
        #print("Its coordinates are x:",target_x,"y:",target_y)
        #print("Going on")
        path_helper = 0
        #print (path)
    
    # read current GPS values
    
    target_distance = math.fabs(current_x - target_x) + math.fabs(current_y - target_y)
    #print(target_distance)
    # Get the values from the compass
    
    #a target iranya 0 elore picit jobbra, 90 jobbra
    #180 mogotte, 270 balra, 360 elore picit balra

    

    #print current GPS position
    #print('Current position: ', current_x, current_y)
    
    ds_values = []
    for i in range(len(ds_list)):
        ds_values.append(ds_list[i].getValue())
        #print(ds_values[i])
    
    #print(ds_values[2])
    #print(ds_values[5])
    
    
    #print("vege:",math.fabs(current_x - end_coords[0]) + math.fabs(current_y - end_coords[1]))
    '''if (math.fabs(current_x - end_coords[0]) + math.fabs(current_y - end_coords[1])) < target_radius * 2:
        driver.setBrakeIntensity(1)
        speed = 0
        angle = 0'''
    
    
    
    if target_distance < target_radius:
        #print ("MEGVAN")
        if (math.fabs(current_x - end_coords[0]) + math.fabs(current_y - end_coords[1])) < target_radius:
            print("We reached our final target. We will need a new path!")
            start = end
            path_helper = 1
        speed = 0
        angle = 0
        stop = 1
        time_helper += 1
        #print(time_helper)
        if time_helper == 1:
            counter += 1
            stop_time = driver.getTime()
            print(counter - 1,". landing")
            #print ("stoptime:",stop_time)
            #print(time_string)
        if current_time > stop_time:
            current_target = path.pop()
            print("The current target is point:",current_target)
            target_coords = graph_coords[current_target]
            target_x = target_coords[0]
            target_y = target_coords[1]
            #print("Its coordinates are x:",target_x,"y:",target_y)
            #print("Going on")
            #print (path)
            stop = 0
            time_helper = 0
            #turn = 1
    
    if stop == 0:
    
        '''if target_bearing > front_left_min + (front_left_max - front_left_min) / 2 or target_bearing < front_right_max - (front_right_max - front_right_min) / 2:
            #print ("gyorsit")
            if target_distance >'''
        
        if target_bearing >= front_right_max and target_bearing < rear:
            angle = max_left_angle
            speed = reverse_speed
            #print("jobbhat")
        elif target_bearing >= rear and target_bearing <= front_left_min:
            angle = max_right_angle
            speed = reverse_speed
            #print("balhat")
            
        elif target_bearing > front_right_min  and target_bearing < front_right_max:
            speed = ((target_bearing - front_right_min) / (front_right_max - front_right_min)) * (min_speed - turn_speed) + turn_speed
            angle = ((target_bearing - front_right_min) / (front_right_max - front_right_min)) * (max_right_angle - min_right_angle) + min_right_angle
            #print("jobbelol")
            
        elif target_bearing < front_left_max  and target_bearing > front_left_min:
            speed = ((target_bearing - front_left_min) / (front_left_max - front_left_min)) * (turn_speed - min_speed) + min_speed
            angle = ((target_bearing - front_left_min) / (front_left_max - front_left_min)) * (min_left_angle - max_left_angle) + max_left_angle
            #print("balelol")
            
        elif target_bearing >= front_left_max or target_bearing <= front_right_min:
            #turn = 0
            angle = 0
            #print("egyenes")
            
            
        if road_angle[current_target] != 0:
            target_radius = big_target
            if ds_values[4] == min_ds:
                if target_bearing > front_left_min:
                    angle = max_left_angle
                    #print("balra")
                if target_bearing > front_left_min + (front_left_max - front_left_min) / 2:
                    angle = 0
                    #print("elore")
                    if target_distance > road_angle[current_target] / 2:
                        angle = max_left_angle / 2
                        #print ("demiert")
                if target_bearing < front_right_max:
                    angle = max_right_angle
                    #print("jobbra")
                    if target_bearing < front_right_max - (front_right_max - front_right_min) / 2:
                        angle = 0
                        #print("elore")
                        if target_distance > road_angle[current_target] / 2:
                            angle = max_right_angle / 2
                            #print ("demiert")
            if target_distance < road_angle[current_target] / 2:
                angle = max_right_angle / 2
                
        elif graph_coords[current_target] == road_coords[current_target]:
            target_radius = big_target
            #print("nagytarget")
            if ds_values[4] == min_ds:
                if target_bearing > front_left_min:
                    angle = max_left_angle
                    #print("balra")
                if target_bearing > front_left_min + (front_left_max - front_left_min) / 2:
                    angle = 0
                if target_bearing < front_right_max:
                    angle = max_right_angle
                    #print("jobbra")
            if target_bearing > front_left_min + (front_left_max - front_left_min) / 2:
                    angle = 0
                    print("elore")
                    if target_distance > min(nonzero_length):
                        speed = target_distance
                        print("eloremegint")
                        if speed > max_speed:
                            speed = max_speed
            
        elif road_angle[current_target] == 0 and graph_coords[current_target] != road_coords[current_target]:
            #print("a celpont egy egyenes")
            target_radius = small_target
            #print("faltartas")
            
            if ds_values[4] == min_ds:
                if target_bearing > front_left_min:
                    angle = max_left_angle
                    #print("balra")
                if target_bearing > front_left_min + (front_left_max - front_left_min) / 2:
                    angle = min_left_angle
                    #print("elore")
                    if target_distance > road_lengths[current_target] / 2:
                       angle = min_left_angle
                if target_bearing < front_right_max:
                    angle = max_right_angle
                    #print("jobbra")
            if target_distance < road_lengths[current_target] / 2:
                       angle = min_right_angle
            if target_distance > min(nonzero_length):
                speed = target_distance
                if speed > max_speed:
                    speed = max_speed
                

            
        if ds_values[2] > min_ds and ds_values[2] > ds_values[3]:
            angle = ((ds_values[2] - min_ds) / (max_ds - min_ds)) * (max_right_angle - min_right_angle) + min_right_angle
            if target_bearing >= front_right_max and target_bearing <= front_left_min:
                speed = reverse_speed
                angle = max_right_angle
            #print("balelol")
        if ds_values[3] > min_ds and ds_values[3] > ds_values[2]:
            angle = ((ds_values[3] - min_ds) / (max_ds - min_ds)) * (max_left_angle + min_left_angle) - min_left_angle
            if target_bearing >= front_right_max and target_bearing <= front_left_min:
                speed = reverse_speed
                angle = max_left_angle
            #print("jobbelol")
        if ds_values[1] > min_ds:
            angle = ((ds_values[1] - min_ds) / (max_ds - min_ds)) * (max_right_angle - min_right_angle) + min_right_angle
            if target_bearing >= front_right_max and target_bearing <= front_left_min:
                speed = reverse_speed
                angle = max_right_angle
            #print("bal")
        if ds_values[4] > min_ds:
            angle = ((ds_values[4] - min_ds) / (max_ds - min_ds)) * (max_left_angle + min_left_angle) - min_left_angle
            if target_bearing >= front_right_max and target_bearing <= front_left_min:
                speed = reverse_speed
                angle = max_left_angle
            #print("jobb")
        if ds_values[0] > max_ds /2:
            angle = max_right_angle
            speed = turn_speed
            print("balhatul")
        elif ds_values[5] > max_ds /2:
            angle = max_left_angle
            speed = turn_speed
            print("jobbhatul")
        if angle > max_right_angle:
            angle = max_right_angle
        if angle < max_left_angle:
            angle = max_left_angle
        
        
        
        
        driver.setSteeringAngle(angle)
        driver.setCruisingSpeed(speed)
        
    else:
        driver.setCruisingSpeed(0)
        driver.setSteeringAngle(0)
        
    
    
    #print(ds_values[4])
    #print (graph_coords[current_target])
    #print (road_coords[current_target])
    #print("angle:",angle)
    print("speed:",speed)
    #print("balhatul:",ds_values[0])
    #print("jobbhatul:",ds_values[5])
    #print(turn)
    #print(stop)
    #main controller
    
    #print ("bear:",target_bearing)
    #print ("dist:",target_distance)
    
    #print ("speed:",speed)
    #print ("angle:",angle)
    #print("ittavege")
    #print("ds:",dsLValue)
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    
    pass

# Enter here exit cleanup code.
