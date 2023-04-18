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

'''with open(r'..\vehicleArgs.pickle', 'rb') as handle:
    vehicle_args = pickle.load(handle)'''
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


ds_names = ["dsRearLeft","dsLeft","dsFrontLeft","dsFrontRight","dsRight","dsRearRight","dsFront"]
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


#auto0 65 10.9 rot 3.14
#auto1 79.75 75.5 rot -1.83
#auto2 32 61 rot 1.57
#auto3 58 79.5 rot 1.57
start = 0
end = 11
small_target = 2.5
big_target = 3.5
target_radius = 2
front_left_max = 359
front_left_min = 270
rear = 180
front = 0
front_right_max = 90
front_right_min = 1
max_right_angle = 0.75
min_right_angle = 0.1
max_left_angle = -0.75
min_left_angle = -0.1
max_speed = 50
turn_speed = 10
min_speed = 1
stop_speed = 0
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
#angle = 0
#speed = 10
end_coords = []
current_target = 0
last_target = 0
reached = 0
path_counter1 = 0
path_counter2 = 0
path_counter3 = 0
path1_1 = 16
path1_2 = 19
path2_1 = 22
path2_2 = 16
path2_3 = 5
path3_1 = 14
path3_2 = 11
path3_3 = 9
target0 = 0
target1 = 1
target2 = 2
#print (road_ids)
#print (graph)

controller = driver.getSelf()
controllerArg = controller.getField('controllerArgs').getMFString(0)
controllerArg = int(controllerArg)


while driver.step() != -1:

    #print (controllerArg)
    '''if controllerArg == 3:
        print (path_helper)'''
    
    #print ("brake:",driver.getBrakeIntensity())
    current_x, current_y, z = gps.getValues()
    current_time = driver.getTime()
    target_bearing = getTargetBearing()
    
    #if vehicle_args
    '''if current_time / timestep / millisec == 1 and controllerArg == 0:
        print("Lets start with getting a path!")'''
    
    if controllerArg == 0:
        if not path and path_helper == 1:
            #end = random.randint(0,11)
            while end == start:
                end = random.randint(0,len(graph)-1)
            #print("start:",start)
            #print("end:",end)
            path = pathfinder(graph, start, end)
            #print("The new path is:",path)
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
    
    if controllerArg == 1:
        if not path and path_helper == 1:
            #end = random.randint(0,11)
            while end == start:
                end = random.randint(0,len(graph)-1)
            #print("start:",start)
            #print("end:",end)
            if path_counter1 % 2 == target0:
                path = pathfinder(graph, path1_1, path1_2)
            elif path_counter1 % 2 == target1:
                path = pathfinder(graph, path1_2, path1_1)
            #print("The new path is:",path)
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
            path_counter1 = path_counter1 + 1
            #print (path_counter)
            #print (path)
            
    
    if controllerArg == 2:
        if not path and path_helper == 1:
            #end = random.randint(0,11)
            while end == start:
                end = random.randint(0,len(graph)-1)
            #print("start:",start)
            #print("end:",end)
            if path_counter2 % 3 == target0:
                path = pathfinder(graph, path2_1, path2_2)
            elif path_counter2 % 3 == target1:
                path = pathfinder(graph, path2_2, path2_3)
            elif path_counter2 % 3 == target2:
                path = pathfinder(graph, path2_3, path2_1)
            #print("The new path is:",path)
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
            path_counter2 = path_counter2 + 1
            #print (path_counter)
            #print (path) 
    
    
    if controllerArg == 3:
        if not path and path_helper == 1:
            #end = random.randint(0,11)
            while end == start:
                end = random.randint(0,len(graph)-1)
            #print("start:",start)
            #print("end:",end)
            if path_counter3 % 3 == target0:
                path = pathfinder(graph, path3_1, path3_2)
            elif path_counter3 % 3 == target1:
                path = pathfinder(graph, path3_2, path3_3)
            elif path_counter3 % 3 == target2:
                path = pathfinder(graph, path3_3, path3_1)
            #print("The new path is:",path)
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
            path_counter3 = path_counter3 + 1
            #print (path_counter)
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
            reached += 1
            if reached == 1:
                if controllerArg == 0:
                    #print("We reached our final target.")
                    path_helper = 0
            if controllerArg == 1 and (current_target == path1_2 or current_target == path1_1):
                path_helper = 1
                #print(path)
            if controllerArg == 2 and (current_target == path2_1 or current_target == path2_2 or current_target == path2_3):
                path_helper = 1
                #print(path)
            if controllerArg == 3 and (current_target == path3_1 or current_target == path3_2 or current_target == path3_3):
                path_helper = 1
                #print(path)
            start = end
            
            speed = stop_speed
            angle = front
            stop = 1
        
        time_helper += 1
        #print(time_helper)
        if time_helper == 1:
            counter += 1
            stop_time = driver.getTime()
            '''if controllerArg == 0:
                print(counter - 1,". landing")'''
            #print ("stoptime:",stop_time)
            #print(time_string)
        if current_time > stop_time and (math.fabs(current_x - end_coords[0]) + math.fabs(current_y - end_coords[1])) > target_radius:
            last_target = current_target
            current_target = path.pop()
            '''if controllerArg == 0:
                print("The current target is point:",current_target)'''
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
            angle = front
            #print("egyenes")
            
          
        if road_angle[current_target] != 0:
            target_radius = big_target
            #print ("a cel egy kanyar")
            if target_distance > road_angle[current_target]:
                if ds_values[4] == min_ds:
                    angle = min_right_angle
                if target_bearing > front_right_max - (front_right_max - front_right_min) / 2 and target_bearing < front_right_max:
                    angle = max_right_angle
                    if ds_values[4] == min_ds:
                        angle = max_right_angle / 2
                if target_bearing < front_right_min + (front_right_min + front_right_max) / 2 and target_bearing > front_right_min:
                    angle = min_right_angle
                    if ds_values[4] == min_ds:
                        angle = max_right_angle / 2
                    if target_distance > min(nonzero_length):
                        speed = target_distance
                if target_bearing > front_left_min + (front_left_max - front_left_min) / 2 and target_bearing < front_left_max:
                    angle = min_right_angle
                    if target_distance > min(nonzero_length):
                        speed = target_distance
                if target_bearing > front_left_max - (front_left_min - front_left_max) / 2 and target_bearing > front_left_min:
                    angle = max_left_angle
            elif target_distance < road_angle[current_target]:
                if ds_values[4] == min_ds:
                    angle = max_right_angle / 2
                    
                
        elif graph_coords[current_target] == road_coords[current_target]:
            #print ("a cel egy kereszt")
            target_radius = big_target
            #print("nagytarget")
            if target_bearing > front_left_min:
                angle = max_left_angle
                #print("balra")
            if target_bearing > front_left_min + (front_left_max - front_left_min) / 2:
                angle = min_left_angle / 2
                if target_distance > min(nonzero_length):
                    speed = target_distance
            if target_bearing < front_right_max:
                angle = max_right_angle
                #print("jobbra")
            if ds_values[4] == min_ds:
                angle = min_right_angle
                    
            
        elif road_angle[current_target] == 0 and graph_coords[current_target] != road_coords[current_target]:
            #print("a celpont egy egyenes")
            target_radius = small_target
            if target_distance > road_lengths[current_target] / 2:
                if target_bearing > front_right_max - (front_right_max - front_right_min) / 2 and target_bearing < front_right_max:
                    angle = max_right_angle
                    
                if target_bearing < front_right_min + (front_right_min + front_right_max) / 2 and target_bearing > front_right_min:
                    angle = min_right_angle
                    if road_angle[last_target] != 0:
                        angle = max_right_angle / 2
                    else:
                        angle = max_right_angle
                        #print ("what")
                
                if target_bearing < front_left_min + (front_left_max - front_left_min) / 2 and target_bearing > front_left_min:
                    angle = max_left_angle / 2
                if target_bearing > front_left_max - (front_left_max - front_left_min) / 2 and target_bearing < front_left_max:
                    angle = front
            elif target_distance < road_lengths[current_target] / 2:
                if ds_values[4] == min_ds:
                    angle = min_right_angle
                if target_bearing < front_right_max:
                    #print("wow")
                    angle = max_right_angle / 2
                if target_distance > min(nonzero_length):
                    speed = target_distance
                
                
            
            
                   
        
        
                

            
        if ds_values[2] > min_ds and ds_values[2] > ds_values[3]:
            angle = ((ds_values[2] - min_ds) / (max_ds - min_ds)) * (max_right_angle - min_right_angle) + min_right_angle
            if ds_values[2] > max_ds / 2:
                angle = max_right_angle
                speed = turn_speed / 2
            if target_bearing >= front_right_max and target_bearing <= front_left_min:
                speed = reverse_speed
                angle = max_right_angle
            #print("balelol")
        if ds_values[3] > min_ds and ds_values[3] > ds_values[2]:
            angle = ((ds_values[3] - min_ds) / (max_ds - min_ds)) * (max_left_angle + min_left_angle) - min_left_angle
            if ds_values[3] > max_ds / 2:
                angle = max_left_angle
                speed = turn_speed / 2
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
        if ds_values[0] > max_ds / 2:
            angle = max_right_angle
            speed = turn_speed
            #print("balhatul")
        elif ds_values[5] > max_ds / 2:
            angle = max_left_angle
            speed = turn_speed
            #print("jobbhatul")
        
        if ds_values[6] > min_ds:
            if target_bearing > front_left_max - (front_left_max - front_left_min) / 2 or target_bearing < front_right_min + (front_right_min + front_right_max) / 2:
                angle = max_left_angle / 2
                speed = min_speed
                if ds_values[6] > max_ds / 2:
                    angle = max_left_angle
            if ds_values[2] > min_ds:
                angle = max_right_angle
                speed = min_speed

        if graph_coords[last_target] == road_coords[last_target]:
            if target_distance > road_lengths[current_target] / 2:
                if target_bearing > front_right_min and target_bearing < front_right_max:
                    if ds_values[3] > min_ds:
                        speed = stop_speed
                if target_bearing > front_left_max - (front_left_max - front_left_min) / 2 and target_bearing < front_left_max:
                    if ds_values[3] > max_ds / 2:
                        speed = min_speed / 2
                    if ds_values[1] > min_ds:
                        speed = min_speed / 2
                    if ds_values[2] > min_ds:
                        speed = stop_speed
                if target_bearing < front_left_min + (front_left_max - front_left_min) / 2 and target_bearing > front_left_min:
                    if ds_values[6] > max_ds / 2 or ds_values[3] > min_ds:
                        speed = stop_speed
                    if ds_values[2] > min_ds:
                        speed = stop_speed


        
        if angle > max_right_angle:
            angle = max_right_angle
        if angle < max_left_angle:
            angle = max_left_angle
        if speed > max_speed:
            speed = max_speed
        
        
        
        driver.setSteeringAngle(angle)
        driver.setCruisingSpeed(speed)
        
    else:
        driver.setCruisingSpeed(0)
        driver.setSteeringAngle(0)
        
    
    #if controllerArg == 1:
        #print(ds_values[4])
        #print (graph_coords[current_target])
        #print (road_coords[current_target])
        #print("angle:",angle)
        #print("speed:",speed)
        #print("balhatul:",ds_values[0])
        #print("bal:",ds_values[6])
        #print("jobb:",ds_values[7])
        #print("jobbhatul:",ds_values[5])
        #print(turn)
        #print(stop)
        #print (last_target)
        #main controller

        #print ("bear:",target_bearing)
        #print ("dist:",target_distance)
        #print(road_angle[current_target])

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
