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
road_ids = list(map(int, road_ids))
for key in graph:
    graph[key] = list(map(int, graph[key]))

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
ds_list = ['','','','','','']

for i in range(6):
    ds_list[i] = DistanceSensor(ds_names[i])
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

'''def getNextTarget():
    current_target = path.pop()
    print("The current target is point:",current_target)
    target_coords = graph_coords[current_target]
    target_x = target_coords[0]
    target_y = target_coords[1]
    print("Its coordinates are x:",target_x,"y:",target_y)
    print("Going on")'''

start = 6
end = 6
target_radius = 0.25


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
turn = 1
time_helper = 0
path_helper = 1
counter = 0
angle = 0
speed = 10
end_coords = []
#print (road_ids)
print (graph)
while driver.step() != -1:
    

    current_x, current_y, z = gps.getValues()
    current_time = driver.getTime()
    target_bearing = getTargetBearing()
    # assume we have a variable `time` that represents the webots world time in seconds
    hours, remainder = divmod(current_time, 3600)
    minutes, seconds = divmod(remainder, 60)

    # format the time string
    time_string = f"{math.floor(hours):02}:{math.floor(minutes):02}:{math.floor(seconds):02}"
    
    
    if current_time < 0.06:
        print("Lets start with getting a path!")
    
    if not path and path_helper == 1:
        #end = random.randint(0,11)
        while end == start:
            end = random.randint(0,16)
        #print("start:",start)
        #print("end:",end)
        path = pathfinder(graph, start, end)
        print("The new path is:",path)
        '''if path == None:
            path = pathfinder(graph, start, end)
            print("ANYAD")'''
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
    
    ds_values = ['','','','','','']
    for i in range(6):
        ds_values[i] = ds_list[i].getValue()
        #print(ds_values[i])
    
    #print(ds_values[2])
    #print(ds_values[5])
    
    
    #print("vege:",math.fabs(current_x - end_coords[0]) + math.fabs(current_y - end_coords[1]))
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
        if current_time > stop_time + 3:
            current_target = path.pop()
            print("The current target is point:",current_target)
            target_coords = graph_coords[current_target]
            target_x = target_coords[0]
            target_y = target_coords[1]
            print("Its coordinates are x:",target_x,"y:",target_y)
            print("Going on")
            #print (path)
            stop = 0
            time_helper = 0
            turn = 1
    
    if stop == 0:
        if turn == 1:
            if target_bearing >= 90 and target_bearing < 180:
                angle = -0.75
                speed = -2
                #print("jobbhat")
            if target_bearing >= 180 and target_bearing <= 270:
                angle = 0.75
                speed = -2
                #print("balhat")
            if target_bearing > 0.01 and target_bearing < 90:
                speed = ((target_bearing - 0.01) / (90 - 0.01)) * (1 - 10) + 10
                angle = ((target_bearing - 0.01) / (90 - 0.01)) * (0.75 - 0.1) + 0.1
                #print("jobb")
            if target_bearing < 359.9  and target_bearing > 270:
                speed = ((target_bearing - 270) / (359.9 - 270)) * (10 - 1) + 1
                angle = ((target_bearing - 270) / (359.9 - 270)) * (-0.1 + 0.75) -0.75
                #print("bal")
            if target_bearing >=359.9 and target_bearing <=0.01:
                turn = 0
                angle = 0
                #print("egyenes")
                            
        if target_distance <= 7.5 * target_radius:
            speed = target_distance
            if target_bearing >= 90 and target_bearing <= 270:
                speed *= -1
            #print("kozel")
        if ds_values[2] > 0 and ds_values[2] > ds_values[3]:
            angle = ((ds_values[2] - 0) / (1 - 0)) * (0.75 - 0.1) + 0.1
            if target_bearing >= 90 and target_bearing <= 270:
                speed = -2
                angle = 0.75
            #print("balelol")
        if ds_values[3] > 0 and ds_values[3] > ds_values[2]:
            angle = ((ds_values[3] - 0) / (1 - 0)) * (-0.75 + 0.1) - 0.1
            if target_bearing >= 90 and target_bearing <= 270:
                speed = -2
                angle = -0.75
            #print("jobbelol")
        if ds_values[1] > 0 and ds_values[1] > ds_values[4]:
            angle = ((ds_values[1] - 0) / (1 - 0)) * (0.75 - 0.1) + 0.1
            if target_bearing >= 90 and target_bearing <= 270:
                speed = -2
                angle = 0.75
            #print("bal")
        if ds_values[4] > 0 and ds_values[4] > ds_values[1]:
            angle = ((ds_values[4] - 0) / (1 - 0)) * (-0.75 + 0.1) - 0.1
            if target_bearing >= 90 and target_bearing <= 270:
                speed = -2
                angle = -0.75
            #print("jobb")
        if ds_values[0] > 0.5 and ds_values[0] > ds_values[5]:
            angle = ds_values[0] * 1
            speed = ds_values[0] * 10
            #print("balhatul")
        if ds_values[5] > 0.5 and ds_values[5] > ds_values[0]:
            angle = ds_values[5] * -1
            speed = ds_values[5] * 10
            #print("jobbhatul")
        if angle > 0.75:
            angle = 0.75
        if angle < -0.75:
            angle = -0.75
        driver.setSteeringAngle(angle)
        driver.setCruisingSpeed(speed)
        
    else:
        driver.setCruisingSpeed(0)
        driver.setSteeringAngle(0)
        
    #print("angle:",angle)
    #print("speed:",speed)
    #print("balhatul:",ds_values[0])
    #print("jobbhatul:",ds_values[5])
    #print(turn)
    #print(stop)
    #main controller
    
    #print ("bear:",target_bearing)
    #print ("dist:",target_distance)
    
    #print ("speed:",driver.getTargetCruisingSpeed())
    #print ("angle:",driver.getSteeringAngle())
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
