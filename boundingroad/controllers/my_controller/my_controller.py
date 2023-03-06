"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import driver, Motor, DistanceSensor
from vehicle import Driver
import math
import sys
import random
from controller import DistanceSensor, Compass, GPS, Compass
from collections import deque
# create the driver instance.
driver = Driver()

# get the time step of the current world.
timestep = int(driver.getBasicTimeStep())

# Example graph represented as a dictionary where each vertex is a key and its value is a list of adjacent vertices
graph = {0: [1, 2], 1: [0, 3, 5, 7], 2: [0, 4, 6, 8], 3: [1, 10], 4: [2, 11], 5: [0], 6: [2], 7: [1], 8: [2], 9: [10, 11], 10: [3, 9], 11: [9, 4]}
graph_coords = {0: [13, 8], 1: [8, 8], 2: [18, 8], 3: [8, 13], 4: [18, 13], 5: [3, 8], 6: [23, 8], 7: [8, 3], 8: [18, 3], 9: [13, 8], 10: [8, 18], 11: [18, 18]}

def bfs_shortest_path(graph, start, end):
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

dsL = DistanceSensor("dsLeft")
dsL.enable(timestep)
dsRR = DistanceSensor("dsRearRight")
dsRR.enable(timestep)
dsRL = DistanceSensor("dsRearLeft")
dsRL.enable(timestep)
dsR = DistanceSensor("dsRight")
dsR.enable(timestep)
dsFL = DistanceSensor("dsFrontLeft")
dsFL.enable(timestep)
dsFR = DistanceSensor("dsFrontRight")
dsFR.enable(timestep)

compass = Compass("compass")
compass.enable(timestep)
gps = GPS("gps")
gps.enable(timestep)



target_x = 8
target_y = 8

start = 0
end = 0
start_coord = []
end_coord = []
target_radius = 0.25


def set_target():
    
    return None

# You should insert a getDevice-like function in order to get the
# instance of a device of the driver. Something like:
#  motor = driver.getDevice('motorname')
#  ds = driver.getDevice('dsname')
#  ds.enable(timestep)

#maximalis jobb driver.setSteeringAngle(0.5)
#maximalis bal driver.setSteeringAngle(-0.5)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
stop = 0
turn = 1
helper = 0
counter = 0
while driver.step() != -1:
    current_x, current_y, z = gps.getValues()
    
    
    driver.setCruisingSpeed(10)
    driver.setSteeringAngle(0)
    
    # read current GPS values
    
    target_distance = math.fabs(current_x - target_x) + math.fabs(current_y - target_y)
    #print(target_distance)
    # Get the values from the compass
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
    #print(target_bearing)
    #a target iranya 0 elore picit jobbra, 90 jobbra
    #180 mogotte, 270 balra, 360 elore picit balra

    current_time = driver.getTime()

    #print current GPS position
    #print('Current position: ', current_x, current_y)
    
    dsRLValue = dsRL.getValue()
    dsLValue = dsL.getValue()
    dsFLValue = dsFL.getValue()
    dsFRValue = dsFR.getValue()
    dsRValue = dsR.getValue()
    dsRRValue = dsRR.getValue()
    
    path = bfs_shortest_path(graph, start, end)
    
    if current_time < 0.064:
        print("Your starting point is:",current_x,"x",current_y,"y")
    set_target()
    
    if target_distance < target_radius:
        #print ("MEGVAN")
        driver.setCruisingSpeed(0)
        driver.setSteeringAngle(0)
        stop = 1
        helper += 1
        #print(helper)
        if helper == 1:
            counter += 1
            stop_time = driver.getTime()
            #print(stop_time)
            print("landolas")
        if current_time > stop_time + 3:
            print("megyunk")
            set_target()
            stop = 0
            helper = 0
            turn = 1

    if stop == 0:
        if turn == 1:
            if target_bearing >= 90 and target_bearing < 180:
                driver.setSteeringAngle(-0.5)
                driver.setCruisingSpeed(-1)
                #print("jobb4")
            if target_bearing >= 180 and target_bearing <= 270:
                driver.setSteeringAngle(0.5)
                driver.setCruisingSpeed(-1)
                #print("bal4")
            if target_bearing > 1 and target_bearing <= 30:
                driver.setSteeringAngle(0.2)
                driver.setCruisingSpeed(7.5)
                #print("jobb2")
            if target_bearing < 90 and target_bearing > 30:
                driver.setSteeringAngle(0.4)
                driver.setCruisingSpeed(5)
                #print("jobb3")
            if target_bearing > 270 and target_bearing < 330:
                driver.setSteeringAngle(-0.4)
                driver.setCruisingSpeed(5)
                #print("bal3")
            if target_bearing >= 330 and target_bearing < 359:
                driver.setSteeringAngle(-0.2)
                driver.setCruisingSpeed(7.5)
                #print("bal2")
            if target_bearing >= 359:
                driver.setSteeringAngle(-0.1)
                driver.setCruisingSpeed(10)
                #print("bal1")
            if target_bearing <= 1:
                driver.setSteeringAngle(0.1)
                driver.setCruisingSpeed(10)
                #print("jobb1")
            if target_bearing >=359.9 and target_bearing <=0.01:
                turn = 0
                #print("egyenes")
        if target_distance < 2 * target_radius:
            driver.setCruisingSpeed(0.5)
            #print("nagyon")
        if target_distance <= 10 * target_radius and target_distance > 2 * target_radius:
            driver.setCruisingSpeed(1)
            #print("kozel")
        if dsLValue > 0.75:
            #print("nagyon bal")
            driver.setSteeringAngle(0.5)
        if dsRValue > 0.75:
            #print("nagyon jobb")
            driver.setSteeringAngle(-0.5)
        if dsLValue > 0.25 and dsLValue <= 0.75:
            #print("bal")
            driver.setSteeringAngle(0.3)
        if dsRValue > 0.25 and dsRValue <= 0.75:
            #print("jobb")
            driver.setSteeringAngle(-0.3)
        if dsFLValue <= 0.1 and dsFLValue > 0:
            #print("megjobalelol")
            driver.setSteeringAngle(0.1)
        if dsRLValue <= 0.1 and dsFLValue > 0:
            #print("megjobalhatul")
            driver.setSteeringAngle(0.1)
        if dsFRValue <= 0.1 and dsFRValue > 0:
            #print("megjojobbelol")
            driver.setSteeringAngle(-0.1)
        if dsRRValue <= 0.1 and dsRRValue > 0:
            #print("megjojobbhatul")
            driver.setSteeringAngle(-0.1)
        if dsFLValue > 0.1 and dsFLValue <=0.4:
            #print("balelol1")
            driver.setSteeringAngle(0.25)
        '''if dsRLValue > 0.1 and dsRLValue <=0.4:
            #print("balhatul1")
            driver.setSteeringAngle(0.25)'''
        '''if dsRRValue > 0.1 and dsRRValue <=0.4:
            #print("jobbhatul1")
            driver.setSteeringAngle(-0.25)'''
        if dsFRValue > 0.1 and dsFRValue <=0.4:
            #print("jobbelol1")
            driver.setSteeringAngle(-0.25)
        if dsFLValue > 0.4:
            #print("balelol2")
            driver.setSteeringAngle(0.5)
        '''if dsRLValue > 0.4:
            #print("balhatul12")
            driver.setSteeringAngle(0.5)'''
        '''if dsRRValue > 0.4:
            #print("jobbhatul2")
            driver.setSteeringAngle(-0.5)'''
        if dsFRValue > 0.4:
            #print("jobbelol2")
            driver.setSteeringAngle(-0.5)
        if dsRRValue > 0.75 and dsFRValue == 0 and dsFLValue == 0 and dsLValue == 0 and dsRValue == 0:
            driver.setCruisingSpeed(1)
            driver.setSteeringAngle(-0.5)
        if dsRLValue > 0.75 and dsFRValue == 0 and dsFLValue == 0 and dsLValue == 0 and dsRValue == 0:
            driver.setCruisingSpeed(1)
            driver.setSteeringAngle(0.5)
    else:
        driver.setCruisingSpeed(0)
        driver.setSteeringAngle(0)
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
