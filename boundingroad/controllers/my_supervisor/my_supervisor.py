from controller import Supervisor
import sys
import pickle
import math

# create supervisor instance
robot = Supervisor()


# get all roads by their ids
road_defs = ['road0','road1','road2','road3','road4','road5','road6','road7','road8','road9','road10','road11','road12','road13','road14','road15','road16','road17','road18','road19','road20','road21','road22','road23','road24','road25','road26','road27','road28','road29','road30','road31','road32']
roads = []
road_ids = []
road_rotations = []
road_coords = []
road_width = []
road_length = []
road_angle = []
road_points = {}



for i in range(len(road_defs)):
    roads.append(robot.getFromDef(road_defs[i]))
for i in range(len(roads)):
    road_ids.append(roads[i].getField('id').getSFString())
for i in range(len(roads)):
    road_coords.append(roads[i].getField('translation').getSFVec3f())
for i in range(len(roads)):
    road_rotations.append(roads[i].getField('rotation').getSFRotation())
    
for i in range(len(roads)):
    try:
        road_angle.append(roads[i].getField('curvatureRadius').getSFFloat())
    except AttributeError:
        road_angle.append(0)

    
for i in range(len(roads)):
    try:
        road_width.append(roads[i].getField('roadsWidth').getSFFloat())
    except AttributeError:
        road_width.append(roads[i].getField('width').getSFFloat())   

        
for i in range(len(roads)):
    try:
        road_length.append(roads[i].getField('length').getSFFloat())
    except AttributeError:
        road_length.append(0)

for i in range(len(road_rotations)):
    road_rotations[i].pop(2)
    road_rotations[i].pop(1)
    road_rotations[i].pop(0)
for i in range(len(road_coords)):
    road_coords[i].pop(2)



start = []
end = []
connect = []
graph = {}

'''class AnyadException(Exception):
    "Raised when the input value is less than 18"
    pass

asd = []
try:
    asd.append(roads[2].getField('connectedRoadIDs').getMFString(3))
    if asd[0] == '':
        raise AnyadException
    else:
        print("fakju")
except AnyadException:
    print("lol")
print (type(asd[0]))
print ("asd:",asd)'''

for i in range(len(roads)):
    graph[i] = []
    connect.clear()
    try:   
        start = roads[i].getField('startJunction').getSFString()
        end = roads[i].getField('endJunction').getSFString()
    except AttributeError:
        for j in range(4):
            if roads[i].getField('connectedRoadIDs').getMFString(j) == '':
                break
            connect.append(roads[i].getField('connectedRoadIDs').getMFString(j))
            
    #print("road",i,"start:",start,"end:",end,"connect:",connect)
    if start:
        graph[i].append(start)
    if end:
        graph[i].append(end)
    if connect:
        for l in range(len(connect)):     
            graph[i].append(connect[l])
    start = []
    end = []




for i in range(len(roads)):
    road_points[i] = []
    if road_rotations[i][0] == round(math.pi/4,4):
        road_points[i] = road_coords[i]
    if road_length[i] != 0 and round(road_rotations[i][0],4) == round(math.pi/2,4):
        #print(road_coords[i][1])
        #print("coord:",road_coords[i][1],"length:",road_length[i] / 2)
        road_points[i].append(road_coords[i][0])
        road_points[i].append(road_coords[i][1] + road_length[i] / 2)
    if road_length[i] != 0 and road_rotations[i][0] == 0.0:
        #print("coord:",road_coords[i][0],"length:",road_length[i] / 2)
        road_points[i].append(road_coords[i][0] + road_length[i] / 2)
        road_points[i].append(road_coords[i][1])
    if road_angle[i] != 0:
        #print("asd",road_rotations[i])
        rot = float(road_rotations[i][0])
        #road_ids = list(map(int, road_ids))
        #print (rot)
        if round(rot,4) == round(math.pi/2,4):
            #print("masik")
            road_points[i].append(road_coords[i][0] - road_angle[i]*(math.pi/4))
            road_points[i].append(road_coords[i][1] + road_angle[i]/(math.pi/2))
        if round(rot,4) == 0:
            road_points[i].append(road_coords[i][0] + road_angle[i]/(math.pi/2))
            road_points[i].append(road_coords[i][1] + road_angle[i]/(math.pi/2))
        if round(rot,4) == round(math.pi/ -2,4):
            road_points[i].append(road_coords[i][0] + road_angle[i]/(math.pi/2))
            road_points[i].append(road_coords[i][1] - road_angle[i]*(math.pi/4))
        if math.fabs(round(rot,4)) == round(math.pi,4):
            road_points[i].append(road_coords[i][0] - road_angle[i]*(math.pi/4))
            road_points[i].append(road_coords[i][1] - road_angle[i]*(math.pi/4))


with open(r'..\myGraph.pickle', 'wb') as handle:
    pickle.dump(graph, handle, protocol=pickle.HIGHEST_PROTOCOL)

with open(r'..\roadIDs.pickle', 'wb') as handle:
    pickle.dump(road_ids, handle, protocol=pickle.HIGHEST_PROTOCOL)

with open(r'..\graphCoords.pickle', 'wb') as handle:
    pickle.dump(road_points, handle, protocol=pickle.HIGHEST_PROTOCOL)

with open(r'..\roadCoords.pickle', 'wb') as handle:
    pickle.dump(road_coords, handle, protocol=pickle.HIGHEST_PROTOCOL)

with open(r'..\roadAngles.pickle', 'wb') as handle:
    pickle.dump(road_angle, handle, protocol=pickle.HIGHEST_PROTOCOL)
	
with open(r'..\roadLengths.pickle', 'wb') as handle:
    pickle.dump(road_length, handle, protocol=pickle.HIGHEST_PROTOCOL)

	

'''vehicle_defs = ["vehicle","vehicle1","vehicle2","vehicle3"]
vehicles = []
vehicle_args = []

for i in range(len(vehicle_defs)):
    vehicles.append(robot.getFromDef(vehicle_defs[i]))
for i in range(len(vehicles)):
    vehicle_args.append(vehicles[i].getField('controllerArgs').getMFString(0))

with open(r'..\vehicleArgs.pickle', 'wb') as handle:
    pickle.dump(vehicle_args, handle, protocol=pickle.HIGHEST_PROTOCOL)'''

#print(vehicles)
#print(vehicle_args)

timestep = int(robot.getBasicTimeStep())
#print(graph)
#print("roads:",roads)
#print("rotate:",road_rotations)
#print("width:",road_width)
#print("length:",road_length)
#print("coords:",road_coords)
#print("points:",road_points)
#print("angle:",road_angle)

while robot.step(timestep) != -1:
    #print("asd")
    
    # Your Supervisor code here
    pass
