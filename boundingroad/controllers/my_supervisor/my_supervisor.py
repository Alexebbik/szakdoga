from controller import Supervisor
import sys
import pickle

# create supervisor instance
robot = Supervisor()


# get all roads by their ids
road_defs = ['road0','road1','road2','road3','road4','road5','road6','road7','road8','road9','road10','road11','road12','road13','road14','road15','road16']
roads = ['','','','','','','','','','','','','','','','','']
road_ids = ['','','','','','','','','','','','','','','','','']
road_rotations = ['','','','','','','','','','','','','','','','','']
road_coords = ['','','','','','','','','','','','','','','','','']
road_width = ['','','','','','','','','','','','','','','','','']
road_length = ['','','','','','','','','','','','','','','','','']
road_points = {}


for i in range(17):
    roads[i] = robot.getFromDef(road_defs[i])
for i in range(17):
    road_ids[i] = roads[i].getField('id').getSFString()
for i in range(17):
    road_coords[i] = roads[i].getField('translation').getSFVec3f()
for i in range(17):
    road_rotations[i] = roads[i].getField('rotation').getSFRotation()

	
for i in range(17):
    try:
        road_width[i] = roads[i].getField('roadsWidth').getSFFloat()
    except AttributeError:
        road_width[i] = roads[i].getField('width').getSFFloat()	

		
for i in range(17):
    try:
        road_length[i] = roads[i].getField('length').getSFFloat()
    except AttributeError:
        road_length[i] = 0

for i in range(17):
    road_rotations[i].pop(2)
    road_rotations[i].pop(1)
    road_rotations[i].pop(0)
for i in range(17):
    road_coords[i].pop(2)
#print (road_ids)



start = []
end = []
connect = []
graph = {}

for i in range(17):
    graph[i] = []
    connect.clear()
    try:
        if roads[i].getField('connectedRoadIDs').getMFString(0) != "":
            #print("asd")
            for j in range(4):
                connect.append(roads[i].getField('connectedRoadIDs').getMFString(j))
        start = roads[i].getField('startJunction').getSFString()
        end = roads[i].getField('endJunction').getSFString()
    except IndexError:
        for k in range(3):
            connect = roads[i].getField('connectedRoadIDs').getMFString(k)
    except AttributeError:
        start = []
        end = []
    #print("road",i,"start:",start,"end:",end,"connect:",connect)
    if start:
        graph[i].append(start)
    if end:
        graph[i].append(end)
    if connect:
        for l in range(len(connect)):
            if connect[l] == '':
                continue            
            graph[i].append(connect[l])
#print(graph)


for i in range(17):
    road_points[i] = []
    if road_rotations[i][0] == 0.7853996938995746:
        road_points[i] = road_coords[i]
    if road_length[i] == 7.8 and road_rotations[i][0] == 1.5707996938995747:
        #print(road_coords[i][1])
        road_points[i].append(road_coords[i][0])
        road_points[i].append(road_coords[i][1] + road_length[i] / 2)
    if road_length[i] == 7.8 and road_rotations[i][0] == 0.0:
        road_points[i].append(road_coords[i][0] + road_length[i] / 2)
        road_points[i].append(road_coords[i][1])
    if i == 9:
        road_points[i] = [11.5, 29.5]
    if i == 13:
        road_points[i] = [20.5, 29.5]
    if i == 14:
        road_points[i] = [38, 9]
    if i == 15:
        road_points[i] = [39, -7]
    if i == 16:
        road_points[i] = [23, -6]


with open(r'..\myGraph.pickle', 'wb') as handle:
    pickle.dump(graph, handle, protocol=pickle.HIGHEST_PROTOCOL)

with open(r'..\roadIDs.pickle', 'wb') as handle:
    pickle.dump(road_ids, handle, protocol=pickle.HIGHEST_PROTOCOL)

with open(r'..\graphCoords.pickle', 'wb') as handle:
    pickle.dump(road_points, handle, protocol=pickle.HIGHEST_PROTOCOL)

timestep = int(robot.getBasicTimeStep())
print("rotate:",road_rotations)
print("width:",road_width)
print("length:",road_length)
print("coords:",road_coords)
print("points:",road_points)
while robot.step(timestep) != -1:
    #print("asd")
    
    # Your Supervisor code here
    pass
