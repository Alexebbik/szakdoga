#a külső könyvtárak behúzása
from controller import Supervisor
import sys
import pickle
import math

#az supervisor irányítója
robot = Supervisor()


#az utak nevei a webots világból
road_defs = ['road0','road1','road2','road3','road4','road5','road6','road7','road8','road9','road10','road11','road12','road13','road14','road15','road16','road17','road18','road19','road20','road21','road22','road23','road24','road25','road26','road27','road28','road29','road30','road31','road32']
#a listák amikbe belerakom az utak különböző információit
roads = []
road_ids = []
road_rotations = []
road_coords = []
road_width = []
road_length = []
road_angle = []
road_points = {}


#az utak információinak lekérdezése a webots világból és elmentésük
#maga az út objektum
for i in range(len(road_defs)):
    roads.append(robot.getFromDef(road_defs[i]))
#az utak id-jai
for i in range(len(roads)):
    road_ids.append(roads[i].getField('id').getSFString())
#az utak elhelyezkedése a világban
for i in range(len(roads)):
    road_coords.append(roads[i].getField('translation').getSFVec3f())
#az utak forgási szögei
for i in range(len(roads)):
    road_rotations.append(roads[i].getField('rotation').getSFRotation())
#a kanyarok görbületei
for i in range(len(roads)):
    try:
        road_angle.append(roads[i].getField('curvatureRadius').getSFFloat())
    #ha nincs ilyen értéke, akkor nullát ír be
    except AttributeError:
        road_angle.append(0)
#az utak szélessége
for i in range(len(roads)):
    try:
        #másképp kell lekérni kereszteződésnél
        road_width.append(roads[i].getField('roadsWidth').getSFFloat())
    except AttributeError:
        #és másképp kanyarnál és egyenesnél
        road_width.append(roads[i].getField('width').getSFFloat())   
#az utak hossza      
for i in range(len(roads)):
    try:
        road_length.append(roads[i].getField('length').getSFFloat())
    except AttributeError:
        #kereszteződésnek nincs ilyen változója, nullát ír be
        road_length.append(0)
#az utak forgási szögei egy négy értéket tartalmazó listát adnak vissza
#amely a három tengelyt és magát a szöget tartalmazza
#nekem csak a szögre van szükségem, mert minden út Z-ben van elforgatva,
#ezért a másik három értéket eldobom
for i in range(len(road_rotations)):
    road_rotations[i].pop(2)
    road_rotations[i].pop(1)
    road_rotations[i].pop(0)
#a koordinátáknál szintén a Z értékre nincs szükségem, hiszen nincsenek lejtők és emelkedők
for i in range(len(road_coords)):
    road_coords[i].pop(2)

#a listáim és a könyvtáram amikre szükségem lesz
start = []
end = []
connect = []
graph = {}

#megint végigmegyek az utakon egy ciklussal
for i in range(len(roads)):
    #létrehozok a könyvtárban egy üres helyet
    graph[i] = []
    #kiürítem a csatlakozási pontok listáját
    connect.clear()
    #egyeneseknek és kanyaroknak ki lehet olvasni a kezdő és vég csatlakozási útjaikat
    try:   
        start = roads[i].getField('startJunction').getSFString()
        end = roads[i].getField('endJunction').getSFString()
    #kereszteződéseknek ezek egy listában vannak, ami tartalmazza az összes
    #hozzá csatlakozó út id-ját
    except AttributeError:
        for j in range(4):
            if roads[i].getField('connectedRoadIDs').getMFString(j) == '':
                break
            #ezeket belerakom a csatlakozási pontos listámba
            connect.append(roads[i].getField('connectedRoadIDs').getMFString(j))
    #ha volt kezdő pontom, belerakom a könyvtárban létrehozott helyére
    if start:
        graph[i].append(start)
    #ha volt vég pontom, belerakom a könyvtárban létrehozott helyére
    if end:
        graph[i].append(end)
    #ha volt csatlakozási pontom, belerakom a könyvtárban létrehozott helyére
    if connect:
        for l in range(len(connect)):     
            graph[i].append(connect[l])
    #kiürítem a listáimat
    start = []
    end = []

#megint végigmegyek az utakon egy ciklussal
for i in range(len(roads)):
    #létrehozok a könyvtárban egy üres helyet
    road_points[i] = []
    #ha az út egy kereszteződés
    if road_rotations[i][0] == round(math.pi/4,4):
        #a saját helyzete a világban, lesz a pont ahová majd az autónak mennie kell
        road_points[i] = road_coords[i]
    #ha egy elforgatott egyenes út
    if road_length[i] != 0 and round(road_rotations[i][0],4) == round(math.pi/2,4):
        #akkor az X koordinátája a saját X koordinátája lesz
        road_points[i].append(road_coords[i][0])
        #az Y pedig a saját Y-jától a hossza felével eltolt pont
        road_points[i].append(road_coords[i][1] + road_length[i] / 2)
    #ha egy nem elforgatott egyenes út
    if road_length[i] != 0 and road_rotations[i][0] == 0.0:
        #az X koordinátája saját X-étől a hossza felével eltolt pont
        road_points[i].append(road_coords[i][0] + road_length[i] / 2)
        #az Y koordinátája pedig a saját Y koordinátája lesz
        road_points[i].append(road_coords[i][1])
    #ha egy kanyar
    if road_angle[i] != 0:
        #lekérdezem a forgási szögét
        rot = float(road_rotations[i][0])
        #és attól függően, hogy hogyan áll a világban, eltolom
        #a koordinátájához képest a pontot, ahová majd az autónak menni kell
        if round(rot,4) == round(math.pi/2,4):
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

#az szükséges listákat kimentem egy-egy fájlba, amit majd a kontrollerben beolvasok
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

#a lépésköz létrehozása amivel a szimuláció menni fog
timestep = int(robot.getBasicTimeStep())
#a supervisor lépéseiben nem kell csinálnom semmit, mert
#nem használom semminek az irányítására,
#az mind a kontrolleremben van
while robot.step(timestep) != -1:
    pass
