#a külső könyvtárak behúzása
#az autó irányításához a Driver könyvtár
from vehicle import Driver
#a PI és a különböző matematikai műveletek (abszolút érték, kerekítés stb.)
import math
import sys
import random
#szenzorok, iránytű, GPS
from controller import DistanceSensor, Compass, GPS
from collections import deque
import pickle

#a supervisor által létrehozott pickle fájlok beolvasása
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

#id-k és graph pontok szövegből számmá alakítása
road_ids = list(map(int, road_ids))
for key in graph:
    graph[key] = list(map(int, graph[key]))
    

#az autót irányító driver létrehozása
driver = Driver()
#a lépésköz létrehozása amivel a szimuláció menni fog
timestep = int(driver.getBasicTimeStep())


#a legrövidebb utat megkereső függvényem
#átadom neki a gráfomat, majd a két pontot ami között keresnie kell az utat
def pathfinder(graph, start, end):
    #létrehoz egy sort amiben eltárolja a csúcsokat amiket meg kell látogatni
    queue = deque()
    #létrehoz egy könyvtárat amiben a meglátogatott csúcsokat és a szülőjüket tárolja
    #az első természetesen a kiinduló pont, aminek nincs szülője
    visited = {start: None}
    #ezt belerakja a sorba
    queue.append(start)
    #elindít egy while-t, ami addig fut, míg a sor üres nem lesz
    while queue:
        #kiveszi az első elemet a sorból, ez lesz a jelenlegi csúcs
        current_node = queue.popleft()
        #megnézi, hogy a kivett csúcs megegyezik-e a végponttal
        if current_node == end:
            #létrehozza az útvonal listát, amit majd a végén visszaad
            path = []
            #addig megy, míg vissza nem ér az eredeti kiinduló csúcshoz
            while current_node is not None:
                #hozzáadja azt az útvonalhoz
                path.append(current_node)
                #megjelöli már meglátogatottként
                current_node = visited[current_node]
            #megfordítja az útvonalat, hogy a megfelelő sorrendben legyenek (kezdő->vég)
            path.reverse()
            #és visszaadja
            return path
        #ha nem egyezik meg a végponttal, megvizsgálja a szomszédos csúcsokat a gráf alapján
        for neighbor in graph[current_node]:
            #hogyha ezek még nem lettek korábban meglátogatva
            if neighbor not in visited:
                #akkor arra megy tovább, a szülőjét beállítva az eddig nézett csúcsra
                visited[neighbor] = current_node
                #és belerakja a sorba
                queue.append(neighbor)
    #ha elér idáig, az azt jelenti, hogy nincs út a két csúcs között
    return None

#a távolságszenzorok nevei a webots világból
ds_names = ["dsRearLeft","dsLeft","dsFrontLeft","dsFrontRight","dsRight","dsRearRight","dsFront"]
ds_list = []

#a távolságszenzorok, az iránytű és a GPS beolvasása és engedélyezése
for i in range(len(ds_names)):
    ds_list.append(DistanceSensor(ds_names[i]))
    ds_list[i].enable(timestep)
compass = Compass("compass")
compass.enable(timestep)
gps = GPS("gps")
gps.enable(timestep)

#a célpont irányát visszaadó függvényem
def getTargetBearing():
    #először lekérem az autóba épített iránytűm értékeit
    #ez visszaad egy 3D vektort, amely a webots-os globális
    #koordinátarendszerben az autóm irányát jelképezi
    north = compass.getValues()
    #kiszámolja radiánban a szöget a változóm és az x tengely pozitív iránya között
    rad = math.atan2(north[1],north[0])
    #kiszámolja a szöget a célpont és az autó jelenlegi pozíciója között
    target_bearing = math.atan2(-target_y + current_y, target_x - current_x)
    #kiszámolja a szöget a célpont és az iránytű között
    target_bearing -= rad
    #ezt az értéket átszámolja fokokba
    target_bearing *= 180/math.pi
    #majd átalakítja, hogy előre legyen a 0 fok, jobbra a 90,
    #hátul a 180, balra a 270, majd ismét előre a 360
    if target_bearing < -90.0:
        target_bearing = target_bearing + 360
    target_bearing += 90
    if target_bearing >= 360.0:
        target_bearing = target_bearing - 360
    #és visszaadja a kiszámolt értéket
    return target_bearing

#az utak hosszának ellenőrzése a sebesség beállításához
nonzero_length = []
for i in range(len(road_lengths)):
    if road_lengths[i] != 0:
        nonzero_length.append(road_lengths[i])

#az autókat a controllerArgs változójukkal különböztetem meg a controllerben
controller = driver.getSelf()
controllerArg = controller.getField('controllerArgs').getMFString(0)
controllerArg = int(controllerArg)

#az összes változó létrehozása amiket használni fogok a controllerben
target_x = 0
target_y = 0
path = []
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
stop = 0
time_helper = 0
path_helper = 1
counter = 0
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

#itt kezdődik el maga a controller
while driver.step() != -1:
    
    #az autó GPS-ének beolvasásával kapom meg az autó x és y koordinátáit
    current_x, current_y, z = gps.getValues()
    #minden lépésben lekérem a jelenlegi időt
    current_time = driver.getTime()
    #és lekérem az irányát a célpontomnak
    target_bearing = getTargetBearing()
    
    
    #a fő autómnak út lekérése
    if controllerArg == 0:
        if not path and path_helper == 1:
            while end == start:
                end = random.randint(0,len(graph)-1)
            #új út kérése kezdő és végpont alapján
            path = pathfinder(graph, start, end)
            #út megfordítása, hogy egyesével ki tudjam venni belőle a pontokat
            path.reverse()
            #az utolsó pont kivétele
            end_target = path[0]
            #az utolsó pont koordinátái
            end_coords = graph_coords[end_target]
            #a jelenlegi pontom ami most a path-ban az utolsó
            current_target = path.pop()
            #ennek a pontnak a koordinátái
            target_coords = graph_coords[current_target]
            target_x = target_coords[0]
            target_y = target_coords[1]
            path_helper = 0
    
    if controllerArg == 1:
        if not path and path_helper == 1:
            while end == start:
                end = random.randint(0,len(graph)-1)
            if path_counter1 % 2 == target0:
                path = pathfinder(graph, path1_1, path1_2)
            elif path_counter1 % 2 == target1:
                path = pathfinder(graph, path1_2, path1_1)
            path.reverse()
            end_target = path[0]
            end_coords = graph_coords[end_target]
            current_target = path.pop()
            target_coords = graph_coords[current_target]
            target_x = target_coords[0]
            target_y = target_coords[1]
            path_helper = 0
            path_counter1 = path_counter1 + 1
            
    
    if controllerArg == 2:
        if not path and path_helper == 1:
            while end == start:
                end = random.randint(0,len(graph)-1)
            if path_counter2 % 3 == target0:
                path = pathfinder(graph, path2_1, path2_2)
            elif path_counter2 % 3 == target1:
                path = pathfinder(graph, path2_2, path2_3)
            elif path_counter2 % 3 == target2:
                path = pathfinder(graph, path2_3, path2_1)
            path.reverse()
            end_target = path[0]
            end_coords = graph_coords[end_target]
            current_target = path.pop()
            target_coords = graph_coords[current_target]
            target_x = target_coords[0]
            target_y = target_coords[1]
            path_helper = 0
            path_counter2 = path_counter2 + 1
    
    
    if controllerArg == 3:
        if not path and path_helper == 1:
            while end == start:
                end = random.randint(0,len(graph)-1)
            if path_counter3 % 3 == target0:
                path = pathfinder(graph, path3_1, path3_2)
            elif path_counter3 % 3 == target1:
                path = pathfinder(graph, path3_2, path3_3)
            elif path_counter3 % 3 == target2:
                path = pathfinder(graph, path3_3, path3_1)
            path.reverse()
            end_target = path[0]
            end_coords = graph_coords[end_target]
            current_target = path.pop()
            target_coords = graph_coords[current_target]
            target_x = target_coords[0]
            target_y = target_coords[1]
            path_helper = 0
            path_counter3 = path_counter3 + 1
    
    #a jelenlegi ponttól a távolság
    target_distance = math.fabs(current_x - target_x) + math.fabs(current_y - target_y)

    #a távolságszenzorok értékeinek a lekérése
    ds_values = []
    for i in range(len(ds_list)):
        ds_values.append(ds_list[i].getValue())
    
    #legelsőnek azt nézem meg, hogy az autó rajta van-e már a célponton
    #amikor beér az autó a cépont hatósugarába
    if target_distance < target_radius:
        #megnézzük ez a végpont-e
        if (math.fabs(current_x - end_coords[0]) + math.fabs(current_y - end_coords[1])) < target_radius:
            #ha igen, növelem a változót, amivel ezt nézem
            reached += 1
            if reached == 1:
                if controllerArg == 0:
                    path_helper = 0
            if controllerArg == 1 and (current_target == path1_2 or current_target == path1_1):
                path_helper = 1
            if controllerArg == 2 and (current_target == path2_1 or current_target == path2_2 or current_target == path2_3):
                path_helper = 1
            if controllerArg == 3 and (current_target == path3_1 or current_target == path3_2 or current_target == path3_3):
                path_helper = 1
            #átállítom a kezdőpontot a végpontra, így az előzőlet végpontnak
            #számító pont lesz a kezdő, így onnan mehet tovább az autó
            start = end
            #nullára állítom a sebességet
            speed = stop_speed
            angle = front
            stop = 1
        time_helper += 1
        if time_helper == 1:
            counter += 1
            stop_time = driver.getTime()
        #hogyha az autó elérte a jelenlegi célpontot és az még nem a végső célpont
        if current_time > stop_time and (math.fabs(current_x - end_coords[0]) + math.fabs(current_y - end_coords[1])) > target_radius:
            #elmentem a célpontot amit épp elhagytunk
            last_target = current_target
            #megkapja az autó a következő célpontot az útvonalból
            current_target = path.pop()
            target_coords = graph_coords[current_target]
            target_x = target_coords[0]
            target_y = target_coords[1]
            stop = 0
            time_helper = 0
    
    
    
    #hogyha nincs még az autó a célponton
    if stop == 0:
        #először a célpont irányát nézem meg,
        #ezekből mindig csak egy lehet igaz, ezért elif-eket használok
        #hogyha az autó mögött van jobbról
        #akkor tolatunk és maximális szögben fordulunk balra
        if target_bearing >= front_right_max and target_bearing < rear:
            angle = max_left_angle
            speed = reverse_speed
        #hogyha az autó mögött van balról
        #akkor tolatunk és maximális szögben fordulunk jobbra
        elif target_bearing >= rear and target_bearing <= front_left_min:
            angle = max_right_angle
            speed = reverse_speed    
        #hogyha az autó előtt van jobbról
        #akkor attól függ a sebesség és a fordulási szög is,
        #hogy milyen szögben van az autóhoz a célpont
        elif target_bearing > front_right_min  and target_bearing < front_right_max:
            speed = ((target_bearing - front_right_min) / (front_right_max - front_right_min)) * (min_speed - turn_speed) + turn_speed
            angle = ((target_bearing - front_right_min) / (front_right_max - front_right_min)) * (max_right_angle - min_right_angle) + min_right_angle   
        #hogyha az autó előtt van balról
        #akkor attól függ a sebesség és a fordulási szög is,
        #hogy milyen szögben van az autóhoz a célpont
        elif target_bearing < front_left_max  and target_bearing > front_left_min:
            speed = ((target_bearing - front_left_min) / (front_left_max - front_left_min)) * (turn_speed - min_speed) + min_speed
            angle = ((target_bearing - front_left_min) / (front_left_max - front_left_min)) * (min_left_angle - max_left_angle) + max_left_angle  
        #hogyha egyenesen van, akkor nem kell fordulnia az autónak
        elif target_bearing >= front_left_max or target_bearing <= front_right_min:
            angle = front
            
            
            
            
        #ezután megnézem, hogy milyen út az, ahová tart az autó
        #ha az út ahová tart az autó rendelkezik valamekkora szöggel
        #akkor az egy kanyar
        if road_angle[current_target] != 0:
        #hogyha kanyar, akkor kicsivel nagyobb kell legyen maga a célpont,
        #hogy az autó biztosan megtalálja és rá tudjon állni
            target_radius = big_target
            #hogyha még nem vagyunk rajta azon az útszakaszon
            if target_distance > road_angle[current_target]:
                #figyeli, hogy van e jobbról valami,
                #ha nincs, óvatosan jobbra tart
                if ds_values[4] == min_ds:
                    angle = min_right_angle
                #ha tudjuk, hogy jobbra van a célpont, akkor fordulhat nagyobb szöggel
                if target_bearing > front_right_max - (front_right_max - front_right_min) / 2 and target_bearing < front_right_max:
                    angle = max_right_angle
                    if ds_values[4] == min_ds:
                        angle = max_right_angle / 2
                #hogyha már nincs teljesen jobbra, akkor megint kevésbé fordul
                if target_bearing < front_right_min + (front_right_min + front_right_max) / 2 and target_bearing > front_right_min:
                    angle = min_right_angle
                    if ds_values[4] == min_ds:
                        angle = max_right_angle / 2
                    #hogyha elég távol van a célpont, akkor felgyorsíthat
                    if target_distance > min(nonzero_length):
                        speed = target_distance
                #ha kicsit balról van a célpont, akkor, mivel tudjuk, hogy kanyarba tart az autó
                #óvatosan tartva a jobb oldali falat, nem lesz probléma
                if target_bearing > front_left_min + (front_left_max - front_left_min) / 2 and target_bearing < front_left_max:
                    angle = min_right_angle
                    #hogyha elég távol van a célpont, akkor felgyorsíthat
                    if target_distance > min(nonzero_length):
                        speed = target_distance
                #ha nagyon balra van a célpont, forduljon balra
                if target_bearing > front_left_max - (front_left_min - front_left_max) / 2 and target_bearing > front_left_min:
                    angle = max_left_angle
            #hogyha már rajta vagyunk a kanyar útszakaszon, tarthat jobbra
            elif target_distance < road_angle[current_target]:
                if ds_values[4] == min_ds:
                    angle = max_right_angle / 2
                    
        #ha az útszakasz koordinátái megegyeznek a gráfban azonos
        #számon lévő koordinátákkal, akkor ez egy kereszteződés
        elif graph_coords[current_target] == road_coords[current_target]:
            #itt is szükség van a nagyobb célpont használatára
            target_radius = big_target
            #ha teljesen balra van a célpont, nagyon balra kanyarodik
            if target_bearing > front_left_min:
                angle = max_left_angle
            #ha már nincs annyira balra, kevésbé kanyarodik
            if target_bearing > front_left_min + (front_left_max - front_left_min) / 2:
                angle = min_left_angle / 2
                #hogyha elég távol van a célpont, akkor felgyorsíthat
                if target_distance > min(nonzero_length):
                    speed = target_distance
            #ha jobbra van a célpont, jobbra kanyarodik
            if target_bearing < front_right_max:
                angle = max_right_angle
            #figyeli, hogy van e jobbról valami,
            #ha nincs, óvatosan jobbra tart
            if ds_values[4] == min_ds:
                angle = min_right_angle
                    
        #ha nem rendelkezik semekkora szöggel, és nem egyeznek meg a gráf koordinátái az
        #útszakaszéval, akkor ez egy egyenes
        elif road_angle[current_target] == 0 and graph_coords[current_target] != road_coords[current_target]:
            #ebben az esetben elég a kisebb célpont is
            target_radius = small_target
            #ha még nem vagyunk benne a útszakaszban
            if target_distance > road_lengths[current_target] / 2:
                #ha nagyon jobbra van a célpont, nagyon jobbra kanyarodik
                if target_bearing > front_right_max - (front_right_max - front_right_min) / 2 and target_bearing < front_right_max:
                    angle = max_right_angle
                #ha kevésbé jobbra van a célpont, már nem annyira jobbra kanyarodik
                if target_bearing < front_right_min + (front_right_min + front_right_max) / 2 and target_bearing > front_right_min:
                    angle = min_right_angle
                    #ha az előző útszakasz egy kanyar volt, akkor valamennyire jobbra tart
                    if road_angle[last_target] != 0:
                        angle = max_right_angle / 2
                    #kúlönben teljesen jobbra kanyarodik
                    else:
                        angle = max_right_angle
                #ha nagyon balra van a célpont, balra kanyarodik
                if target_bearing < front_left_min + (front_left_max - front_left_min) / 2 and target_bearing > front_left_min:
                    angle = max_left_angle / 2
                #ha nincs teljesen balra, akkor már nem kanyarodik, mert már igazából
                #jó irányba néz, figyelembe véve, hogy a jobb sávban kell mennie
                if target_bearing > front_left_max - (front_left_max - front_left_min) / 2 and target_bearing < front_left_max:
                    angle = front
            #ha már rajta vagyunk az egyenes útszakaszon
            elif target_distance < road_lengths[current_target] / 2:
                #figyeli, hogy van e jobbról valami,
                #ha nincs, óvatosan jobbra tart
                if ds_values[4] == min_ds:
                    angle = min_right_angle
                #ha jobbra van a célpont, jobbra kanyarodik
                if target_bearing < front_right_max:
                    angle = max_right_angle / 2
                #hogyha elég távol van a célpont, akkor felgyorsíthat
                if target_distance > min(nonzero_length):
                    speed = target_distance

        #ezek után ellenőrzöm a távolságszenzorok értékeit
        #mivel ezek közül egyszerre több is érzékelhet, itt nem használok elif-et
        #ha a bal első szenzor érzékel, és nagyobb az értéke, mint a jobb oldalinak
        if ds_values[2] > min_ds and ds_values[2] > ds_values[3]:
            #jobbra fordulunk a kapott értékkel arányosan
            #ha kicsi a kapott érték, az azt jelenti, hogy még messze van az objektum
            #amit érzékel a szenzor, szóval még nem kell annyira fordulni
            angle = ((ds_values[2] - min_ds) / (max_ds - min_ds)) * (max_right_angle - min_right_angle) + min_right_angle
            #ha már elég közel van az érzékelt objektum, akkor nem arányosan fordul
            if ds_values[2] > max_ds / 2:
                angle = max_right_angle
                speed = turn_speed / 2
            #ha mögötte van a célpont, akkor tolat és teljesen fordul
            if target_bearing >= front_right_max and target_bearing <= front_left_min:
                speed = reverse_speed
                angle = max_right_angle
        #ha a jobb első szenzor érzékel, és nagyobb az értéke, mint a bal oldalinak
        if ds_values[3] > min_ds and ds_values[3] > ds_values[2]:
            #balra fordulunk a kapott értékkel arányosan
            angle = ((ds_values[3] - min_ds) / (max_ds - min_ds)) * (max_left_angle + min_left_angle) - min_left_angle
            #ha már elég közel van az érzékelt objektum, akkor nem arányosan fordul
            if ds_values[3] > max_ds / 2:
                angle = max_left_angle
                speed = turn_speed / 2
            #ha mögötte van a célpont, akkor tolat és teljesen fordul
            if target_bearing >= front_right_max and target_bearing <= front_left_min:
                speed = reverse_speed
                angle = max_left_angle
        #ha a bal oldali szenzor érzékel
        if ds_values[1] > min_ds:
            #jobbra fordulunk a kapott értékkel arányosan
            angle = ((ds_values[1] - min_ds) / (max_ds - min_ds)) * (max_right_angle - min_right_angle) + min_right_angle
            #ha mögötte van a célpont, akkor tolat és teljesen fordul
            if target_bearing >= front_right_max and target_bearing <= front_left_min:
                speed = reverse_speed
                angle = max_right_angle
        #ha a jobb oldali szenzor érzékel
        if ds_values[4] > min_ds:
            #balra fordulunk a kapott értékkel arányosan
            angle = ((ds_values[4] - min_ds) / (max_ds - min_ds)) * (max_left_angle + min_left_angle) - min_left_angle
            #ha mögötte van a célpont, akkor tolat és teljesen fordul
            if target_bearing >= front_right_max and target_bearing <= front_left_min:
                speed = reverse_speed
                angle = max_left_angle
        #ha a bal hátsó szenzor érzékel
        if ds_values[0] > max_ds / 2:
            #teljesen jobbra fordul
            angle = max_right_angle
            speed = turn_speed
        #ha a jobb hátsó szenzor érzékel
        #itt használtam elifet, hogyha valahogy ugyanakkora értéket adnának vissza
        #a két hátsó szenzorja, akkor el tudja dönteni merre forduljon
        elif ds_values[5] > max_ds / 2:
            #teljesen balra fordul
            angle = max_left_angle
            speed = turn_speed
        
        #ezután jön a kerülési manőver
        #ha az elől lévő szenzor érzékel
        if ds_values[6] > min_ds:
            #és a célpontom bal elől van
            #ami azt jelenti, hogy épp a jobb sávban haladok, vagy fordulok
            if target_bearing > front_left_max - (front_left_max - front_left_min) / 2 or target_bearing < front_right_min + (front_right_min + front_right_max) / 2:
                #akkor kerül, tehát balra fordul közepesen
                angle = max_left_angle / 2
                #minimális sebességel, hogy ne menjen neki az objektumnak
                speed = min_speed
                #hogyha elég közel van az objektum, akkor teljesen fordul
                if ds_values[6] > max_ds / 2:
                    angle = max_left_angle
            #ha beérzékel a bal első szenzor is, az azt jelenti, hogy a bal oldali falnál vagyunk,
            #tehát vissza kell majd állni a jobb sávba, ezért jobbra fordulunk
            if ds_values[2] > min_ds:
                angle = max_right_angle
                speed = min_speed
        
        #és végül a kereszteződés elsőbbségmegadás
        #hogyha a legutóbbi célpont egy kereszteződés volt
        if graph_coords[last_target] == road_coords[last_target]:
            #és még nem vagyunk rajta az egyenes útszakaszon
            if target_distance > road_lengths[current_target] / 2:
                #és jobbról van a target
                if target_bearing > front_right_min and target_bearing < front_right_max:
                    #hogyha a jobb első szenzor érzékel
                    if ds_values[3] > min_ds:
                        #akkor megáll
                        speed = stop_speed
                #hogyha bal elől van a célpont
                if target_bearing > front_left_max - (front_left_max - front_left_min) / 2 and target_bearing < front_left_max:
                    #hogyha a jobb első szenzor nagyon érzékel
                    if ds_values[3] > max_ds / 2:
                        #nagyon lelassít
                        speed = min_speed / 2
                    #hogyha a bal oldali szenzor érzékel
                    if ds_values[1] > min_ds:
                        #nagyon lelassít
                        speed = min_speed / 2
                    #hogyha a bal elől lévő szenzor érzékel
                    if ds_values[2] > min_ds:
                        #akkor megáll
                        speed = stop_speed
                #hogyha nagyon balra van a célpont
                if target_bearing < front_left_min + (front_left_max - front_left_min) / 2 and target_bearing > front_left_min:
                    #és az első szenzor nagyon beérzékel, vagy a jobb első
                    if ds_values[6] > max_ds / 2 or ds_values[3] > min_ds:
                        #akkor megáll
                        speed = stop_speed
                    #ha a bal oldali érzékel
                    if ds_values[2] > min_ds:
                        #akkor is megáll
                        speed = stop_speed


        #kizárom a lehetőségét, hogy a fordulási szög, vagy a sebesség
        #nagyobb legyen a megengedettnél
        if angle > max_right_angle:
            angle = max_right_angle
        if angle < max_left_angle:
            angle = max_left_angle
        if speed > max_speed:
            speed = max_speed
        
        
        #átadom a kiszámolt fordulási szög és sebesség értékeket az autónak
        driver.setSteeringAngle(angle)
        driver.setCruisingSpeed(speed)
        
    else:
        driver.setCruisingSpeed(stop_speed)
        driver.setSteeringAngle(front)
    
    #lezárom a kontrollert
    pass







