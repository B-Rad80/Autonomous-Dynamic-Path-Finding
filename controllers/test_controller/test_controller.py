from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d # Uncomment if you want to use something else for finding the configuration space
from queue import PriorityQueue
import Algorithms as algo
from RRT import rrt

MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12

LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 5.5 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)
# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

def loadmap(file):
         with open(file, 'rb') as f:
                   return np.load(f)

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.09, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts=[]

for i in range(N_PARTS):
    robot_parts.append(robot.getDevice(part_names[i]))
    robot_parts[i].setPosition(float(target_pos[i]))
    robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

display = robot.getDevice("display")

# Odometry
pose_x     = 2.58
pose_y     = 8.9
pose_theta = 0

vL = 0
vR = 0


mode = 'autonomous'
# mode = 'autonomous'

lidar_sensor_readings = []
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # remove blocked sensor rays

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

#conversion functions
def convertM(x,y):
    return (round(y*(30)),round(x*(30)))
def convertW(x,y):
    return(y*(1/30), x*(1/30))

if mode == 'planner':

    map = loadmap("test.npy")
    box_map = algo.gen_box_map(map, 11)
    plt.imshow(box_map)
    plt.show()

    #starting world coordinates of the bot
    s_x = 1
    s_y = .815
    #converting the coordinates to map coordinates
    s_x1, s_y1 = convertM(s_x, s_y)
    #ending coordinates for the bot
    f_x = 8.62
    f_y = 6.76
    #converting the coordinates to map coordinates
    f_x1, f_y1 = convertM(f_x, f_y)

    #running the two sets of coordinates on the path planner using the configuration space map    
    best_path, visited = algo.path_planner(box_map, (s_y1 ,s_x1), (f_y1,f_x1))
    locs = []
    #going through each point in the path and adding it to the list 
    for n in reversed(best_path):
        locs.append(n.location)
    # print("best path found in ", len(locs), " steps")
    
    for x,y in visited:
        box_map[x][y] = .2
    for x,y in locs:
        box_map[x][y] = .7
    plt.imshow(box_map)
    #making a file with all the path coordinates
    with open('path.npy', 'wb') as f:
        np.save(f, locs)
    #making a file with the final path
    with open('final.npy', 'wb') as f:
        np.save(f, box_map)
    plt.show()


def printMap(map):
    mapa = map.copy()
    for i in range(len(mapa)):
        for k in range(len(mapa)):
            if mapa[i][k] >= 1:
                mapa[i][k] = 1
        
    plt.imshow(mapa)
    plt.show()

    if 1 in map:
        print("failure")
# Initialize your map data structure here as a 2D floating point array
if mode == 'manual':
    map = np.zeros((500,500), dtype=float)

if mode == 'autonomous':
    def loadmap(file):
         with open(file, 'rb') as f:
                   return np.load(f)
    #opening the visualization of the path and the path
    map = loadmap("box_map.npy")
    points = loadmap('path.npy')

    #convert world states to map states
    def convertM(x,y):
        return (round(y*(30)),round(x*(30)))
    #converting map states to world states
    def convertW(x,y):
        return(y*(1/30), x*(1/30))
    
    locs = []
    #going through each point in the path and converting points to world states and adding to locs list
    for i in points:
        locs.append(convertW(i[1],i[0]))
state = 20 # use this to iterate through your path

def printPath(coords, map):
    first = []
    second = []
    for x,y in coords:
        map[int(x)][int(y)] = .5
    printMap(map)
def check_neighbors_exist(possible_neighbors, map, visited):
       

    map_size = len(map)
    neighbors = []
    for x, y in possible_neighbors:
        if(x < map_size and x > 0 and y < map_size and y > 0 and not (x,y) in visited):
            neighbors.append((x,y))
    return neighbors
def findNewStart(map, tup, visited = []):
    x,y = tup
    neighbors = [(x-1, y), (x+1, y), (x,y+1), (x,y-1)]
    n = check_neighbors_exist(neighbors, map, visited)
    visited.append((x,y))
    if(map[x][y] < 1):
        return (x,y)
    elif(n == []):
        return "Found Nothing"
    else:
        for i in n:
            return findNewStart(map, i, visited)
def checkColl(loc, map, r = round((AXLE_LENGTH/2) * 30) -1):
    x,y = loc
    cool = [(x+r,y),(x,y+r),(x+r,y+r), (x+r,y-r),(x-r,y+r), (x-r,y),(x,y-r),(x-r,y-r)]
    n = check_neighbors_exist(cool, map, [])
    for x,y in n:
        if(map[x][y] >= 1):
            return True
    return False
while robot.step(timestep) != -1 and mode != 'planner':

# Sensing
    # Ground truth pose
    pose_y = gps.getValues()[2]
    pose_x = gps.getValues()[0]
    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.61988)
    pose_theta = rad

    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]

    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            rho = LIDAR_SENSOR_MAX_RANGE

        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho

        wx =  math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x
        wy =  -(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y


        if rho < 0.5*LIDAR_SENSOR_MAX_RANGE:
            
            m_x = int(wx*30-1) -1 
            m_y = int(wy*30-1) -1 \

            mpose_x = int(pose_x*30-1) -1 
            mpose_y = int(pose_y*30-1) -1
           
            if(map[m_x][m_y] <= 1):
            
               map[m_x][m_y] += .005
           
            g = map[m_x][m_y]
            if(g>.75):
            
                color = int(g*256**2 + g**256 + g)*255
                
                if(color > int(0xFFFFFF)):
                
                    color = 0xFFFFFF
                
                display.setColor(color)
                display.drawPixel(450- int(wy*30),int(wx*30))

    display.setColor(int(0xFF0000))
    display.drawPixel(450-int(pose_y*30),int(pose_x*30))

# Controller

    if mode == 'manual':
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT :
            vL = -MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.RIGHT:
            vL = MAX_SPEED
            vR = -MAX_SPEED
        elif key == keyboard.UP:
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED
        elif key == ord(' '):
            vL = 0
            vR = 0
        elif key == ord('S'):
            with open('test.npy', 'wb') as f:
                np.save(f, map)        

            print("Map file saved")
        elif key == ord('L'):
            # You will not use this portion but here's an example for loading saved a numpy array
            map = np.load("map.npy")
            print("Map loaded")
        else: # slow down
            vL *= 0.75
            vR *= 0.75
    else: # not manual mode
        
       #calculating the bearing and positional errors for the bot
        errorp = ((pose_x-locs[state][0])**2+(pose_y-locs[state][1])**2)**(1/2)
        errorb = math.atan2((locs[state][1]-pose_y),(locs[state][0]-pose_x)) + pose_theta
        #STEP 2: Controller
        if((errorp < .2)):
                #increment to the next waypoint
                state = state + 10
               
            
                #if the waypoint number is equivalant to the amount of waypoints
                if(state >= len(locs)):
                    #set motor velocities at zero
                    robot_parts[MOTOR_LEFT].setVelocity(0)
                    robot_parts[MOTOR_RIGHT].setVelocity(0)
                    #end the loop
                    print("Final State", pose_x, pose_y)
                    break
                elif(state + 20 <= len(locs)):

                    #continue on
                    i = state
                    new_path_flag = False
                    while (i < state + 20 ) and not new_path_flag:
                        next_pos = convertM(locs[i][1],locs[i][0])
                    
                        next_x = int(next_pos[0])
                        next_y = int(next_pos[1])
                        i +=1
                        
                        if(map[next_x][next_y] >= 1):
                            print("recalculate path ",map[next_x][next_y] ,convertW(next_y ,next_x))

                            if(i + 50  < len(locs)):
                                next_pos = convertM(locs[i+50][1],locs[i+50][0])
                
                                
                            else:
                                next_pos = convertM(locs[-1][1],locs[-1][0])
                            
                            next_x = int(next_pos[0])
                            next_y = int(next_pos[1])
                            maps = map.copy()
                            map1 = algo.gen_box_map(map, 30)
                            map = algo.gen_box_map(maps, 11)
                            

                            # print("Before RRT:", '\n')
                            
                            if(map1[convertM(pose_y, pose_x)[0]][convertM(pose_y, pose_x)[1]] >= 1):
                                newStart = findNewStart(map1, convertM(pose_y, pose_x))
                            else:
                                newStart = convertM(pose_y, pose_x)
                            print(newStart)
                            goalloc = (next_x,next_y)
                            while(map1[goalloc[0]][goalloc[1]] >= 1):
                                goalloc = convertM(locs[i+50][1],locs[i+50][0])
                                i+=50
                            new_path = rrt(newStart, goalloc, 2000, 5, map1)
                            print(new_path)
                            print("start and end pos", newStart,  goalloc)

                            # print("After RRT:")
                            
                            A = False
                            if (not new_path):
                                print("Fall Back to A*")
                                new_path = algo.path_planner(map1, newStart, goalloc)
                                new_path1 = []
                                print(new_path)
                                for z in new_path[0]:
                                    new_path1.append(z.location)
                                new_path = new_path1
                                new_path.reverse()
                                A = True
                                # print("After RRT:")
                            printPath(new_path, map)
                            print_list = []
                            new_world_path = []
    
                            for x,y in new_path:
                                new_world_path.append(convertW(y,x))
                                print_list.append([(round(convertW(y,x)[0],3), round(convertW(y,x)[1],3))])
                                if map[int(x)][int(y)] > .75:
                                    print("COLLISION IN NEW PATH", print_list[-1])
                                if(A == False):
                                    for w in range(9):
                                        new_world_path.append(convertW(y,x))
                            print(print_list)  
                            #print(locs[state: -1])
                           # print("before")

                            locs = locs[0:state+1] + new_world_path + locs[state+50:] 

                                #print(locs[state:-1], "AFter")
                            new_path_flag = True
                                

                        
        #if the bearing error is large enough. then prioritize it in the gains
        if(abs(errorb) > .2):
            gainp = 0
            gainb = 3
        #if the bearing is small, prioritize equally
        if(abs(errorb) > 3):
            gainp = 0
            gainb = -3            
        else:
            gainp = 3
            gainb = 6
        #calculating wheelspeeds given errors and gains.
        vR = ((2*errorp*gainp)-(errorb*AXLE_LENGTH*gainb))/2
        vL = ((2*errorp*gainp)+(errorb*AXLE_LENGTH*gainb))/2
            
        #STEP 4: Normalize wheelspeed
        #if the wheelspeeds are greater than max, set them to max
        if(vR > MAX_SPEED):
            vR = MAX_SPEED
        if(vL > MAX_SPEED):
            vL = MAX_SPEED  
        #setting the actual wheelspeeds
        
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)