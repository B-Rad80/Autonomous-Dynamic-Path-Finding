
"""lab5 controller."""
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d # Uncomment if you want to use something else for finding the configuration space
from queue import PriorityQueue

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

# The Tiago robot has a couple more sensors than the e-Puck
# Some of them are mentioned below. We will use its LiDAR for Lab 5

# range = robot.getDevice('range-finder')
# range.enable(timestep)
# camera = robot.getDevice('camera')
# camera.enable(timestep)
# camera.recognitionEnable(timestep)
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

# The display is used to display the map. We are using 360x360 pixels to
# map the 12x12m2 apartment
display = robot.getDevice("display")

# Odometry
pose_x     = 2.58
pose_y     = 8.9
pose_theta = 0

vL = 0
vR = 0


# mode = 'planner'
mode = 'autonomous'

lidar_sensor_readings = []
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # remove blocked sensor rays

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

###################
#
# Planner
#
###################
if mode == 'planner':

    class Node:
        def __init__(self, loc = None, w = float('inf'), parent = None):
            self.location = loc
            self.weight = w
            self.neighbors = None
            self.parent = parent
            self.combined_weight = w
    
        #Prioity Queue compares nodes (Node < Node2 ) as defined below based on their combined weight which
            # in this case is equal to the node weight + heuristic weight 
        def __lt__(self, other):
            return self.combined_weight < other.combined_weight
    
        def __le__(self, other):
            return self.combined_weight <= other.combined_weight
    
        def __gt__(self, other):
            return self.combined_weight > other.combined_weight
    
        def __ge__(self, other):
            return self.combined_weight >= other.combined_weight
        #def __eq__(self, other):
           # return (self.combined_weight == other.combined_weight and self.location == other.location and 
            #    self.weight == other.weight and self.neightbors == other.neighbors and self.parent == other.parent)
    
    class Tree:
        def __init__(self):
            self.root = None
    def path_planner(map, start, goal):    
        tree = Tree()
        node_list = []
        visited = []
    
        q = PriorityQueue()
    
        if(map[start[0]][start[1]] == 1):
            print("INVALID STARTING LOCATION")
            return "INVALID STARTING LOCATION"
        if(map[goal[0]][goal[1]]==1):
            print("INVALID GOAL LOCATION")
            return "INVALID GOAL LOCATION"
    
        #returns the number of non-unique values (for testing)
        def unique(arr):
            ct = 0
            t_arr = []
            for i in arr:
                if i in t_arr:
                    ct+1
                t_arr.append(i)
            return ct
        
        #finds the final solution path by going to every nodes parent (root/start == None)
        def find_path(end):
            best_path = []
            path_not_found = True
    
            best_path.append(end)
            ctr = 0
            while path_not_found:
    
                if best_path[ctr].parent == None:
                    print("final Locations", best_path[ctr].location, tree.root.location)
                    if(best_path[ctr].location == tree.root.location):
                        print("success!")
                    else:
                        print("Something went wrong")
    
                    path_not_found == False
                    return best_path
                #print(node_list[best_path[ctr].parent].location, "Location")
                best_path.append(node_list[best_path[ctr].parent])
                ctr+= 1
    
            
        #euclidian distance
        def euc_dist(start, goal):
            dist = ((start[0] - goal[0])**2 + (start[1] - goal[1]))**(1/2)
            #print(int(dist.real))
            return int(dist.real) # had to do this to handle complex numbers?
        def manhatten_dist(start, goal):
            dist = abs(start[0]- goal[0]) + abs(start[1]- goal[1])
            return dist
        
        #returns an array of neighbors for a given node who exist in the map and are not an obsticle and not visited
        def check_neighbors_exist(possible_neighbors, map, visited):
            map_size = len(map)
            neighbors = []
            #unq_cnt = unique(visited)
            #print(unq_cnt, "visit")
            for x, y in possible_neighbors:
                if(x < map_size and x > 0 and y < map_size and y > 0 ):
                    if(map[x][y] == 0 and (x, y) not in visited):
                        visited.append((x,y))
                        neighbors.append((x,y))
            return neighbors
    
    
        # A* time 
        tree.root = Node(start, 0, None)
        print(tree.root.parent, "parent")
        node_list.append(tree.root)
    
        q.put(tree.root)
        tnode1 = Node(start, 0, None)
        tnode2 = Node(start, 0, None)
        tnode3 = Node(start, 0, None)
    
        visited.append(tree.root.location)
    
    
        while not q.empty():
            cur_node = q.get()            
    
            #print("\n\nnew node ",cur_node.location, cur_node.weight)
    
            # check to see if we are at the goal 
            if cur_node.location == goal:
                print(cur_node.location, node_list[cur_node.parent].location, "compare curnode to parent")
                best_path = find_path(cur_node)
                return best_path, visited
    
            x, y = cur_node.location
            
            #get neighbors
            neighbors = [(x-1, y), (x+1, y), (x,y+1), (x,y-1)]
            cur_node.neighbors = check_neighbors_exist(neighbors, map, visited)
            
            #for each neighbor calculate the parent, heuristic + node weight, and append to node_list and priority queue
            for xy in cur_node.neighbors:
               
                index = node_list.index(cur_node)
                tmp_node = Node(xy,cur_node.weight+1, index)
                tmp_node.combined_weight =  manhatten_dist(xy, goal) + cur_node.weight + 1
    
                #if(tmp_node.parent == None):
              #     print(" parent == root ")
               # else:
                    #print(node_list[tmp_node.parent].location, "parent ")
                node_list.append(tmp_node)
    
               # print(tmp_node.combined_weight, tmp_node.location, "info")
                q.put(tmp_node)
    
        return "failed to find path"



    def loadmap(file):
         with open(file, 'rb') as f:
                   return np.load(f)
    map = loadmap("test.npy")


    def gen_box_map(map, box_size):
        box_map =  np.zeros_like(map)
        if 1 in map:
            print("1 found!")
        for x in range(len(map)):
            for y in range(len(map[x])):
                #print(map[x][y], "map_XY")
                if map[x][y] >= 1:
        
                    for xb in range(box_size+2):
                        for yb in range(box_size+2):
    
                            if x-box_size/2 + xb > 0 and x-box_size/2 + xb < len(map) and y-box_size/2 + yb > 0 and y-box_size/2 + yb < len(map):
    
                                box_map[int(x-box_size/2 + xb)][int(y-box_size/2+ yb)] = 2
    
        for row in range(len(box_map)):
            for val in range(len(box_map[row])):
                if box_map[row][val] == 2:
                    box_map[row][val] = 1
                else:
                    box_map[row][val] = 0
    
        if 2 in box_map:
            print("failure")     
    
        with open('box_map.npy', 'wb') as f:
                    np.save(f, box_map)
    
        return box_map
    box_map = gen_box_map(map, 11)
    plt.imshow(box_map)
    plt.show()
# Part 2.3 continuation: Call path_planner
    #start positions in world coordinates
    #equations to convert map points to world points and world points to map points
    def convertM(x,y):
        return (round(y*(30)),round(x*(30)))
    def convertW(x,y):
        return(y*(1/30), x*(1/30))
    #starting world coordinates of the bot
    s_x = 1
    s_y = .815
    #converting the coordinates to map coordinates
    s_x1, s_y1 = convertM(s_x, s_y)
    #ending coordinates for the bot
    f_x = 8.03
    f_y = 13.6
    #converting the coordinates to map coordinates
    f_x1, f_y1 = convertM(f_x, f_y)

    #running the two sets of coordinates on the path planner using the configuration space map    
    best_path, visited = path_planner(box_map, (s_y1 ,s_x1), (f_y1,f_x1))
    locs = []
    #going through each point in the path and adding it to the list 
    for n in reversed(best_path):
        locs.append(n.location)
    # print("best path found in ", len(locs), " steps")
    
    
# Part 2.4: Turn paths into goal points and save on disk as path.npy and visualize it
    #making it so we can visualize the path in the box_map
    for x,y in visited:
        box_map[x][y] = .2
    for x,y in locs:
        box_map[x][y] = .5
    plt.imshow(box_map)
    #making a file with all the path coordinates
    with open('path.npy', 'wb') as f:
        np.save(f, locs)
    #making a file with the final path
    with open('final.npy', 'wb') as f:
        np.save(f, box_map)
    plt.show()


# Part 1.2: Map Initialization

# Initialize your map data structure here as a 2D floating point array
if mode == 'manual':
    map = np.zeros((500,500), dtype=float)

if mode == 'autonomous':
# Part 3.1: Load path from disk and visualize it (Make sure its properly indented)
    def loadmap(file):
         with open(file, 'rb') as f:
                   return np.load(f)
    #opening the visualization of the path and the path
    map = loadmap("final.npy")
    points = loadmap('path.npy')
    plt.imshow(map)
    plt.show()
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

while robot.step(timestep) != -1 and mode != 'planner':
###################
#
# Sensing
#
###################
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
# Part 1.3: visualize map gray values.

            # You will eventually REPLACE the following 2 lines with a more robust version of map
            # and gray drawing that has more levels than just 0 and 1.

            
            m_x = int(wx*30-1) -1 
            m_y = int(wy*30-1) -1 
            if(map[m_x][m_y] <= 1):
            
               map[m_x][m_y] += .01
            
            g = map[m_x][m_y]
            if(g>.75):
            
                color = int(g*256**2 + g**256 + g)*255
                
                if(color > int(0xFFFFFF)):
                
                    color = 0xFFFFFF
                
                display.setColor(color)
                display.drawPixel(450- int(wy*30),int(wx*30))

    display.setColor(int(0xFF0000))
    display.drawPixel(450-int(pose_y*30),int(pose_x*30))
    
    
    
    #The equation to calculate the colors assumes g to be in the range of 0..255, not 0..1
    #display.setColor(int(g*256**2+g*256+g))
    #g=map[i][j]*255




###################
#
# Controller
#
###################
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
# Part 1.4: Save map to disc
        

            print("Map file saved")
        elif key == ord('L'):
            # You will not use this portion but here's an example for loading saved a numpy array
            map = np.load("map.npy")
            print("Map loaded")
        else: # slow down
            vL *= 0.75
            vR *= 0.75
    else: # not manual mode
# Part 3.2: Feedback controller
        #STEP 1: Calculate the error
        #position an  
        
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
                else:
                    #continue on
                    continue
        #if the bearing error is large enough. then prioritize it in the gains
        if(abs(errorb) > .2):
            gainp = .2
            gainb = 3
        #if the bearing is small, prioritize equally
        if(abs(errorb) > 3):
            gainp = .2
            gainb = -3            
        else:
            gainp = 6
            gainb = 6
        #STEP 3: Compute wheelspeeds
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
        
        #STEP 3: Compute wheelspeeds


    # Normalize wheelspeed
    # Keep the max speed a bit less to minimize the jerk in motion


    # Odometry code. Don't change speeds after this
    # We are using GPS and compass for this lab to get a better pose but this is how you'll do the odometry
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

    # print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta)) #/3.1415*180))

    # Actuator commands
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)