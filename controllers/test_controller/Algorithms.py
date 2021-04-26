#from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d # Uncomment if you want to use something else for finding the configuration space
from queue import PriorityQueue

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

    if(map[start[0]][start[1]] >= 1):
        print("INVALID STARTING LOCATION")
        return "INVALID STARTING LOCATION"
    if(map[goal[0]][goal[1]]>= 1):
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


def gen_box_map(map : list, box_size, save : bool = False):
    box_map =  map.copy()
    #if 1 in map:
     #   print("1 found!")
    for x in range(len(map)):
        for y in range(len(map[x])):
            #print(map[x][y], "map_XY")
            if map[x][y] >= 1 and map[x][y] < 2:
                #print(map[x][y], "gen_box_map", x, y)
    
                for xb in range(box_size):
                    for yb in range(box_size):

                        if x-box_size/2 + xb > 0 and x-box_size/2 + xb < len(map) and y-box_size/2 + yb > 0 and y-box_size/2 + yb < len(map):

                            box_map[int(x-box_size/2 + xb)][int(y-box_size/2+ yb)] = 2

    for row in range(len(box_map)):
        for val in range(len(box_map[row])):
            
            if not box_map[row][val] == 2:
                box_map[row][val] = 0
            '''
            if box_map[row][val] == 2:
                box_map[row][val] = 1
            else:
                box_map[row][val] = 0
            '''
            

    if 1 in box_map:
        print("failure")     

    if(save):
        with open('box_map.npy', 'wb') as f:
            np.save(f, box_map)
            print("saved box map")

    return box_map

def printMap(map):
    mapa = map.copy()
    for i in range(len(mapa)):
        for k in range(len(mapa)):
            if mapa[i][k] >= 1:
                mapa[i][k] = 1
        
    plt.imshow(mapa)
    plt.show()
if __name__ == "__main__": 
    map = loadmap("./test.npy")

    printMap(map)
    map = gen_box_map(map, 20, True)
    #map = loadmap("./box_map.npy")
    printMap(map)