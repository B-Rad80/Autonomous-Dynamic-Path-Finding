import numpy as np
import matplotlib.pyplot as plt
import math
import random


class Node:
    def __init__(self, pt, parent=None):
        self.point = pt # n-Dimensional point
        self.parent = parent # Parent node
        self.path_from_parent = [] # List of points along the way from the parent node (for edge's collision checking)

def state_is_valid(state, mapa):
    if state[0] < 0: return False
    if state[0] >= len(mapa): return False
    if state[1] >= len(mapa): return False
    if state[1] < 0: return False    
    if(mapa[round(state[0])][round(state[1])] >= 1):
        return False
    return True

def get_random_valid_vertex(mapa):
    vertex = None
    while vertex is None: # Get starting vertex
        pt = [random.randrange(0,400), random.randrange(0,400)]
        if state_is_valid(pt, mapa):
            vertex = np.array(pt)
    return vertex


def get_nearest_vertex(node_list, q_point):

    #min node and min value variables 
    minv = None
    minn = None
    #going through each node in the list
    for i in node_list:
    	#getting the length of the vector between the two points
        x = np.linalg.norm(i.point-q_point)
        #if there is no min node, then it automatically is the min node
        if(minn == None):
            minn = i
            minv = x
        #if there is a min node and it's less than it, then its the new min node
        elif(x < minv):
            minn = i
            minv = x
    #return the minimum of all nodes
    return minn
def steer(from_point, to_point, delta_q):

    dist = np.linalg.norm(to_point-from_point)
    #if the length of the distance is less than the max length then its ok
    if(dist <= delta_q):
    	#getting 10 points between the two nodes
        path = np.linspace(from_point, to_point, num = 50)
    #if the length of the distance between the two nodes is greater than the max
    else:
    	#get the vector between the two nodes and divide by its length to make it a unit vector. then multiply it by the max length to make it the max length. Add the vector to the from point to get the final poin
        sub = (((to_point-from_point)/(dist))*delta_q)+from_point
        #getting 10 point between the start and end point
        path = np.linspace(from_point, sub, num = 50)
        for i in range(len(path)):
            path[i] = np.array([int(path[i][0]),int(path[i][1])])

    return path

def check_path_valid(path, mapa):
    #going through every point in the path
    for i in path:
    	#if the point is not valid, return false.
        if (state_is_valid(i, mapa) is not True):
            return False
    #gone through every point without returning false, then it must be true
    return True

def rrt(starting_point, goal_point, k, delta_q, mapa):

    node_list = []
    node_list.append(Node(starting_point, parent=None)) # Add Node at starting point with no parent

    # going through k iterations
    broken = False
    for i in range(0,k):

       
    	#if we have a goal point
        
        if(goal_point is not None):
        	#if random returns a value less than .01, then use the goal point
            if(random.random() < .1):
                x = goal_point
            #if the random is not less than .01, then use a random valid goal point
            else:
                #print("pre get random valid vertex")
                x = get_random_valid_vertex(mapa)
        #if we dont have a goal point, just use a random valid point
        else:
            x = get_random_valid_vertex(mapa)
        #print(x, "random valid vertex")
        #print("pre get_nearest")
        x = np.array([int(x[0]),int(x[1])])
        #get the nearest vertex to the point
        v = get_nearest_vertex(node_list, x)
        #getting a path no longer than delta q towards that point
        #print("pre steer")
        p = steer(v.point, x, delta_q)
        #if the path is valid
        #print("pre valid path")
        if(check_path_valid(p, mapa) == True):
        	#create a new node at the final point in the path whose parent is the closest vertex
            index = node_list.index(v)
            cool = Node(p[-1], index )
            #adding the path
            cool.path_from_parent = p
            #adding the node to the list
            node_list.append(cool)
        #if there is a goal point
        if(goal_point is not None):
        	#if the latest node point added is very close to the goal, then end the loop
            if(np.linalg.norm(np.subtract(node_list[len(node_list) - 1].point, goal_point)) < 10**-5):
                broken = True
                break
    #returning the list of nodes.
    if(broken == True):
        nodes = []
        nodes.append(node_list[-1])
        parent = node_list[node_list[-1].parent]
        points =[]
        points.append(parent.point)
        while not parent == None:
            if(parent.parent):
                    nodes.append(node_list[parent.parent])
                    parent = node_list[parent.parent]
                    points.append(parent.point)
            else:
                points.reverse()
                return points

        print("other points", points)
        return points.reverse()
    else:
        return None
