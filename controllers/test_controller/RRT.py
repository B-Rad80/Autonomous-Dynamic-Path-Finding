import numpy as np
import matplotlib.pyplot as plt
import math
import random

###############################################################################
## Base Code
###############################################################################
class Node:
    """
    Node for RRT Algorithm. This is what you'll make your graph with!
    """
    def __init__(self, pt, parent=None):
        self.point = pt # n-Dimensional point
        self.parent = parent # Parent node
        self.path_from_parent = [] # List of points along the way from the parent node (for edge's collision checking)

def state_is_valid(state, mapa):
    '''
    Function that takes an n-dimensional point and checks if it is within the bounds and not inside the obstacle
    :param state: n-Dimensional point
    :return: Boolean whose value depends on whether the state/point is valid or not
    '''
    if state[0] >= 0: return False
    if state[0] < len(mapa): return False
    if state[1] < len(mapa[0]): return False
    if state[1] >= 0: return False    
    if(mapa[state[0]][state[1]] == 1):
        return False
    return True

return state_bounds, obstacles, state_is_valid

def get_random_valid_vertex(mapa):
    '''
    Function that samples a random n-dimensional point which is valid (i.e. collision free and within the bounds)
    :param state_valid: The state validity function that returns a boolean
    :param bounds: The world bounds to sample points from
    :return: n-Dimensional point/state
    '''
    vertex = None
    while vertex is None: # Get starting vertex
        pt = [random.randrange(0,len(mapa)), random.randrange(0,len(mapa))]
        if state_is_valid(pt):
            vertex = np.array(pt)
    return vertex

###############################################################################
## END BASE CODE
###############################################################################

def get_nearest_vertex(node_list, q_point):
    '''
    Function that finds a node in node_list with closest node.point to query q_point
    :param node_list: List of Node objects
    :param q_point: n-dimensional array representing a point
    :return Node in node_list with closest node.point to query q_point
    '''

    # TODO: Your Code Here
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
    '''
    :param from_point: n-Dimensional array (point) where the path to "to_point" is originating from (e.g., [1.,2.])
    :param to_point: n-Dimensional array (point) indicating destination (e.g., [0., 0.])
    :param delta_q: Max path-length to cover, possibly resulting in changes to "to_point" (e.g., 0.2)
    :return path: Array of points leading from "from_point" to "to_point" (inclusive of endpoints)  (e.g., [ [1.,2.], [1., 1.], [0., 0.] ])
    '''
    # TODO: Figure out if you can use "to_point" as-is, or if you need to move it so that it's only delta_q distance away

    # TODO Use the np.linspace function to get 10 points along the path from "from_point" to "to_point"
    #getting the length of the distance between the two nodes
    dist = np.linalg.norm(to_point-from_point)
    #if the length of the distance is less than the max length then its ok
    if(dist <= delta_q):
    	#getting 10 points between the two nodes
        path = np.linspace(from_point, to_point, num = 10)
    #if the length of the distance between the two nodes is greater than the max
    else:
    	#get the vector between the two nodes and divide by its length to make it a unit vector. then multiply it by the max length to make it the max length. Add the vector to the from point to get the final poin
        sub = (((to_point-from_point)/(dist))*delta_q)+from_point
        #getting 10 point between the start and end point
        path = np.linspace(from_point, sub, num = 10)

    return path

def check_path_valid(path):
    '''
    Function that checks if a path (or edge that is made up of waypoints) is collision free or not
    :param path: A 1D array containing a few (10 in our case) n-dimensional points along an edge
    :param state_is_valid: Function that takes an n-dimensional point and checks if it is valid
    :return: Boolean based on whether the path is collision free or not
    '''

    # TODO: Your Code Here
    #going through every point in the path
    for i in path:
    	#if the point is not valid, return false.
        if (state_is_valid(i) is not True):
            return False
    #gone through every point without returning false, then it must be true
    return True

def rrt(starting_point, goal_point, k, delta_q, mapa):
    '''
    TODO: Implement the RRT algorithm here, making use of the provided state_is_valid function.
    RRT algorithm.
    If goal_point is set, your implementation should return once a path to the goal has been found 
    (e.g., if q_new.point is within 1e-5 distance of goal_point), using k as an upper-bound for iterations. 
    If goal_point is None, it should build a graph without a goal and terminate after k iterations.

    :param state_bounds: matrix of min/max values for each dimension (e.g., [[0,1],[0,1]] for a 2D 1m by 1m square)
    :param state_is_valid: function that maps states (N-dimensional Real vectors) to a Boolean (indicating free vs. forbidden space)
    :param starting_point: Point within state_bounds to grow the RRT from
    :param goal_point: Point within state_bounds to target with the RRT. (OPTIONAL, can be None)
    :param k: Number of points to sample
    :param delta_q: Maximum distance allowed between vertices
    :returns List of RRT graph nodes
    '''
    node_list = []
    node_list.append(Node(starting_point, parent=None)) # Add Node at starting point with no parent

    # TODO: Your code here
    # TODO: Make sure to add every node you create onto node_list, and to set node.parent and node.path_from_parent for each
    # going through k iterations
    for i in range(0,k):
    	#if we have a goal point
        if(goal_point is not None):
        	#if random returns a value less than .01, then use the goal point
            if(random.random() < .1):
                x = goal_point
            #if the random is not less than .01, then use a random valid goal point
            else:
                x = get_random_valid_vertex(mapa)
        #if we dont have a goal point, just use a random valid point
        else:
            x = get_random_valid_vertex(mapa)
        #get the nearest vertex to the point
        v = get_nearest_vertex(node_list, x)
        #getting a path no longer than delta q towards that point
        p = steer(v.point, x, delta_q)
        #if the path is valid
        if(check_path_valid(p) == True):
        	#create a new node at the final point in the path whose parent is the closest vertex
            cool = Node(p[len(p)-1], v)
            #adding the path
            cool.path_from_parent = p
            #adding the node to the list
            node_list.append(cool)
        #if there is a goal point
        if(goal_point is not None):
        	#if the latest node point added is very close to the goal, then end the loop
            if(np.linalg.norm(node_list[len(node_list) - 1].point - goal_point) < 10**-5):
                break
    #returning the list of nodes.
    return node_list