#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-1
# PRACTICE 4 - PATH SMOOTHING BY GRADIENT DESCEND
#
# Instructions:
# Write the code necessary to smooth a path using the gradient descend algorithm.
# Re-use the practice02 codes to implement the Dijkstra and A* algorithm.
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT
#
import math as m
import sys
import numpy
import heapq
import rospy
import copy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import *
from collections import deque

NAME = "ARIAS_PELAYO"

def dijkstra(start_r, start_c, goal_r, goal_c, grid_map, cost_map):
    #
    # TODO:
    # Write a Dijkstra algorithm to find a path in an occupancy grid map given the start cell
    # [start_r, start_c], the goal cell [goal_r, goal_c] and the map 'grid_map'.
    # Return the set of points of the form [[start_r, start_c], [r1,c1], [r2,c2], ..., [goal_r, goal_c]]
    # If path cannot be found, return an empty tuple []
    # Hint: Use a priority queue to implement the open list. 
    # Documentation to implement priority queues in python can be found in
    # https://docs.python.org/2/library/heapq.html
    #
    execution_steps=0
    open_list = []
    heapq.heapify(open_list) #necesitamos una lista con prioridad
    in_open_list   = numpy.full(grid_map.shape, False)
    in_closed_list = numpy.full(grid_map.shape, False)
    distances      = numpy.full(grid_map.shape, sys.maxint)
    #g_values  = numpy.full(grid_map.shape, sys.maxint)
    parent_nodes   = numpy.full((grid_map.shape[0], grid_map.shape[1], 2), -1)

    [r,c] = [start_r, start_c]
    heapq.heappush(open_list, (0, [start_r, start_c]))
    in_open_list[start_r, start_c] = True
    distances   [start_r, start_c] = 0

    while len(open_list) > 0 and [r,c] != [goal_r, goal_c]:
        [r,c] = heapq.heappop(open_list)[1]  #se hace pop del menor valor, elemento es peso y el nuevo elemento es 1
        in_closed_list[r,c] = True
        neighbors = [[r+1, c],  [r,c+1],  [r-1, c],  [r,c-1]]

        #dist = distances[r,c] + 1

        for [nr,nc] in neighbors:
            if grid_map[nr,nc] > 40 or grid_map[nr,nc] < 0 or in_closed_list[nr,nc]:
                continue
            peso = distances[r,c] + 1 + cost_map[nr][nc]
            if peso < distances[nr,nc]:
                distances[nr,nc]    = peso
                parent_nodes[nr,nc] = [r,c]
            if not in_open_list[nr,nc]:
                in_open_list[nr,nc] = True
                heapq.heappush(open_list, (peso, [nr,nc]))#agregamos elemento a la lista heap
            execution_steps += 1

    if [r,c] != [goal_r, goal_c]:
        print "Cannot calculate path by Dijkstra'("
        return []
    print "Path calculated after " + str(execution_steps) + " steps."
    path = []
    while [parent_nodes[r,c][0],parent_nodes[r,c][1]] != [-1,-1]:
        path.insert(0, [r,c])
        [r,c] = parent_nodes[r,c]
    return path

def a_star(start_r, start_c, goal_r, goal_c, grid_map, cost_map):
    #
    # TODO:
    # Write a A* algorithm to find a path in an occupancy grid map given the start cell
    # [start_r, start_c], the goal cell [goal_r, goal_c] and the map 'grid_map'.
    # Return the set of points of the form [[start_r, start_c], [r1,c1], [r2,c2], ..., [goal_r, goal_c]]
    # If path cannot be found, return an empty tuple []
    # Use Manhattan distance as heuristic function
    # Hint: Use a priority queue to implement the open list
    # Documentation to implement priority queues in python can be found in
    # https://docs.python.org/2/library/heapq.html
    execution_steps=0
    open_list = []
    heapq.heapify(open_list) #necesitamos una lista con prioridad
    in_open_list   = numpy.full(grid_map.shape, False)
    in_closed_list = numpy.full(grid_map.shape, False)
    distances      = numpy.full(grid_map.shape, sys.maxint)
    parent_nodes   = numpy.full((grid_map.shape[0], grid_map.shape[1], 2), -1)

    [r,c] = [start_r, start_c]
    heapq.heappush(open_list, (0, [start_r, start_c]))
    in_open_list[start_r, start_c] = True
    distances   [start_r, start_c] = 0

    while len(open_list) > 0 and [r,c] != [goal_r, goal_c]:
        [r,c] = heapq.heappop(open_list)[1]  #se hace pop del menor valor
        in_closed_list[r,c] = True
        neighbors = [[r+1, c],  [r,c+1],  [r-1, c],  [r,c-1]]

        #dist = distances[r,c] + 1

        for [nr,nc] in neighbors:
            if grid_map[nr,nc] > 40 or grid_map[nr,nc] < 0 or in_closed_list[nr,nc]:
                continue
            peso = distances[r,c] + 1 + cost_map[nr][nc]
            h = m.sqrt(pow((nr - goal_r),2) + pow(nc - goal_c,2))#heuristica: subestima el costo hasta llegar al nodo meta aka dis. Manhattan
            f = peso +h 
            if peso < distances[nr,nc]:
                distances[nr,nc]    = peso
                parent_nodes[nr,nc] = [r,c]
            if not in_open_list[nr,nc]:
                in_open_list[nr,nc] = True
                heapq.heappush(open_list, (f, [nr,nc]))#agregamos elemento a la lista heap
            execution_steps += 1

    if [r,c] != [goal_r, goal_c]:
        print "Cannot calculate path by A*'("
        return []
    print "Path calculated after " + str(execution_steps) + " steps."
    path = []
    while [parent_nodes[r,c][0],parent_nodes[r,c][1]] != [-1,-1]:
        path.insert(0, [r,c])
        [r,c] = parent_nodes[r,c]
    return path

def get_smooth_path(original_path, alpha, beta):
    #
    # TODO:
    # Write an algorithm to smooth the 'original_path' and return the new path.
    # The path is given as a set of points [x,y] in the form:
    # [[x0,y0], [x1,y1], ..., [xn,ym]].
    # Example. The following line of code
    # [xo_i,yo_i] = original_path[i]
    # stores the x,y coordinates of the i-th point of the original path
    # in the variables xo_i and yo_i respectively. 
    #
    #
    smooth_path  = copy.deepcopy(original_path)            # At the beginnig, the smooth path is the same than the original path.
    tolerance    = 0.00001                                 # If gradient magnitude is less than a tolerance, we consider.
    gradient_mag = tolerance + 1                           # we have reached the local minimum.
    gradient     = [[0,0] for i in range(len(smooth_path))]# Gradient has N components of the form [x,y]. 
    epsilon      = 0.5                                     # This variable will weight the calculated gradient.

    N = len(smooth_path) -1

    while gradient_mag > tolerance:
      #Se calcula xn_0
      gradient[0][0] = alpha* (smooth_path[0][0] - original_path[0][0]) - beta * (smooth_path[1][0]-smooth_path[0][0])
      smooth_path[0][0] = smooth_path[0][0] - epsilon * gradient[0][0]

      #Duda: si trato de combinar las dos instrucciones anteriores en una misma linea de codigo el suavizado de curvas no me lo hace bien.
      #Lo mismo ocure en el resto de lineas comentadas en la funcion
      #smooth_path[0][0] =smooth_path[0][0] -  epsilon*(alpha* (smooth_path[0][0] - original_path[0][0]) - beta * (smooth_path[1][0]-smooth_path[0][0]))

      #Se calcula yn_0
      gradient[0][1] = alpha* (smooth_path[0][1] - original_path[0][1]) - beta * (smooth_path[1][1]-smooth_path[0][1])
      smooth_path[0][1] = smooth_path[0][1] - epsilon * gradient[0][1]
      

      #smooth_path[0][1] =smooth_path[0][1] -  epsilon*(alpha* (smooth_path[0][1] - original_path[0][1]) - beta * (smooth_path[1][1]-smooth_path[0][1]))
      
      for i in range(1,N - 1):
        #Se calcula xn_i
        gradient[i][0] = alpha * (smooth_path[i][0] - original_path[i][0]) + beta * (2*smooth_path[i][0] - smooth_path[i-1][0] - smooth_path[i+1][0])
        smooth_path[i][0] = smooth_path[i][0] - epsilon * gradient[i][0]
        #smooth_path[i][0] =smooth_path[i][0] -  epsilon*(alpha* (smooth_path[i][0] - original_path[i][0]) - beta * (2*smooth_path[i][0] - smooth_path[i-1][0] - smooth_path[i+1][0]))

        #Se calcula yn_i
        gradient[i][1] = alpha * (smooth_path[i][1] - original_path[i][1]) + beta * (2*smooth_path[i][1] - smooth_path[i-1][1] - smooth_path[i+1][1])
        smooth_path[i][1] = smooth_path[i][1] - epsilon * gradient[i][1]
        #smooth_path[i][1] =smooth_path[i][1] -  epsilon*(alpha* (smooth_path[i][1] - original_path[i][1]) - beta * (2*smooth_path[i][1] - smooth_path[i-1][1] - smooth_path[i+1][1]))
        

      #Se calcula xnN
      gradient[N][0] = alpha * (smooth_path[N][0] - original_path[N][0]) + beta * (smooth_path[N][0] - smooth_path[N-1][0])
      smooth_path[N][0] = smooth_path[N][0] - epsilon * gradient[N][0]
      #smooth_path[N][0] = smooth_path[N][0] - epsilon * (alpha * (smooth_path[N][0] - original_path[N][0]) + beta * (smooth_path[N][0] - smooth_path[N-1][0]))

      #Se calcula ynN
      gradient[N][1] = alpha * (smooth_path[N][1] - original_path[N][1]) + beta * (smooth_path[N][1] - smooth_path[N-1][1])
      smooth_path[N][1] = smooth_path[N][1] - epsilon * gradient[N][1]
      #smooth_path[N][1] = smooth_path[N][1] - epsilon * (alpha * (smooth_path[N][1] - original_path[N][1]) + beta * (smooth_path[N][1] - smooth_path[N-1][1]))

      gradient_mag = numpy.linalg.norm(gradient)#Se sale del while al cumplir condicion gradient_mag > tolerance
    
    return smooth_path


def get_maps():
    clt_static_map = rospy.ServiceProxy("/static_map"  , GetMap)
    clt_cost_map   = rospy.ServiceProxy("/cost_map"    , GetMap)
    clt_inflated   = rospy.ServiceProxy("/inflated_map", GetMap)
    static_map   = clt_static_map()
    static_map   = static_map.map
    try:
        inflated_map = clt_inflated()
        inflated_map = inflated_map.map
    except:
        inflated_map = static_map
        print("Cannot get inflated map. Using static map instead")
    inflated_map = numpy.asarray(inflated_map.data)
    inflated_map = numpy.reshape(inflated_map, (static_map.info.height, static_map.info.width))
    try:
        cost_map = clt_cost_map()
        cost_map = cost_map.map
    except:
        cost_map = inflated_map
        print("Cannot get cost map. Using static map instead")
    cost_map = numpy.asarray(cost_map.data)
    cost_map = numpy.reshape(cost_map, (static_map.info.height, static_map.info.width))
    return [static_map, inflated_map, cost_map]

def generic_callback(req, algorithm):
    [static_map, inflated_map, cost_map] = get_maps()
    
    [start_x, start_y] = [req.start.pose.position.x, req.start.pose.position.y]
    [goal_x,  goal_y ] = [req.goal.pose.position.x , req.goal.pose.position.y ]
    [zero_x,  zero_y ] = [static_map.info.origin.position.x,static_map.info.origin.position.y]
    [start_c, start_r] = [int((start_x - zero_x)/static_map.info.resolution), int((start_y - zero_y)/static_map.info.resolution)]
    [goal_c , goal_r ] = [int((goal_x  - zero_x)/static_map.info.resolution), int((goal_y  - zero_y)/static_map.info.resolution)]

    if algorithm == 'dijkstra':
        print("Calculating path by Dijkstra from " + str([start_x, start_y])+" to "+str([goal_x, goal_y]))
        path = dijkstra(start_r, start_c, goal_r, goal_c, inflated_map, cost_map)
    else:
        print("Calculating path by A* from " + str([start_x, start_y])+" to "+str([goal_x, goal_y]))
        path = a_star(start_r, start_c, goal_r, goal_c, inflated_map, cost_map)

    smooth_path = []
    for [r,c] in path:
        x = c*static_map.info.resolution + static_map.info.origin.position.x
        y = r*static_map.info.resolution + static_map.info.origin.position.y
        smooth_path.append([x,y])
    if rospy.has_param("/navigation/path_planning/smoothing_alpha"):
        alpha = rospy.get_param("/navigation/path_planning/smoothing_alpha")
    else:
        alpha = 0.5
    if rospy.has_param("/navigation/path_planning/smoothing_beta"):
        beta = rospy.get_param("/navigation/path_planning/smoothing_beta")
    else:
        beta = 0.5
    smooth_path = get_smooth_path(smooth_path, alpha, beta)
        
    msg_path = Path()
    msg_path.header.frame_id = "map"
    for [x,y] in smooth_path:
        p = PoseStamped()
        p.pose.position.x = x
        p.pose.position.y = y
        msg_path.poses.append(p)
    pub_path = rospy.Publisher('/navigation/calculated_path', Path, queue_size=10)
    pub_path.publish(msg_path)
    pub_path.publish(msg_path)
    return GetPlanResponse(msg_path)

def callback_dijkstra(req):
    return generic_callback(req, 'dijkstra')

def callback_a_star(req):
    return generic_callback(req, 'a_star')

def main():
    print "PRACTICE 04 - " + NAME
    rospy.init_node("practice04")
    rospy.Service('/navigation/path_planning/dijkstra_search', GetPlan, callback_dijkstra)
    rospy.Service('/navigation/path_planning/a_star_search'  , GetPlan, callback_a_star)
    rospy.wait_for_service('/static_map')
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
