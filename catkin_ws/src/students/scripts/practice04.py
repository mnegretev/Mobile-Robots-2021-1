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

import sys
import numpy
import heapq
import rospy
import copy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import *
from collections import deque

NAME = "MEDINA FERNANDEZ ARMANDO "

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
    execution_steps=0;
    open_list      = [] ############ HINT
    heapq.heapify(open_list) 
    ###Si los datos ya estan en la memoria, es mas eficiente usar
    #heapify() para reordenar los elementos de la lista en su lugar
    in_open_list   = numpy.full(grid_map.shape, False)
    in_closed_list = numpy.full(grid_map.shape, False)
    distances      = numpy.full(grid_map.shape, sys.maxint)
    parent_nodes   = numpy.full((grid_map.shape[0], grid_map.shape[1], 2), -1)

    [r,c] = [start_r, start_c]
    heapq.heappush(open_list, (0, [start_r, start_c])) ###Cuando se usa heappush() el orden de
    #clasificacion de los elementos se mantiene a medida que se agregan nuevos elementos desde una
    #fuente de datos
    in_open_list[start_r, start_c] = True
    distances   [start_r, start_c] = 0

    while len(open_list) > 0 and [r,c] != [goal_r, goal_c]:
        [r,c] = heapq.heappop(open_list)[1]  ###Una vez que el monton este organizado correctamente,
        #usa heappop() para eliminar el elemento con el valor mas bajo.
        in_closed_list[r,c] = True
        neighbors = [[r+1, c],  [r,c+1],  [r-1, c],  [r,c-1]]
        ##dist = distances[r,c] + 1
        for [nr,nc] in neighbors:
            if grid_map[nr,nc] > 40 or grid_map[nr,nc] < 0 or in_closed_list[nr,nc]:
                continue
            g = distances[r,c] + 1 + cost_map[nr][nc]
            if g < distances[nr,nc]:
                distances[nr,nc]    = g
                parent_nodes[nr,nc] = [r,c]
            if not in_open_list[nr,nc]:
                in_open_list[nr,nc] = True
                heapq.heappush(open_list, (g, [nr,nc]))
            execution_steps += 1

    if [r,c] != [goal_r, goal_c]:
        print "Cannot calculate path by Dijkstra:'("
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
    #
    execution_steps=0
    open_list      = [] ############ HINT
    heapq.heapify (open_list) ###Si los datos ya estan en la memoria, es mas eficiente usar
    #heapify() para reordenar los elementos de la lista en su lugar.
    in_open_list   = numpy.full(grid_map.shape, False)
    in_closed_list = numpy.full(grid_map.shape, False)
    distances      = numpy.full(grid_map.shape, sys.maxint)
    parent_nodes   = numpy.full((grid_map.shape[0], grid_map.shape[1], 2), -1)

    [r,c] = [start_r, start_c]
    heapq.heappush(open_list, (0, [start_r, start_c])) ###Cuando se usa heappush(), el orden de
    #clasificacion de los elementos se mantiene a medida que se agregan nuevos elementos desde una 
    #fuente de datos.
    in_open_list[start_r, start_c] = True
    distances   [start_r, start_c] = 0

    while len(open_list) > 0 and [r,c] != [goal_r, goal_c]:
        [r,c] = heapq.heappop(open_list)[1]  ###Una vez que el monton este organizado correctamente,
        #usa heappop() para eliminar el elemento con el valor mas bajo.
        in_closed_list[r,c] = True
        neighbors = [[r+1, c],  [r,c+1],  [r-1, c],  [r,c-1]]
        ##dist = distances[r,c] + 1
        for [nr,nc] in neighbors:
            if grid_map[nr,nc] > 40 or grid_map[nr,nc] < 0 or in_closed_list[nr,nc]:
                continue
            g = distances[r,c] + 1 + cost_map[nr][nc]
            h = abs(nr - goal_r) + abs(nc - goal_c)
            f = g + h
            if g < distances[nr,nc]:
                distances[nr,nc]    = g
                parent_nodes[nr,nc] = [r,c]
            if not in_open_list[nr,nc]:
                in_open_list[nr,nc] = True
                heapq.heappush(open_list, (f, [nr,nc]))
            execution_steps += 1

    if [r,c] != [goal_r, goal_c]:
        print "Cannot calculate path by A*:'("
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
    tam=len(smooth_path)-1

    while gradient_mag>tolerance:
        [x0_i, y0_i]=original_path[0]
        [xn_i, yn_i]=smooth_path[0]
        [xn_in, yn_in]=smooth_path[1]
        grad_x=alpha*(xn_i-x0_i)-beta*(xn_in-xn_i)
        grad_y=alpha*(yn_i-y0_i)+beta*(yn_in-yn_i)
        gradient[0]=[grad_x,grad_y]
        #gradient[0][0] = alpha*(smooth_path[0][0] - original_path[0][0]) - beta*(smooth_path[1][0] - smooth_path[0][0])
        #gradient[0][1] = alpha*(smooth_path[0][1] - original_path[0][1]) - beta*(smooth_path[1][1] - smooth_path[0][1])
        #smooth_path[0][0] = smooth_path[0][0] - epsilon*gradient[0][0]
        #smooth_path[0][1] = smooth_path[0][1] - epsilon*gradient[0][1]
        
        for i in range(1,tam):
           [x0_i, y0_i]=original_path[i]
           [xn_i, yn_i]=smooth_path[i]
           [xn_ip, yn_ip]=smooth_path[i-1]
           [xn_in, yn_in]=smooth_path[i+1]
           grad_x=alpha*(xn_i-x0_i)+beta*(2*xn_i-xn_ip-xn_in)
           grad_y=alpha*(yn_i-y0_i)+beta*(2*yn_i-yn_ip-xn_in)
           gradient[i]=[grad_x,grad_y]

           #gradient[i][0] = alpha*(smooth_path[i][0] - original_path[i][0]) + beta*((2*smooth_path[i][0])-(smooth_path[i-1][0])-(smooth_path[i+1][0]))
           #gradient[i][1] = alpha*(smooth_path[i][1] - original_path[i][1]) + beta*((2*smooth_path[i][1])-(smooth_path[i-1][1])-(smooth_path[i+1][1]))
           #smooth_path[i][0] = smooth_path[i][0] - epsilon*gradient[i][0]
           #smooth_path[i][1] = smooth_path[i][1] - epsilon*gradient[i][1]
        
        [x0_i, y0_i]=original_path[tam]
        [xn_i, yn_i]=smooth_path[tam]
        [xn_ip, yn_ip]=smooth_path[tam-1]
        grad_x=alpha*(xn_i-x0_i)+beta*(-xn_ip+xn_i)
        grad_y=alpha*(yn_i-y0_i)+beta*(-yn_ip+yn_i)
        gradient[tam]=[grad_x,grad_y]

        #gradient[tam][0] = alpha*(smooth_path[tam][0] - original_path[tam][0]) + beta*(smooth_path[tam][0] - smooth_path[tam-1][0])
        #gradient[tam][1] = alpha*(smooth_path[tam][1] - original_path[tam][1]) + beta*(smooth_path[tam][1] - smooth_path[tam-1][1])
        #smooth_path[tam][0] = smooth_path[tam][0] - epsilon*(alpha*(smooth_path[tam][0] - original_path[tam][0]) + beta*(smooth_path[tam][0] - smooth_path[tam-1][0]))
        #smooth_path[tam][1] = smooth_path[tam][1] - epsilon*(alpha*(smooth_path[tam][1] - original_path[tam][1]) + beta*(smooth_path[tam][1] - smooth_path[tam-1][1]))
        #gradient_mag = numpy.linalg.norm(gradient)
        
        for i in range(0,tam):
           smooth_path[i][0] -= epsilon*gradient[i][0]
           smooth_path[i][1] -= epsilon*gradient[i][1]
        gradient_mag=0

        for i in range(0,len(smooth_path)):
           gradient_mag+=abs(gradient[i][0])+abs(gradient[i][1])

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
    print "PRACTICE 02 - " + NAME
    rospy.init_node("practice02")
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
    
