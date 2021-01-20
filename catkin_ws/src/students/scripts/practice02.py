#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-1
# PRACTICE 2 - PATH PLANNING BY DIJKSTRA AND A-STAR
#
# Instructions:
# Write the code necessary to plan a path using two search algorithms:
# Dijkstra and A*
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT
#

import sys
import numpy
import heapq
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import *
from collections import deque

NAME = "AMALFI_FIGUEROA_ISAAC"

def dijkstra(start_r, start_c, goal_r, goal_c, grid_map, cost_map):

    #TOMAMOS EL ALGORTIMOS DE BUSQUEDA  breadth_first_search() DE LA SESION ANTERIOR
    #las lineas comentadas fueron las modificadas para este algoritmo

    execution_steps=0
    open_list = [] # deque()
    heapq.heapify(open_list) # Add this line
    in_open_list   = numpy.full(grid_map.shape, False)
    in_closed_list = numpy.full(grid_map.shape, False)
    distances      = numpy.full(grid_map.shape, sys.maxint)
    parent_nodes   = numpy.full((grid_map.shape[0], grid_map.shape[1], 2), -1)

    [r,c] = [start_r, start_c]
    heapq.heappush(open_list, (0, [start_r, start_c]))
    in_open_list[start_r, start_c] = True
    distances   [start_r, start_c] = 0

    while len(open_list) > 0 and [r,c] != [goal_r, goal_c]:
        [r,c] = heapq.heappop(open_list)[1] #[r,c] = open_list.popleft()
        in_closed_list[r,c] = True
        neighbors = [[r+1, c],  [r,c+1],  [r-1, c],  [r,c-1]]
        # dist = distances[r,c] + 1
        for [nr,nc] in neighbors:
            if grid_map[nr,nc] > 40 or grid_map[nr,nc] < 0 or in_closed_list[nr,nc]:
                continue
            g = distances[r,c] + 1 + cost_map[nr][nc] 
            if g < distances[nr,nc]:
                distances[nr,nc]    = g
                parent_nodes[nr,nc] = [r,c]
            if not in_open_list[nr,nc]:
                in_open_list[nr,nc] = True
                heapq.heappush(open_list, (g, [nr, nc])) # open_list.append([nr,nc])
            execution_steps += 1

    if [r,c] != [goal_r, goal_c]:
        print "Cannot calculate path by Dijkstra:'(" #print "Cannot calculate path by Breadth First Search:'("
        return []
    print "Path calculated after " + str(execution_steps) + " steps."
    path = []
    while [parent_nodes[r,c][0],parent_nodes[r,c][1]] != [-1,-1]:
        path.insert(0, [r,c])
        [r,c] = parent_nodes[r,c]
    return path


def a_star(start_r, start_c, goal_r, goal_c, grid_map, cost_map):

    #TOMAMOS EL ALGORITMO DE BUSQUEDA DE LA SESION ANTERIOR
    #las lineas comentadas fueron las modificadas para este algoritmo

    execution_steps=0
    open_list      = []
    in_open_list   = numpy.full(grid_map.shape, False)
    in_closed_list = numpy.full(grid_map.shape, False)
    distances      = numpy.full(grid_map.shape, sys.maxint)
    parent_nodes   = numpy.full((grid_map.shape[0], grid_map.shape[1], 2), -1)

    [r,c] = [start_r, start_c]
    heapq.heappush(open_list, [0, 0, start_r, start_c])   ##### Indice 0 es el valor f. Indice 1 es el valor g
    in_open_list[start_r, start_c] = True
    distances   [start_r, start_c] = 0

    while len(open_list) > 0 and [r,c] != [goal_r, goal_c]:
        r_temp = open_list[0][2]
        c_temp = open_list[0][3]
        heapq.heappop(open_list)    ###### Sacar el menor elemento de la lista
        [r,c]  = [r_temp,c_temp]
        in_closed_list[r,c] = True
        neighbors = [[r+1, c],  [r,c+1],  [r-1, c],  [r,c-1]]
        for [nr,nc] in neighbors:
            if grid_map[nr,nc] > 40 or grid_map[nr,nc] < 0 or in_closed_list[nr,nc]:
                continue
            g         = distances[r,c] + 1 + cost_map[nr,nc]     #### Calculo de g
            manhattan = abs(nr-goal_r) + abs(nc-goal_c)    ##### Distancia de Manhattan
            f         = g + manhattan
            if g < distances[nr,nc]:     
                distances[nr,nc]    = g
                parent_nodes[nr,nc] = [r,c]
            if not in_open_list[nr,nc]:
                in_open_list[nr,nc] = True
                heapq.heappush(open_list, [f, g, nr, nc])     #### Anadir a la open_list con valor f
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
        cost_map = static_map
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

    msg_path = Path()
    msg_path.header.frame_id = "map"
    for [r,c] in path:
        p = PoseStamped()
        p.pose.position.x = c*static_map.info.resolution + static_map.info.origin.position.x
        p.pose.position.y = r*static_map.info.resolution + static_map.info.origin.position.y
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
