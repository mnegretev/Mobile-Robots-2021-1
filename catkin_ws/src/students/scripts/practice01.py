#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-1
# PRACTICE 1 - PATH PLANNING BY BREADTH FIRST SEARCH AND DEPTH FIRST SEARCH
#
# Instructions:
# Write the code necessary to plan a path using two search algorithms:
# Breadth first search and Depth first search
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT


import sys
import numpy
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import *
from collections import deque

NAME = "Martinez-Fadul-Jesus"


static_map = None
grid_map   = None

def breadth_first_search(start_r, start_c, goal_r, goal_c, grid_map):
    #
    # TODO:
    # Write a breadth first search algorithm to find a path in an occupancy grid map given the start cell
    # [start_r, start_c], the goal cell [goal_r, goal_c] and the map 'grid_map'.
    # Return the set of points of the form [[start_r, start_c], [r1,c1], [r2,c2], ..., [goal_r, goal_c]]
    # If path cannot be found, return an empty tuple []
    #
    execution_steps=0
    open_list      = deque() ############ HINT
    in_open_list   = numpy.full(grid_map.shape, False)
    in_closed_list = numpy.full(grid_map.shape, False)
    distances      = numpy.full(grid_map.shape, sys.maxint)
    parent_nodes   = numpy.full((grid_map.shape[0], grid_map.shape[1], 2), -1)

    [r,c] = [start_r, start_c]
    open_list.append([start_r, start_c])
    in_open_list[start_r, start_c] = True
    distances   [start_r, start_c] = 0

    while len(open_list) > 0 and [r,c] != [goal_r, goal_c]:
        [r,c] = open_list.popleft()  ######## HINT

        in_closed_list[r,c] = True
        neighbors = [[r+1, c],  [r,c+1],  [r-1, c],  [r,c-1]]
        dist = distances[r,c] + 1
        for [nr,nc] in neighbors:
            if grid_map[nr,nc] > 40 or grid_map[nr,nc] < 0 or in_closed_list[nr,nc]:
                continue
            if dist < distances[nr,nc]:
                distances[nr,nc]    = dist
                parent_nodes[nr,nc] = [r,c]
            if not in_open_list[nr,nc]:
                in_open_list[nr,nc] = True
                open_list.append([nr,nc])
            execution_steps += 1

    if [r,c] != [goal_r, goal_c]:
        print "Cannot calculate path by Breadth First Search:'("
        return []
    print "Path calculated after " + str(execution_steps) + " steps."
    path = []
    while [parent_nodes[r,c][0],parent_nodes[r,c][1]] != [-1,-1]:
        path.insert(0, [r,c])
        [r,c] = parent_nodes[r,c]
    return path

def depth_first_search(start_r, start_c, goal_r, goal_c, grid_map):
    #
    # TODO:
    # Write a breadth first search algorithm to find a path in an occupancy grid map given the start cell
    # [start_r, start_c], the goal cell [goal_r, goal_c] and the map 'grid_map'.
    # Return the set of points of the form [[start_r, start_c], [r1,c1], [r2,c2], ..., [goal_r, goal_c]]
    # If path cannot be found, return an empty tuple []
    #
    execution_steps=0
    open_list      = deque() ############ HINT
    in_open_list   = numpy.full(grid_map.shape, False)
    in_closed_list = numpy.full(grid_map.shape, False)
    distances      = numpy.full(grid_map.shape, sys.maxint)
    parent_nodes   = numpy.full((grid_map.shape[0], grid_map.shape[1], 2), -1)

    [r,c] = [start_r, start_c]
    open_list.append([start_r, start_c])
    in_open_list[start_r, start_c] = True
    distances   [start_r, start_c] = 0

    while len(open_list) > 0 and [r,c] != [goal_r, goal_c]:
        [r,c] = open_list.pop()  ######## HINT

        in_closed_list[r,c] = True
        neighbors = [[r+1, c],  [r,c+1],  [r-1, c],  [r,c-1]]
        dist = distances[r,c] + 1
        for [nr,nc] in neighbors:
            if grid_map[nr,nc] > 40 or grid_map[nr,nc] < 0 or in_closed_list[nr,nc]:
                continue
            if dist < distances[nr,nc]:
                distances[nr,nc]    = dist
                parent_nodes[nr,nc] = [r,c]
            if not in_open_list[nr,nc]:
                in_open_list[nr,nc] = True
                open_list.append([nr,nc])
            execution_steps += 1

    if [r,c] != [goal_r, goal_c]:
        print "Cannot calculate path by Breadth First Search:'("
        return []
    print "Path calculated after " + str(execution_steps) + " steps."
    path = []
    while [parent_nodes[r,c][0],parent_nodes[r,c][1]] != [-1,-1]:
        path.insert(0, [r,c])
        [r,c] = parent_nodes[r,c]
    return path

def generic_callback(req, algorithm):
    [start_x, start_y] = [req.start.pose.position.x, req.start.pose.position.y]
    [goal_x,  goal_y ] = [req.goal.pose.position.x , req.goal.pose.position.y ]
    [zero_x,  zero_y ] = [static_map.info.origin.position.x,static_map.info.origin.position.y]
    [start_c, start_r] = [int((start_x - zero_x)/static_map.info.resolution), int((start_y - zero_y)/static_map.info.resolution)]
    [goal_c , goal_r ] = [int((goal_x  - zero_x)/static_map.info.resolution), int((goal_y  - zero_y)/static_map.info.resolution)]

    if algorithm == 'bfs':
        print("Calculating path by Breadth First Search from " + str([start_x, start_y])+" to "+str([goal_x, goal_y]))
        path = breadth_first_search(start_r, start_c, goal_r, goal_c, grid_map)
    else:
        print("Calculating path by Depth First Search from " + str([start_x, start_y])+" to "+str([goal_x, goal_y]))
        path = depth_first_search(start_r, start_c, goal_r, goal_c, grid_map)
    
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

def callback_bfs(req):
    return generic_callback(req, 'bfs')

def callback_dfs(req):
    return generic_callback(req, 'dfs')

def main():
    global static_map, grid_map
    print "PRACTICE 01 - " + NAME
    rospy.init_node("practice01")
    rospy.Service('/navigation/path_planning/breadth_first_search', GetPlan, callback_bfs)
    rospy.Service('/navigation/path_planning/depth_first_search'  , GetPlan, callback_dfs)
    rospy.wait_for_service('/static_map')
    clt = rospy.ServiceProxy("/static_map", GetMap)
    static_map = clt()
    static_map = static_map.map
    grid_map   = numpy.asarray(static_map.data)
    grid_map   = numpy.reshape(grid_map, (static_map.info.height, static_map.info.width))
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
