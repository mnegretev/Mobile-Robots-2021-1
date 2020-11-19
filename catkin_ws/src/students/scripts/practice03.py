#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-1
# PRACTICE 3 - INFLATION AND COST MAPS
#
# Instructions:
# Write the code necesary to inflate a map given an inflation radius, and to
# get a cost map given a cost radius.
# Consider the map as a bidimensional array with free and occupied cells:
# [[ 0 0 0 0 0 0]
#  [ 0 X 0 0 0 0]
#  [ 0 X X 0 0 0]
#  [ 0 X X 0 0 0]
#  [ 0 X 0 0 0 0]
#  [ 0 0 0 X 0 0]]
# Where occupied cells 'X' have a value of 100 and free cells have a value of 0.
# In this example map[1][1] has a value of 100 and map[1][2] has a value of 0.
#
# Consider the cost map as a bidimensional array where free cells have a value indicating
# how near they are to the nearest occupied cell:
# [[ 3 3 3 2 2 1]
#  [ 3 X 3 3 2 1]
#  [ 3 X X 3 2 1]
#  [ 3 X X 3 2 2]
#  [ 3 X 3 3 3 2]
#  [ 3 3 3 X 3 2]]
# 
#

import sys
import rospy
import numpy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from nav_msgs.srv import GetMapResponse
from nav_msgs.srv import GetMapRequest

NAME = "NAJERA_GARCIA_EDUARDO"
static_map = None

def get_inflated_map(static_map, r):
    print("Inflating map by " + str(r) + " cells")
    inflated = numpy.copy(static_map)
    [height, width] = static_map.shape
    #
    # TODO:
    # Write the code necessary to inflate the obstacles in the map a radius
    # given by 'r'
    # Map is given in 'static_map' as a bidimensional numpy array.
    # Consider as occupied cells all cells with an occupation value greater than 50
    #
    
    for i in range(0, height-1):
        for j in range(0, width-1):
            if  static_map[i, j] > 50:
                for k1 in range(i-r, i+r):
                    for k2 in range(j-r, j+r):
                        inflated[k1, k2] = 100
    
    return inflated

def get_cost_map(static_map, cr):
    print "Calculating cost map with " +str(cr) + " cells"
    cost_map = numpy.copy(static_map)
    [height, width] = static_map.shape
    #
    # TODO:
    # Write the code necessary to calculate a cost map of the give map.
    # Cost must be calculated a number of 'cr' cells around the occupied space.
    # Cost must increase when distance to obstacles decreases, and must be zero for all
    # cells farther than 'cr' cells from the obstacles. 
    # Map is given in 'static_map' as a bidimensional numpy array.
    # Consider as occupied cells all cells with an occupation value greater than 50
    #

    for i in range(0,height-1):
	for j in range(0,width-1):
	    if static_map[i,j]>50:
		for k1 in range(-cr,cr):
		    for k2 in range(-cr,cr):
			c = cr+1-max(abs(k1),abs(k2))
			cost_map[i+k1, j+k2] = max(c, cost_map[i+k1, j+k2])

    return cost_map

def callback_inflated_map(req):
    global static_map, inflation_radius
    grid = numpy.asarray(static_map.data, dtype='int')
    grid = numpy.reshape(grid, (static_map.info.height, static_map.info.width))
    r = int(inflation_radius/static_map.info.resolution)
    inflated_map = get_inflated_map(grid, r)
    resp = GetMapResponse()
    resp.map = OccupancyGrid()
    resp.map.info = static_map.info
    resp.map.data = numpy.ravel(numpy.reshape(inflated_map, (len(static_map.data), 1)))
    return resp

def callback_cost_map(req):
    global static_map, cr
    grid = numpy.asarray(static_map.data, dtype='int')
    grid = numpy.reshape(grid, (static_map.info.height, static_map.info.width))
    cost_cells = int(cr/static_map.info.resolution)
    cost_map = get_cost_map(grid, cost_cells)
    resp = GetMapResponse()
    resp.map = OccupancyGrid()
    resp.map.info = static_map.info
    resp.map.data = numpy.ravel(numpy.reshape(cost_map, (len(static_map.data), 1)))
    return resp
    
def main():
    global static_map, inflation_radius, cr
    print "PRACTICE 03 - " + NAME
    rospy.init_node("practice03")
    rospy.Service('/inflated_map', GetMap, callback_inflated_map)
    rospy.Service('/cost_map', GetMap, callback_cost_map)
    pub_inflated = rospy.Publisher("/inflated_map", OccupancyGrid, queue_size=10)
    rospy.wait_for_service('/static_map')
    clt_static_map = rospy.ServiceProxy("/static_map", GetMap)
    static_map     = clt_static_map()
    static_map     = static_map.map
    loop = rospy.Rate(10)
    
    counter = 0
    cr = 0.5
    inflation_radius = 0.3
    while not rospy.is_shutdown():
        if counter == 0:
            if rospy.has_param("/navigation/path_planning/cr"):
                cr = rospy.get_param("/navigation/path_planning/cr")
            if rospy.has_param("/navigation/path_planning/inflation_radius"):
                new_inflation_radius = rospy.get_param("/navigation/path_planning/inflation_radius")
                if new_inflation_radius != inflation_radius:
                    inflation_radius = new_inflation_radius
                    pub_inflated.publish(callback_inflated_map(GetMapRequest()).map)
        counter = (counter + 1) % 10
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
