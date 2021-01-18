#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-1
# PRACTICE 9 - COLOR SEGMENTATION
#
# Instructions:
# Complete the code to estimate the position of an object 
# given a colored point cloud using color segmentation.
#

import numpy
import cv2
import ros_numpy
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped

NAME = "Torres_Trejo"

def segment_by_color(img_bgr, points):
    #
    # TODO:
    # - Change color space from RGB to HSV.
    #   Check online documentation for cv2.cvtColor function
    # - Determine the pixels whose color is in the color range of the ball.
    #   Check online documentation for cv2.inRange
    # - Calculate the centroid of all pixels in the given color range (ball position).
    #   Check online documentation for cv2.findNonZero and cv2.mean
    # - Calculate the centroid of the segmented region in the cartesian space
    #   using the point cloud 'points'. Use numpy array notation to process the point cloud data.
    #   Example: 'points[240,320][1]' gets the 'y' value of the point corresponding to
    #   the pixel in the center of the image.
    # Return a tuple of the form [img_c, img_r, x, y, z] where:
    # [img_c, img_r] is the centroid of the segmented region in image coordinates.
    # [x,y,z] is the centroid of the segmented region in cartesian coordinate. 
    #

    x = 0
    y = 0
    z = 0

    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)                          # Change Color Space from RGB to HSV 
    
    lowYellow = numpy.array([25, 100, 100])                                     # Low Yellow
    highYellow = numpy.array([30, 255,255])                                     # High Yellow
    mask= cv2.inRange(img_hsv, lowYellow, highYellow)                           # Mask
    
    indice = cv2.findNonZero(mask)                                              # Index 
    meanPoints = cv2.mean(indice)                                               # Centroid in image Coordinates
    
    img_c = meanPoints[0]                                                       # img_c (Centroid in image Coordinates)
    img_r = meanPoints[1]                                                       # img_r (Centroid in image Coordinates)
    
    
    for i in range(len(indice)-1):
        x += points[indice[i][0][1]][indice[i][0][0]][0]
        y += points[indice[i][0][1]][indice[i][0][0]][1]
        z += points[indice[i][0][1]][indice[i][0][0]][2]

    x = x/len(indice)                                                           # x Component of Segmentd Region (Cartesian Coordinate)
    y = y/len(indice)                                                           # y Component of Segmentd Region (Cartesian Coordinate)
    z = z/len(indice)                                                           # z Component of Segmentd Region (Cartesian Coordinate)
    
    
    return[img_c,img_r,x,y,z]
    

def callback_point_cloud(msg):
    global pub_point
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    rgb_arr = arr['rgb'].copy()
    rgb_arr.dtype = numpy.uint32
    r = numpy.asarray((rgb_arr >> 16) & 255, dtype=numpy.uint8)
    g = numpy.asarray((rgb_arr >> 8) & 255, dtype=numpy.uint8)
    b = numpy.asarray(rgb_arr & 255, dtype=numpy.uint8)
    img_bgr = cv2.merge((b,g,r))
    [centroid_x, centroid_y, x, y, z] = segment_by_color(img_bgr, arr)
    p = PointStamped()
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = "kinect_link"
    [p.point.x, p.point.y, p.point.z] = [x,y,z]
    pub_point.publish(p)
    cv2.circle(img_bgr, (int(centroid_x), int(centroid_y)), 20, [0, 255, 0], thickness=3)
    cv2.imshow("Color Segmentation", img_bgr)
    cv2.waitKey(1)

def main():
    global pub_point
    print "PRACTICE 09 - " + NAME
    rospy.init_node("practice09")
    rospy.Subscriber("/kinect/points", PointCloud2, callback_point_cloud)
    pub_point = rospy.Publisher('/detected_object', PointStamped, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

