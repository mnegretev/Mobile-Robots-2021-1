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

NAME = "rangel_navarro"

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
    xp = 0
    yp = 0
    zp = 0
    n = 0
    epsilon = 0.00001
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    lower = numpy.array([25,50,50])
    upper = numpy.array([67,255,255])
    mask = cv2.inRange(hsv,lower,upper)
    index = cv2.findNonZero(mask)
    
    cent_img = cv2.mean(index)
    
    for i in index:
        r = i[0][0] #index row
	c = i[0][1] #index column 
        
        if (points[c,r][0] > epsilon):
  	 xp += points[c,r][0]
        
	if (points[c,r][1] > epsilon):
  	 yp += points[c,r][1]
        
	if (points[c,r][2] > epsilon):
  	 zp += points[c,r][2]
         n += 1

    x = xp/n
    y = yp/n
    z = zp/n
    img_c = cent_img[0]
    img_r = cent_img[1]
   
    print(img_c,img_r,x,y,z)
    #print('BGR',img_bgr[240,320])
    #print('HVS',hsv[240,320])
    #print("\n")
    #cv2.imshow("hvs",hsv)
    #cv2.imshow("Blanco y Negro", mask)
    #print(cent_img[0],cent_img[1])
    #return [1,1,0,0,0]
    return [img_c, img_r, x,y,z]

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


