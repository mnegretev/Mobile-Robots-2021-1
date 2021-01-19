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

NAME = "ARIAS_PELAYO"

def segment_by_color(img_bgr, points):
    #
    # TODO:
    # - Change color space from RGB to HSV.                                                         -LISTO
    #   Check online documentation for cv2.cvtColor function
    # - Determine the pixels whose color is in the color range of the ball.
    #   Check online documentation for cv2.inRange                                                  -LISTO
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
    
    image = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV ) #se convierte a un espacio de color HSV
    #hsv, azul, verde, rojo y amarillo

    # definicion de color azul en HSV
    #lower_blue = numpy.array([110,50,50])
    #upper_blue = numpy.array([130,255,255])

    # definicion de verde en HSV
    #lower_green = numpy.array([36,0,0])
    #upper_green = numpy.array([70,255,255])

    # definicion de rojo en HSV
    #lower_red = numpy.array([110,50,50])
    #upper_red = numpy.array([130,255,255])

    # definicion de amarillo en HSV
    lower_yellow = numpy.array([20,100,100], numpy.uint8)
    upper_yellow = numpy.array([30,255,255], numpy.uint8)

    # define range of orange color in HSV
    #lower_orange = numpy.array([10,100,20], numpy.uint8)
    #upper_orange = numpy.array([25,255,255], numpy.uint8)

    # Threshold de la imagen HSV para obtener solo el color elegido
    mask = cv2.inRange(image, lower_yellow, upper_yellow) #mascara o imagen binaria, le pasamos el color elegido

    #obtener indices de los pixeles blancos de la mascara
    #indices = cv2.findNonZero()#lista con indice correspondientes que no son cero
    img_c,img_r,_,_= cv2.mean(cv2.findNonZero(mask))

    #imprime los puntos
    x,y,z,_= points[int(img_r)][int(img_c)]

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
