#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-1
# PRACTICE 10 - PERCEPTRON TRAINING BY GRADIENT RULE
#
# Instructions:
# Complete the code to train a perceptron using the gradient rule.
# Perceptron's output should indicate wheter a given input image
# corresponds to the trained digit or not. 
#

import sys
import numpy
import cv2
import math
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError

NAME = "ARGUELLES_MACOSAY"

def evaluate(weights, image):
    #
    # TODO:
    # Calculate the output of perceptron given an image.
    # Use of numpy.dot is strongly recommended to save execution time.
    # Return perceptron output.
    #   
    t = numpy.dot(weights,image)
    # pasar el proucto punto por una sigmoide 
    #P(t)=1/1+e a la menos t
    output = 1/(1+(numpy.exp(-t)))
    return output

def train_perceptron(weights, images, labels, desired_digit):
    print "Training perceptron for digit " + str(desired_digit) + " with " + str(len(images)) + " images. "
    #
    # TODO:
    # Train the perceptron (array of weights) given a set of training images, a set of labels (corresponding digits)
    # and a desired digit to be trained.
    # Perceptron is an array of floats representing the input weights and threshold
    # Last value in 'weights' corresponds to threshold.
    # You can use gradient rule or perceptron rule, but gradient rule is recommended.
    # Use of numpy.dot, numpy.add, numpy.zeros, numpy.asarray and numpy.linalg.norm is suggested.
    # Return the trained weights.
    #
    tol      = 0.001                                   #Max gradient magnitude to consider a local minimum
    attempts = 5000                                    #Max number of training iterations
    epsilon  = 0.2/len(weights)                        #Constant to ponderate gradient
    weights  = numpy.asarray(weights)                  #Array of perceptron's weights
    inputs   = [numpy.asarray(d+[-1]) for d in images] #Threshold is a weight whose corresponding input is always -1
    # numpy.dot es para el producto punto de dos vectores
    #
    # WHILE |gradient| > tol and attempts > 0 and not rospy.is_shutdown():
    #     Set gradient to vector zero
    #     FOR EACH img IN images:
    #         y_hat = Perceptron's output for image 'img' with the current weights
    #         y     = Desired output (1 if label for img corresponds to the desired digit. 0, otherwise)
    #         g_j   = Gradient term corresponding to input 'img'
    #         gradient = gradient + g_j
    #     weights = weights - epsilon*gradient
    #     attempts = attempts - 1
    #     
    #gradiente es un arreglo de 785 valores resolucion+umbral
    gradient_mag = 2 
    print("magnitud inicial: " + str(gradient_mag))
    print("espera mientras el perceptron aprende...")
    while gradient_mag > tol and attempts > 0 and not rospy.is_shutdown():
        gradient = numpy.zeros(len(weights))
        index=0
        for i in inputs:
            y_hat = evaluate(weights,i)
            y= 1 if labels[index]==desired_digit else 0
            g_j= (y_hat-y)*(y_hat*(1-y_hat))* i
            gradient=gradient+g_j
            index+=1
        weights = weights - epsilon *gradient
        attempts = attempts - 1
        gradient_mag= numpy.linalg.norm(gradient)
    print("intentos restantes = " + str(attempts) + "   final gradient magnitude  = " + str(gradient_mag))
        
    return weights

def load_dataset_digit(file_name):
    #It is assumed the file contains 1000 images with 28x28 grayscale pixels each one.
    print "Loading data set from file " +  file_name
    f_data = [ord(c)/255.0 for c in open(file_name, "rb").read(784000)]
    images = []
    for i in range(1000):
        images.append(f_data[784*i:784*(i+1)])
    print "Loaded " + str(len(images)) + " images."
    return images

def load_dataset_all_digits(folder):
    print "Loading data set from " + folder
    if not folder.endswith("/"):
        folder += "/"
    training_dataset = []
    training_labels  = []
    testing_dataset  = []
    testing_labels   = []
    for i in range(10):
        digit_dataset = load_dataset_digit(folder + "data" + str(i))
        training_dataset += digit_dataset[0:len(digit_dataset)/2]
        training_labels  += [i for j in range(len(digit_dataset)/2)]
        testing_dataset  += digit_dataset[len(digit_dataset)/2:len(digit_dataset)]
        testing_labels   += [i for j in range(len(digit_dataset)/2)]
    return training_dataset, training_labels, testing_dataset, testing_labels

def main():
    print "PRACTICE 10 - " + NAME
    rospy.init_node("practice10")
    rospack = rospkg.RosPack()
    dataset_folder = rospack.get_path("bring_up") + "/handwriting_digits/"
    if rospy.has_param("~dataset_folder"):
        dataset_folder = rospy.get_param("~dataset_folder")
    desired_digit = 0
    if rospy.has_param("~digit"):
        desired_digit = rospy.get_param("~digit")

    training_dataset, training_labels, testing_dataset, testing_labels = load_dataset_all_digits(dataset_folder)
    perceptron = [0 for i in range(784+1)]
    perceptron = train_perceptron(perceptron, training_dataset, training_labels, desired_digit)
    loop = rospy.Rate(10)
    img = testing_dataset[numpy.random.randint(0, 4999)]
    print("")
    print("Press digit key to test perceptron...")
    while not rospy.is_shutdown():
        test_digit = cv2.waitKey(10) - 48
        if test_digit >= 0 and test_digit <= 9:
            img = testing_dataset[test_digit*500 + numpy.random.randint(0, 499)]
            y = evaluate(perceptron, img + [-1])
            print "Perceptron output: " + str(y)
            if y > 0.5:
                print("Image corresponds to trained digit.")
            else:
                print("Image does not correspond to the trained digit.")
        cv2.imshow("Digit", numpy.reshape(numpy.asarray(img, dtype="float32"), (28,28,1)))
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
