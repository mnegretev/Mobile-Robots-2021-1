#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sound_play.msg import SoundRequest

NAME = "MENDOZA_TOLEDO_OSCAR"
pub_robot_sound = None
pub_direccion = None

#Estados
"""
Estado 0: Espera de Orden (Si llega orden ejecuta movimiento)
Estado 1: Ejecutando Orden (Si llega una orden la ignora)
Estado 2: Orden terminada (Dice Llegue!, regresa a E0)
"""

def estado0(orden):
    global pub_direccion
    direccion = PoseStamped()
    if orden == "ROBOT GO TO THE BEDROOM":
        direccion.pose.position.x = 7
        direccion.pose.position.y = 0
        direccion.pose.orientation.w = 1
        pub_direccion.publish(direccion)
        print("FP-En camino a BEDROOM")
    elif orden == "ROBOT GO TO THE LIVINGROOM":
        direccion.pose.position.x = 8
        direccion.pose.position.y = 7
        direccion.pose.orientation.w = 1
        pub_direccion.publish(direccion)
        print("FP-En camino a LIVINGROOM")
    elif orden == "ROBOT GO TO THE CORRIDOR":
        direccion.pose.position.x = 3
        direccion.pose.position.y = 4
        direccion.pose.orientation.w = 1
        pub_direccion.publish(direccion)
        print("FP-En camino a CORRIDOR")
    elif orden == "ROBOT GO TO THE KITCHEN":
        direccion.pose.position.x = 3
        direccion.pose.position.y = 10
        direccion.pose.orientation.w = 1
        pub_direccion.publish(direccion)
        print("FP-En camino a KITCHEN")
    elif orden == "ROBOT GO TO THE ENTRANCE":
        direccion.pose.position.x = 3
        direccion.pose.position.y = 0
        direccion.pose.orientation.w = 1
        pub_direccion.publish(direccion)
        print("FP-En camino a ENTRANCE")

def robot_say(texto):
    print("FP-Robot va a decir: " + texto)
    voice = SoundRequest()
    voice.sound    = -3
    voice.command = 1
    voice.volume  = 1.0
    voice.arg     = texto
    pub_robot_sound.publish(voice)

def orden_recibida(orden):
    global estado
    print("FP-Orden recibida, ESTADO: " + str(estado))
    nueva_orden = orden.data

    if estado == 0:
        estado0(nueva_orden);
        robot_say("OKEY. TO INFINITY. AND BEYOND!")
        estado = 1

def status_vel(velocidad):
    global estado
    if(velocidad.linear.x == 0 and estado == 1):
        estado = 0
        robot_say("COMMAND STAR COMMAND. ARRIVE AT THE DESTINATION")

def main():
    global pub_robot_sound, pub_direccion, estado
    print "FINAL PROJECT - " + NAME
    estado = 0
    rospy.init_node("final_project")

    pub_direccion = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    pub_robot_sound = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)

    rospy.Subscriber('/cmd_vel', Twist, status_vel)
    rospy.Subscriber('/recognized',String,orden_recibida)

    loop = rospy.Rate(20)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
