#include "ros/ros.h"
#include "std_msgs/String.h"

#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "sound_play/sound_play.h"

#include <sstream>

ros::Publisher setGoalXYA;
ros::Subscriber PS_heard;

std_msgs::String order;

bool moving = false;
bool speakF = false;
bool newGoal = false;

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  /* code */
  speakF = true;
  ROS_INFO("I heard: [%s]", msg -> data.c_str());
  if(!moving) //Cambiar el nuevo objetivo si no se esta moviendo
  {
    newGoal = true;
    order.data = msg -> data;
  }
}

//Funcion para publicar mensajes tipo PoseStamped necesarios para establecer un punto meta. Basado en el script QtRosNode.cpp

void pubGoalXYA(float goal_x, float goal_y, float goal_a)
{
  geometry_msgs::PoseStamped msg; //Creando la variable de tipo mensaje PoseStamped
  msg.pose.position.x = goal_x;
  msg.pose.position.y = goal_y;
  msg.pose.orientation.w = cos(goal_a/2);
  msg.pose.orientation.w = sin(goal_a/2);
  setGoalXYA.publish(msg);
}

int main(int argc, char **argv) {
  /* code */
  float x_g,y_g,x,y;

  ros::init(argc, argv, "proyectof");

  ros::NodeHandle nh;

  sound_play::SoundClient decir(nh,"/robotsound");

  setGoalXYA = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);
  PS_heard = nh.subscribe("recognized", 1000, chatterCallback);

  tf::TransformListener tf_listener; //Suscriptor para recibir los topicos transform del mapa y del robot

  ros::Rate loop_rate(10);

  decir.say("Hello");

  int count = 0;
  while (ros::ok()) {
    /* code */
    tf::StampedTransform transform; //Transformacion de la posicion del robot con respecto al mapa;

        /* Places
          BEDROOM 5.79, 2.56
          LIVINGROOM 7.4, 4.31
          KITCHEN 4.27, 7.6
          CORRIDOR 3.6, 3.37
          ENTRANCE 2.62, 0.6
        */

    //Conjunto de ordenes
    if(!moving)
    {
      if(newGoal)
      {
        if(order.data == "ROBOT GO TO THE ENTRANCE")
        {
          x_g = 2.62;
          y_g = 0.6;
          if (speakF) {
            decir.say("I am going to the entrance");
            speakF=false;
          }
        }else
        if(order.data == "ROBOT GO TO KITCHEN")
        {
          x_g = 4.27;
          y_g = 7.6;
          if (speakF) {
            decir.say("I am going to the kitchen");
            speakF=false;
          }
        }else
        if(order.data == "ROBOT GO TO THE BEDROOM")
        {
          x_g = 5.79;
          y_g = 2.56;
          if (speakF) {
            decir.say("I am going to the bedroom");
            speakF=false;
          }
        }else
        if(order.data == "ROBOT GO TO THE LIVINGROOM")
        {
          x_g = 7.4;
          y_g = 4.31;
          if (speakF) {
            decir.say("I am going to the livingroom");
            speakF=false;
          }
        }else
        if(order.data == "ROBOT GO TO THE CORRIDOR")
        {
          x_g = 3.6;
          y_g = 3.37;
          if (speakF) {
            decir.say("I am going to the corridor");
            speakF=false;
          }
        }
        pubGoalXYA(x_g,y_g,0);
        moving = true;
        newGoal = false;
      }
    } else
    {
      //Obtener la localizacion del robot
      try
      {
        tf_listener.lookupTransform("/map","/base_link", ros::Time(0), transform);
        x=transform.getOrigin().x();
        y=transform.getOrigin().y();
        ROS_INFO("Robot Position (%f,%f)", x, y);
      }
      catch(tf::TransformException ex)
      {
        ROS_ERROR("ERROR! %s", ex.what());
      }

      float dist = sqrt((x_g-x)*(x_g-x)+(y_g-y)*(y_g-y));

      if(dist<=0.5)
      {
        decir.say("I have arrived");
        moving = false;
      }

      if(speakF)
      {
        decir.say("I am moving already");
        speakF = false;
      }
    }


    //ROS_INFO("[%s]", order.data.c_str());
    ros::spinOnce(); //Para callbacks
    loop_rate.sleep();
  }

  return 0;
}
