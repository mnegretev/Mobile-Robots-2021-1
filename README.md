# Mobile Robots Course 2021-1 FI, UNAM

Material para el curso de Robots Móviles de la Facultad de Ingeniería, UNAM, Semestre 2020-1

## Requerimientos

* Ubuntu 18.04
* ROS Melodic

## Instalación

Nota: se asume que ya se tiene instalado Ubuntu y ROS.

* $ git clone https://github.com/mnegretev/Mobile-Robots-2021-1
* $ cd Mobile-Robots-2021-1
* $ ./Setup.sh
* $ cd catkin_ws
* $ catkin_make -j2 -l2

Para probar que todo se instaló y compiló correctamente:

* $ cd 
* $ source Mobile-Robots-2021-1/catkin_ws/devel/setup.bash
* $ roslaunch bring_up justina_simple.launch

Si todo se instaló y compiló correctamente, se debería ver un RViz como el siguiente:

<img src="https://github.com/mnegretev/PAPIME_PE112120/blob/master/Media/TestMaps2.png" alt="Star Gazer App" width="300"/>

Y un Gazebo como el siguiente:

<img src="https://github.com/mnegretev/PAPIME_PE112120/blob/master/Media/TestMaps2.png" alt="Star Gazer App" width="300"/>

## Contacto
Dr. Marco Negrete<br>
Profesor Asociado C<br>
Departamento de Procesamiento de Señales<br>
Facultad de Ingeniería, UNAM <br>
[mnegretev.info](http://mnegretev.info)<br>
contact@mnegretev.info<br>
marco.negrete@ingenieria.unam.edu<br>
