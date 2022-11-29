g_compensation
=========================================

* This repository contains the **g_compensator** ROS node.

* The node subscribes to a [Wrenched Stamped topic](http://docs.ros.org/jade/api/geometry_msgs/html/msg/WrenchStamped.html) performes a non-dynamical gravitation compensation and publishes another [Wrenched Stamped topic](http://docs.ros.org/jade/api/geometry_msgs/html/msg/WrenchStamped.html)

* The correction is based on the parameter **mass** in kg, **com_frame** (center of mass) as tf-frame id, **gravity_frame** (world frame) as tf-frame id with gravity pointing in negative z-direction with g=9.81. All parameters are to be set on the [parameter-server](http://wiki.ros.org/Parameter%20Server)

* All calculations are based on [PyKDL](http://docs.ros.org/diamondback/api/kdl/html/python/) and [tf2](http://wiki.ros.org/tf2)

* Some manipulators, like the KUKA iiwa or the Franka Emika Panda, publish the wrench data of their force-torque sensor in the opposite convention. I.e., they publish the wrench with which the environment acts on the manipulator. To convert such a signal into the more usual definition (the wrench with which the manipulator acts on the environment), the parameter **negate_wrench** can be set to `True` (default: `False`).

* A rosservice */tare* can be called to tare in the current pose.

## Acknowledgements
This project is a result of the LIAA project.
http://www.project-leanautomation.eu/

![LIAA](http://www.project-leanautomation.eu/fileadmin/img/LIAALogo/Logo_LIAA.png "LIAA")

![EC](http://www.project-leanautomation.eu/typo3temp/pics/b3ba71db31.jpg "EC")

LIAA received funding from the European Union’s Seventh Framework Programme for research, technological development and demonstration under grant agreement no. 608604.

Project runtime: 02.09.2013 – 31.08.2017.