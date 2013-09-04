CATVehicle REU at the University of Arizona: ROS-to-JAUS
==========================================

This folder is a collection of components and nodes created on the ROS and
JAUS architectures used to build a bridge for effective bidirectional
communication between the two architectures.

This project contains the following folders:

beginner_tutorials:	
	A ROS stack containing the nodes created and used for the bridge.
lib:				
	A collection of supporting libraries for the JAUS components used in this folder.
libazcar:			
	Another collection of supporting libraries for JAUS, in this case relevant specifically to the car.
libtutorial:		
	Supporting libraries for the tutorial components around which some of the bridges were constructed. May not be used.
oj2torial:			
	The original number-passing JAUS tutorial component, without ROS.
ojComfortMotion:	
	A JAUS component designed to operate an autonomous car in a smooth
	turning path. This component may have make issues since it is
	generally to be run from outside this folder, and such is not
	expected to be built with the rest of the folders in simpleSim.
ojGazeboBridge:		
	The most significant bridge, this component allows for the
	ojComfortMotion component to operate a car simulated in Gazebo.
ojROSComfort:		
	Another bridge component, this is a variant of the ojComfortController
	which performs the same purpose but offloads some computation to ROS.
ojTutorial:			
	A variant of the ojTutorial number-passing JAUS component which is integrated with ROS and accepts input from a ROS publishing node to change the number being passed as well as reporting that number
	to a ROS topic.
ojVehicleSim:		
	A collection of JAUS components that work together to simulate a JAUS ground vehicle subsystem. These components provide a set of services that are commonly used in real ground vehicle
	implementations.

Ownership
-----------------------------
This code is a combination of multiple open source libraries that have been extended
and combined, so ownership becomes confusing. For the purpose of clarity, code that
was created exclusively for the ROS to JAUS project has been marked with HTML style
tags like so: <ROS2JAUS>. Code outside of these tags was created by the writers of
openJAUS or by Sean Whitsitt at the University of Arizona. The code within the tags
was written by Ethan Rabb, with help from project partners Alex Warren and PJ Morley.

The ROS2JAUS tags can be found in the ojTutorial, ojROSComfort, and ojGazeboBridge
components. The ojGazebo bridge is the final working bridge and was almost completely
custom-written for this project.

Target Operating Environment:
-----------------------------

This software has been tested only on an Ubuntu 12.04 system with ROS groovy installed
as well as OpenJAUS. Both ROS and JAUS are designed to be compatible with multiple
platforms, although ROS runs only on Linux systems.

Installation:
-----------------------------

It is first necessary to install both ROS groovy and OpenJAUS. Both are open-source and
installation instructions can be found online, at www.openjaus.com and www.ros.org.

Assuming these are installed, you must also set your ROS_PACKAGE_PATH environment
variable to include the path to both the simpleSim folder and the beginner_tutorials
folder.

At this point, you should be able to "rosmake beginner_tutorials", which will create
executables for the necessary ROS components.

To make the JAUS and Bridge components, navigate to the simpleSim directory and type
these commands:
	mkdir build
	cd build
	cmake ..
	make -j

At this point all executables should be created and you should be ready to test the code.

Testing:
-----------------------------
Any time you would like to test a ROS node, you must run roscore.
Any time you would like to test a JAUS component, you must be running the ojNodeManager
component, which is included within openJAUS.

In general, ROS executables can be run with "rosrun [NAME OF NODE]"
In general, JAUS executables can be run by navigating to the folder for the executable
	(for example, navigating within the ojVehicleSim folder)
and then typing the command "../build/[COMPONENT NAME]/[EXECUTABLE NAME]"
	(for example, "../build/ojVehicleSim/ojVehicleSim" since in this case the component
	 and the executable have the same name)

There are three bridges that can be tested with the code available in this folder. To do
this, you will generally need to run multiple components and nodes. I will outline those
here.

The Tutorial Bridge:
	This bridge will showcase simple bidirectional communication between ROS and JAUS using
	integers. 

	To run the bridge, first run both the ojTutorial and oj2Torial components.

	Next, run the listenerInt ROS node. You should now see that whatever integer the two
	tutorial components are printed is also being printed by the listener. 

	Last, run the talkerInt ROS node. 

	You should now see that the number passing components
	will repeatedly update the number being passed to whatever number is being published 
	by the talker.

The Comfort Bridge:
	This bridge shows rapid interaction with a ROS node and a JAUS component while both are
	working at heavy computation.

	To run the bridge, first run the ojVehicleSim component.

	Next, run the steeringControl ROS node. This will activate the service which will do the
	calculation for controlling the car steering.

	Finally, run the ojROSComfort component. 

	You should see the vehicle simulator enact a
	series of throttle, brake, and steering values that correspond with a smooth turn
	to the right. If you have access to a working ojComfortMotion component, you might want
	to run that node beforehand for comparison.

The Gazebo Bridge:
	This bridge will showcase full multifaceted communication for operating a simulated
	vehicle in the ROS simulator Gazebo using the JAUS comfort controller component.

	To run the bridge, first run the talker and wrenchTalker ROS nodes. These will spoof
	velocity and wrench information from Gazebo, unless you have a working implementation
	of a simulated car in Gazebo, in which case these nodes are not necessary.

	Next, run the ojGazeboBridge component. The bridge will begin passing any messages it
	recieves to the appropriate recipients.

	Finally, run the comfort controller. This can be done most easily with a traditional
	ojComfortController component, but the one included within simpleSim, as discussed,
	may not compile easily. The ojROSComfort component can be used here, so long as you
	run the steeringControl ROS node before running ojROSComfort.

	If you are using a Gazebo simulated vehicle, you should now see it follow the path
	dictated by the comfort controller. If not, the results will not be as clear, but using
	the ROS methods for printing topics or node structures you should be able to see that
	the bridge is publishing data from the comfort controller node to ROS.

More Information
-----------------------------
For more information about this project, consult these websites:
http://www2.engr.arizona.edu/~sprinkjm/research/catvehicle2013/index.php/ROS2JAUS/HomePage
http://www2.engr.arizona.edu/~sprinkjm/research/catvehicle2013/index.php/Ethan/HomePage
http://catvehicle.arizona.edu/

Also, the previously listed home pages for documentation on ROS and openJAUS:
www.ros.org, www.openjaus.com

The paper written on the content of this project is included within the simpleSim folder as well.
