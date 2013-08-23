#include "tutorial.h"
#include <iostream>
#include <string>
#include <cassert>
#include <sys/time.h>
#include <sstream>

// These came direct from the waypoint driver example
// Incoming Service Connection Defines
#define TUTORIAL_INC_SC_TIMEOUT_SEC 	1.0		// The timeout between receiving service connection messages
#define TUTORIAL_INC_SC_UPDATE_RATE_HZ 	20.0	// Requested service connection update rate
#define TUTORIAL_INC_SC_PRESENCE_VECTOR	0xFF	// The GPOS Presence Vector, set this to the fields desired (default = ALL)
#define TUTORIAL_INC_SC_QUEUE_SIZE		1		// The Service Connection Manager's queue size (0 = infinite)
#define SENSOR_COMPONENT				42

//Hardcoded definition for the component to which our messages are sent
#define JAUS_EXPERIMENTAL_MOTION_PROFILE_DRIVER 10

//This function was pulled from another JAUS component for use with ROS2JAUS.
//It allows for the safe use of our service connections.
void sendServiceMessage( JausMessage message, OjCmpt cmpt )
{
	ServiceConnection scList;
	ServiceConnection sc;

	if (ojCmptIsOutgoingScActive(cmpt, message->commandCode)) {
		printf("Our outgoing SC is active...\n");
		scList = ojCmptGetScSendList(cmpt, message->commandCode);
		sc = scList;
		while (sc) {
			jausAddressCopy(message->destination, sc->address);
			message->sequenceNumber = sc->sequenceNumber;
			message->properties.scFlag = JAUS_SERVICE_CONNECTION_MESSAGE;
			ojCmptSendMessage(cmpt, message);
			jausMessageDestroy(message);
			sc = sc->nextSc;
		}

		ojCmptDestroySendList(scList);
	}
}

OjCmpt TutorialComponent::create(std::string prettyName) {
	printf("creating\n");
	OjCmpt result;
	TutorialData *data = NULL;
	// it is unbelieveable that I have to do this...what a HACK
	char szName[256];
	strcpy(szName, prettyName.c_str());
	// now, we create it using the global (groan) methods
	result = ojCmptCreate(szName, JAUS_EXPERIMENTAL_TUTORIAL_COMPONENT, TUTORIAL_THREAD_DESIRED_RATE_HZ);

	if (result == NULL) {
		// something bad happened...
		std::cout << "Error starting controller...aborting." << std::endl;
		return result;
	} else {
//<ROS2JAUS>
		//First spoof the comand line arguments to ROS
		char **argw = (char**)malloc(sizeof(char*)*2);
		char *str = new char[8];
		strcpy(str,"tutorial");
		argw[0]	= str;
		argw[1]	= NULL;
		int one	= 1;
		ros::init(one, argw, "bridge");
		//Initialize the data struct and set its values
		data = (TutorialData *)malloc(sizeof(TutorialData));

		data->gotVelocity = false;
		data->gotWrench = false;

		data->nodeHandle = new ros::NodeHandle; 
		data->wrenchPub	= new ros::Publisher;
		data->wrenchSub = new ros::Subscriber;
		data->velocitySub = new ros::Subscriber;

		*(data->wrenchPub) = data->nodeHandle->advertise<beginner_tutorials::wrenchData>("setWrench", 1000);
		
		//Set up service connections
		ojCmptAddService(result, JAUS_PRIMITIVE_DRIVER);
		ojCmptAddService(result, JAUS_VELOCITY_STATE_SENSOR);
		ojCmptAddService(result, JAUS_EXPERIMENTAL_MOTION_PROFILE_DRIVER);	
		ojCmptAddServiceOutputMessage(result, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_WRENCH_EFFORT, 0xFF);
		ojCmptAddServiceOutputMessage(result, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_COMPONENT_STATUS, 0xFF);
		ojCmptAddServiceOutputMessage(result, JAUS_EXPERIMENTAL_MOTION_PROFILE_DRIVER, JAUS_REPORT_COMPONENT_STATUS, 0xFF);
		ojCmptAddServiceOutputMessage(result, JAUS_VELOCITY_STATE_SENSOR, JAUS_REPORT_VELOCITY_STATE, 0xFF);
		ojCmptAddSupportedSc(result, JAUS_REPORT_WRENCH_EFFORT);
		ojCmptAddSupportedSc(result, JAUS_REPORT_COMPONENT_STATUS);
		ojCmptAddSupportedSc(result, JAUS_REPORT_VELOCITY_STATE);
//</ROS2JAUS>
		// Register function callback for the process message state
		ojCmptSetMessageProcessorCallback(result, TutorialComponent::processMessage);
		// Register function callback for the standby state
		ojCmptSetStateCallback(result, JAUS_STANDBY_STATE, TutorialComponent::standbyState);
		// Register function callback for the ready state
		ojCmptSetStateCallback(result, JAUS_READY_STATE, TutorialComponent::readyState);
		// Register function callback for the initialize state
		ojCmptSetStateCallback(result, JAUS_INITIALIZE_STATE, TutorialComponent::initState);
		// Register function callback for the shutdown state
		ojCmptSetStateCallback(result, JAUS_SHUTDOWN_STATE, TutorialComponent::shutdownState);
		ojCmptSetState(result, JAUS_INITIALIZE_STATE);

		data->running = JAUS_TRUE;

		ojCmptSetUserData(result, (void *)data);
	}
	return result;

}

void TutorialComponent::run(OjCmpt cmpt) {
	std::cout << "[RUNNING]" << std::endl;
	ojCmptRun(cmpt);
//<ROS2JAUS>
	ROSComponent roscomp;
	roscomp.cmpt = &cmpt;
	TutorialData* data = (TutorialData *)ojCmptGetUserData(cmpt);
	*(data->velocitySub) = data->nodeHandle->subscribe<std_msgs::Float64>("reportVelocity", 1000, &ROSComponent::reportVelocity, &roscomp);
	*(data->wrenchSub) = data->nodeHandle->subscribe<beginner_tutorials::wrenchData>("reportWrench", 1000, &ROSComponent::reportWrench, &roscomp);
	while (data->running == JAUS_TRUE) {
		ros::spinOnce();
//</ROS2JAUS>
	}
}

void TutorialComponent::destroy(OjCmpt cmpt) {
	printf("destroy State\n");
	TutorialData *data;
	data = (TutorialData *)ojCmptGetUserData(cmpt);

	// finally, release the data that we malloc'd
	free(data);

	return;
}

void TutorialComponent::initState(OjCmpt cmpt) {
	std::cout << "[INITIALIZING]" << std::endl;
	TutorialData *data = NULL;
	data = (TutorialData *)ojCmptGetUserData(cmpt);

	if (data) {
		ojCmptSetState(cmpt, JAUS_READY_STATE);
	} else {
		// bad things, man...
	}
}

void TutorialComponent::readyState(OjCmpt cmpt) {
	std::cout << "[READY]" << std::endl;
	TutorialData *data = (TutorialData *)ojCmptGetUserData(cmpt);
//<ROS2JAUS>
	//Check if velocity or wrench state data has been recieved from ROS. 
	//If so, call our service connections to send it.
	ros::spinOnce();
	if (data->gotVelocity) {
		sendServiceMessage(data->reportVelocity, cmpt);
	}	

	ros::spinOnce();
	if (data->gotWrench) {
		sendServiceMessage(data->reportWrench, cmpt);
	}
//</ROS2JAUS>
}

void TutorialComponent::standbyState(OjCmpt cmpt) {
	std::cout << "[STANDBY]" << std::endl;
	TutorialData *data = (TutorialData *)ojCmptGetUserData(cmpt);
	if (data) {
		ojCmptSetState(cmpt, JAUS_READY_STATE);
	} else {
		// bad stuff
	}
}

void TutorialComponent::shutdownState(OjCmpt cmpt) {
	std::cout << "[SHUTDOWN]" << std::endl;
	TutorialData *data = (TutorialData *)ojCmptGetUserData(cmpt);
	if (data) {
		data->running = JAUS_FALSE;
	}
}

void TutorialComponent::processMessage(OjCmpt cmpt, JausMessage msg) {
//<ROS2JAUS>
	TutorialData *data = NULL;
	data = (TutorialData *)ojCmptGetUserData(cmpt);
	SetWrenchEffortMessage wrMsg;
	beginner_tutorials::wrenchData rosWrenchMsg;

	switch (msg->commandCode) {

		case JAUS_SET_WRENCH_EFFORT:
			//If we recieve a set wrench message from someone, publish the contents of the message to ROS
			wrMsg = setWrenchEffortMessageFromJausMessage(msg);
			rosWrenchMsg.throttle = wrMsg->propulsiveLinearEffortXPercent;
			rosWrenchMsg.brake = wrMsg->resistiveLinearEffortXPercent;
			rosWrenchMsg.steering = wrMsg->propulsiveRotationalEffortZPercent;
			data->wrenchPub->publish(rosWrenchMsg);

		case JAUS_REQUEST_COMPONENT_CONTROL:
			//Hardcode an outgoing address, only necessary if the message doesn't provide a source
			JausAddress addr;
			addr = jausAddressCreate();
			addr->subsystem = 1;
			addr->node = 1;
			addr->component = JAUS_EXPERIMENTAL_MOTION_PROFILE_DRIVER;
			addr->instance = 1;
			
			//Spoof a message to confirm control of the primitive driver	
			ConfirmComponentControlMessage confirmComponentControl; 
			confirmComponentControl = confirmComponentControlMessageCreate();
			jausAddressCopy(confirmComponentControl->source, ojCmptGetAddress(cmpt));
			jausAddressCopy(confirmComponentControl->destination, addr);
			confirmComponentControl->responseCode = JAUS_CONTROL_ACCEPTED;

			JausMessage confirmControl;
			confirmControl = confirmComponentControlMessageToJausMessage(confirmComponentControl);
			ojCmptSendMessage(cmpt, confirmControl);
			
			//Spoof a message to report the status of the component
			ReportComponentStatusMessage reportComponentStatus;
			reportComponentStatus = reportComponentStatusMessageCreate();
			reportComponentStatus->primaryStatusCode = JAUS_READY_STATE;
			jausAddressCopy(reportComponentStatus->source, ojCmptGetAddress(cmpt));
			jausAddressCopy(reportComponentStatus->destination, addr);
			
			JausMessage reportStatus;
			reportStatus = reportComponentStatusMessageToJausMessage(reportComponentStatus);
			ojCmptSendMessage(cmpt, reportStatus);


			ojCmptDefaultMessageProcessor(cmpt, msg);
//</ROS2JAUS>
		default:
			//if we recieve an unfamiliar message, print its command code
			if (msg->commandCode != JAUS_REPORT_HEARTBEAT_PULSE && msg->commandCode != JAUS_CREATE_SERVICE_CONNECTION) {
				std::cout << "Received message..." << std::endl;
				std::cout << "Command code=" << jausCommandCodeString(msg->commandCode) << std::endl;
			}

			ojCmptDefaultMessageProcessor(cmpt, msg);


	}

}
