#include "tutorial.h"
#include <iostream>
#include <string>
#include <cassert>
#include <sys/time.h>
#include <sstream>

// These came direct from the waypoint driver example
// Incoming Service Connection Defines
#define TUTORIAL_INC_SC_TIMEOUT_SEC 			1.0		// The timeout between receiving service connection messages
#define TUTORIAL_INC_SC_UPDATE_RATE_HZ 		20.0	// Requested service connection update rate
#define TUTORIAL_INC_SC_PRESENCE_VECTOR 		0xFF	// The GPOS Presence Vector, set this to the fields desired (default = ALL)
#define TUTORIAL_INC_SC_QUEUE_SIZE			1		// The Service Connection Manager's queue size (0 = infinite)
#define SENSOR_COMPONENT				42

OjCmpt TutorialComponent::create(std::string prettyName) {
	OjCmpt result;
	TutorialData *data = NULL;
	JausAddress tutorialAddr;
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
		//ethan undo
		data = (TutorialData *)malloc(sizeof(TutorialData));
		//ethan undo


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

		// our address
		tutorialAddr = ojCmptGetAddress(result);

		//data = (TutorialData *)malloc(sizeof(TutorialData));
		data->oneNumberMessage = oneNumberMessageCreate();
		data->instance = tutorialAddr->instance;
		data->limit = 50000;
		data->running = JAUS_TRUE;
		//ethan undo

		ojCmptSetUserData(result, (void *)data);
	}
	return result;

}

void TutorialComponent::run(OjCmpt cmpt) {
	ojCmptRun(cmpt);
	TutorialData* data = (TutorialData *)ojCmptGetUserData(cmpt);
	while (data->running == JAUS_TRUE) {
		ojSleepMsec(100);
	}
}

void TutorialComponent::destroy(OjCmpt cmpt) {
	TutorialData *data;
	data = (TutorialData *)ojCmptGetUserData(cmpt);

	// release all global messages we allocated earlier in the data
	if (data->oneNumberMessage) {
		oneNumberMessageDestroy(data->oneNumberMessage);
	}

	// finally, release the data that we malloc'd
	free(data);

	return;
}

void TutorialComponent::initState(OjCmpt cmpt) {
	std::cout << "[INITIALIZING]" << std::endl;
	TutorialData *data = NULL;
	data = (TutorialData *)ojCmptGetUserData(cmpt);

	if (data) {
		data->oneNumberMessage->number++;
		JausAddress destination = jausAddressCreate();
		
		// this is the address that we want to send it to
		if (data->instance == 1) {
			destination->instance = 2;
		} else {
			destination->instance = 1;
		}
		destination->component = JAUS_EXPERIMENTAL_TUTORIAL_COMPONENT;
		destination->node = 1;
		destination->subsystem = 1;
		
		data->oneNumberMessage->source = ojCmptGetAddress(cmpt);
		data->oneNumberMessage->destination = destination;
		
		JausMessage txMsg = oneNumberMessageToJausMessage(data->oneNumberMessage);
		ojCmptSendMessage(cmpt, txMsg);
		
		ojCmptSetState(cmpt, JAUS_READY_STATE);
	} else {
		// bad things, man...
	}
}

void TutorialComponent::readyState(OjCmpt cmpt) {
	//std::cout << "[READY]" << std::endl;
	//TutorialData *data = (TutorialData *)ojCmptGetUserData(cmpt);
}

void TutorialComponent::standbyState(OjCmpt cmpt) {
	std::cout << "[STANDBY]" << std::endl;
	
	TutorialData *data = (TutorialData *)ojCmptGetUserData(cmpt);
	if (data) {
		data->oneNumberMessage->number++;
		JausAddress destination = jausAddressCreate();
		
		// this is the address that we want to send it to
		if (data->instance == 1) {
			destination->instance = 2;
		} else {
			destination->instance = 1;
		}
		destination->component = JAUS_EXPERIMENTAL_TUTORIAL_COMPONENT;
		destination->node = 1;
		destination->subsystem = 1;
		
		data->oneNumberMessage->source = ojCmptGetAddress(cmpt);
		data->oneNumberMessage->destination = destination;
		
		JausMessage txMsg = oneNumberMessageToJausMessage(data->oneNumberMessage);
		ojCmptSendMessage(cmpt, txMsg);
		
		if (data->oneNumberMessage->number >= data->limit) {
			ojCmptSetState(cmpt, JAUS_SHUTDOWN_STATE);
		} else {
			ojCmptSetState(cmpt, JAUS_READY_STATE);
		}
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
	TutorialData *data = NULL;

	data = (TutorialData *)ojCmptGetUserData(cmpt);
	//ethan undo

	switch (msg->commandCode) {

		case JAUS_EXPERIMENTAL_ONENUMBER_MESSAGE:
			data->oneNumberMessage = oneNumberMessageFromJausMessage(msg);
			std::cout << "One number received: " << data->oneNumberMessage->number << std::endl;

			//ethan undo

			ojCmptSetState(cmpt, JAUS_STANDBY_STATE);
			break;
			
		default:
			if (msg->commandCode != JAUS_REPORT_HEARTBEAT_PULSE) {
				std::cout << "Received message..." << std::endl;
				std::cout << "Command code=" << jausCommandCodeString(msg->commandCode) << std::endl;
			}

			ojCmptDefaultMessageProcessor(cmpt, msg);


	}

}
