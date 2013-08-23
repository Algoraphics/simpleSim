#ifndef OJTUTORIAL_H
#define OJTUTORIAL_H

#include <jaus.h>
#include <openJaus.h>
#include <string>
#include "common.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "beginner_tutorials/wrenchData.h"

/**
 Struct that tells who is connected to us, as well as
 the kinds of messages we can send.
 */	
typedef struct {
	// things we can report
	OneNumberMessage oneNumberMessage;
	JausByte instance;
	JausInteger limit;
	JausBoolean running;
	JausMessage reportVelocity;
	JausMessage reportWrench;
	bool gotVelocity;
	bool gotWrench;
	ros::NodeHandle *nodeHandle;
	ros::Publisher *publisher;
	ros::Publisher *wrenchPub;
	ros::Subscriber *wrenchSub;
	ros::Subscriber *velocitySub;
	int numReceived;
	int numReturned;
} TutorialData;

void sendServiceMessage( JausMessage message, OjCmpt cmpt );

class ROSComponent {
	public:
		OjCmpt *cmpt;
		
		void reportVelocity(const std_msgs::Float64::ConstPtr& msg)
		{
			//printf("velocity callback\n");
			//create msg of type JAUS_REPORT_VELOCITY_STATE
			ReportVelocityStateMessage message;
			message = reportVelocityStateMessageCreate();
			//data = (TutorialData *)ojCmptGetUserData(*cmpt);
			//populate with data from msg
			//printf("populating message\n");
			message->velocityXMps = msg->data;
			message->velocityYMps = 0;
			message->velocityZMps = 0;
			message->velocityRmsMps = 0.01;
			message->rollRateRps = 0.0;
			message->pitchRateRps = 0.0;
			message->yawRateRps = 0.0;
			message->rateRmsRps = 0;
			JausMessage txMessage = reportVelocityStateMessageToJausMessage(message);
			TutorialData *data = (TutorialData *)ojCmptGetUserData(*cmpt);
			data->reportVelocity = txMessage;
			data->gotVelocity = true;
		}

		void reportWrench(const beginner_tutorials::wrenchData::ConstPtr& msg)
		{
			//printf("report wrench callback\n");
			ReportWrenchEffortMessage message;
			message = reportWrenchEffortMessageCreate();
			message->propulsiveLinearEffortXPercent = msg->throttle;
			message->resistiveLinearEffortXPercent = msg->brake;
			message->propulsiveRotationalEffortZPercent = msg->steering;
			JausMessage txMessage = reportWrenchEffortMessageToJausMessage(message);
			TutorialData *data = (TutorialData *)ojCmptGetUserData(*cmpt);
			data->reportWrench = txMessage;
			data->gotWrench = true;
		}
};


// this value gets set to the static double in the cpp file
#define TUTORIAL_THREAD_DESIRED_RATE_HZ			50.0

// more values that the CPP file will need

class TutorialComponent {

	public:
		static OjCmpt create(std::string prettyName);
		static void destroy(OjCmpt cmpt);
		static double threadDesiredRate;

		// this is what we do when we're READY
		static void readyState(OjCmpt cmpt);
		// this is what we do when we're STANDBY
		static void standbyState(OjCmpt cmpt);
		// this is what we do when we're INITIALIZING
		static void initState(OjCmpt cmpt);
		// this is what we do when we're SHUTTING DOWN
		static void shutdownState(OjCmpt cmpt);
		// runs until the user presses any button
		static void run(OjCmpt cmpt);

		// when we receive a message, decides what to do
		static void processMessage(OjCmpt cmpt, JausMessage msg);

	private:
		// no need to create these objects...
		TutorialComponent();

};

#endif
