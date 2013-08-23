#ifndef OJTUTORIAL_H
#define OJTUTORIAL_H

#include <jaus.h>
#include <openJaus.h>
#include <string>
#include "common.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"

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
	//alex_do
	ros::NodeHandle *nodeHandle;
	ros::Publisher *publisher;
	//ethan do
	ros::Subscriber *subscriber;
	int numReceived;
	int numReturned;
} TutorialData;

class ROSComponent {
	public:
		TutorialData * data;
		
		void chatterCallback(const std_msgs::Int32::ConstPtr& msg)
		{
			data->oneNumberMessage->number = msg->data;
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
