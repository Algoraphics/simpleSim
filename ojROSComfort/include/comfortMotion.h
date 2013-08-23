#ifndef OJVEHICLECAPTURE_H
#define OJVEHICLECAPTURE_H

#include <jaus.h>
#include <openJaus.h>
#include <string>
#include "common.h"
#include "setMotionProfileMessage.h"
#include "reportCurvatureMessage.h"
#include "motionCommand.h"
#include "ros/ros.h"
#include "beginner_tutorials/steeringControl.h"
#include "beginner_tutorials/savePath.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include <sys/time.h>
#include <math.h>
//#include "queryCurvatureMessage.h"
//#include "reportCurvatureMessage.h"

// path types
#define PATH_RIGHT_TURN		0
#define PATH_LEFT_TURN		1
#define PATH_SIN			2
#define PATH_STRAIGHT		3
#define path_type			PATH_RIGHT_TURN

// LOD types
#define LOD_PARKING_LOT		0
#define LOD_STANDARD		1
#define lod_type			LOD_STANDARD
// 4167 for pl, 17802 for standard
#define LOD_LENGTH 4167				// number of points in the line of death

#define MAX_TIRE_ANGLE (35.0*JAUS_PI/180.0)	// radians
#define APPROX_ARC1_LENGTH 15			// meters
#define APPROX_ARC2_LENGTH 15			// meters
#define APPROX_ARC3_LENGTH 10			// meters
#define APPROX_PATH_LENGTH (APPROX_ARC1_LENGTH+APPROX_ARC2_LENGTH+APPROX_ARC3_LENGTH)
#define PATH_STEP 0.01					// meters
#define PATH_AMPLITUDE 5				// meters
#define PATH_FREQUENCY (JAUS_PI/40)		// radians
#define MAX_ACCELERATION 10				// meters per seconds squared
#define PI JAUS_PI
#define BRAKING_LIMIT 0.5
#define ACCELERATION_LIMIT 0.20
#define MIN_VELOCITY 0.25
#define MAX_VELOCITY 10
#define OMEGA_GAIN 10

#define DRIVE 127
#define NEUTRAL 128

/**
 Struct that tells who is connected to us, as well as
 the kinds of messages we can send.
 */
typedef struct {
	// things we can report
	SetMotionProfileMessage setMotionProfile;
	ReportWrenchEffortMessage reportWrench;
	ReportVelocityStateMessage reportVss;
	ReportComponentStatusMessage controllerStatus;
	SetDiscreteDevicesMessage discreteDevices;

	// things we can order other people around with
	SetWrenchEffortMessage setWrenchEffort;
	SetTravelSpeedMessage setSpeed;

	JausTime motionExpireTime;

	// service connection IDs for the connected components
	int pdWrenchSc; // primitive driver wrench
	int pdStatusSc; // primitive driver status
	int mpdCurvatureSc;
	int mpdStatusSc;
	int vssSc;

	JausState pdState; // state of the primitive driver
	JausState mpdState;
	JausAddress pdAddress; // addr of the primitive driver
	JausAddress mpdAddress;
	JausAddress vssAddress; // addr of the vss

	JausBoolean inControl; // true if we are in control
	JausBoolean requestControl; // true if someone has requested control
	double last_message;
	int vssReceived;
	int wrenchReceived;

//<ROS2JAUS>
	ros::NodeHandle *rosServiceNode;
	ros::ServiceClient *rosPathClient;
	ros::ServiceClient *rosSteerClient;
	ros::Publisher *rosPublisher;
//</ROS2JAUS>

	// vehicle state
	double x_pos; // meters
	double y_pos; // meters
	double heading; // radians
	double velocity; // m/s
	double tire_angle; // radians
	double wangle; // percent
	double path_x[(int)(APPROX_PATH_LENGTH / PATH_STEP)];
	double path_y[(int)(APPROX_PATH_LENGTH / PATH_STEP)];
	int path_index;
	double lod_x[LOD_LENGTH];
	double lod_y[LOD_LENGTH];

	unsigned int gear;

	// debug
	double actual_x[(int)(APPROX_PATH_LENGTH / PATH_STEP)];
	double actual_y[(int)(APPROX_PATH_LENGTH / PATH_STEP)];
	double actual_acceleration[(int)(APPROX_PATH_LENGTH / PATH_STEP)];
	double actual_velocity[(int)(APPROX_PATH_LENGTH / PATH_STEP)];
	double actual_tire_angle[(int)(APPROX_PATH_LENGTH / PATH_STEP)];
	double actual_propulsive[(int)(APPROX_PATH_LENGTH / PATH_STEP)];
	double actual_resistive[(int)(APPROX_PATH_LENGTH / PATH_STEP)];
	double actual_tire_angle_percent[(int)(APPROX_PATH_LENGTH / PATH_STEP)];
	int debug_index;

	int published_data;
} VcData;


// this value gets set to the static double in the cpp file
#define VC_THREAD_DESIRED_RATE_HZ			50.0

// more values that the CPP file will need

class VehicleController {

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
		// runs until the user presses any button
		static void run(OjCmpt cmpt);

		static void manageControl(OjCmpt cmpt);

		static void resume(OjCmpt cmpt);
		static void save_data(char *filename, double *data, int length);
		static void load_data(char *filename, double *data);

		static void steering_controller(double *omega, double *delta_d, int *path_index, double x, double y, double velocity, double heading, double tire_angle, double *path_x, double *path_y, int flag, VcData *data);
		static double fromAngleToAngle(double from, double to);
		static double wrapTo2Pi(double angle);
		static double wrapToPi(double angle);

		static void comfort_controller(double *acceleration, int path_index, double x, double y, double velocity, double heading, double tire_angle, double *path_x, double *path_y, double *lod_x, double *lod_y, VcData *data);
		static int indexClosest(double *array, double value, int length);

		// when we receive a message, decides what to do
		static void processMessage(OjCmpt cmpt, JausMessage msg);

	private:
		// no need to create these objects...
		VehicleController();

};

#endif
