#include "comfortMotion.h"
#include <iostream>
#include <string>
#include <cassert>
#include <sys/time.h>
#include "motionCommand.h"

// These came direct from the waypoint driver example
// Incoming Service Connection Defines
#define VC_INC_SC_TIMEOUT_SEC 			1.0		// The timeout between receiving service connection messages
#define VC_INC_SC_UPDATE_RATE_HZ 		20.0	// Requested service connection update rate
#define VC_INC_SC_PRESENCE_VECTOR 		0xFF	// The GPOS Presence Vector, set this to the fields desired (default = ALL)
#define VC_INC_SC_QUEUE_SIZE			1		// The Service Connection Manager's queue size (0 = infinite)
#define SENSOR_COMPONENT				42

#define POSITION_MODE

char vslog[100];
char wexlog[100];
char weylog[100];
char wezlog[100];


OjCmpt VehicleController::create(std::string prettyName) {
	OjCmpt result;
	VcData *data = NULL;
	JausAddress vcAddr;
	// it is unbelieveable that I have to do this...what a HACK
	char szName[256];
	strcpy(szName, prettyName.c_str());
	// now, we create it using the global (groan) methods
	result = ojCmptCreate(szName, JAUS_EXPERIMENTAL_MOTION_PROFILE_DRIVER, VC_THREAD_DESIRED_RATE_HZ);

	if (result == NULL) {
		// something bad happened...
		std::cout << "Error starting controller...aborting." << std::endl;
		return result;
	} else {
		data = (VcData *)malloc(sizeof(VcData));
//<ROS2JAUS>
		printf("initializing ros input args\n");
		char **argw = (char**)malloc(sizeof(char*)*2);
		char *str = (char*)malloc(sizeof(char)*strlen("tutorial"));=
		argw[0] = str;
		argw[1] = NULL;
		int one = 1;
		printf("initializing ros service\n");
		ros::init(one, argw, "steering_control_client");

		data->rosServiceNode = new ros::NodeHandle;
		data->rosPathClient = new ros::ServiceClient;
		data->rosSteerClient = new ros::ServiceClient;
//</ROS2JAUS>		
		// this is the service we provide
		ojCmptAddService(result, JAUS_EXPERIMENTAL_MOTION_PROFILE_DRIVER);
		// these are the messages we can receive while providing this service
		ojCmptAddServiceInputMessage(result, JAUS_EXPERIMENTAL_MOTION_PROFILE_DRIVER,
		                             JAUS_EXPERIMENTAL_SET_MOTION_PROFILE, 0xFF);
		ojCmptAddServiceInputMessage(result, JAUS_EXPERIMENTAL_MOTION_PROFILE_DRIVER,
		                             JAUS_SET_TRAVEL_SPEED, 0xFF);
		// JAUS_REPORT_VELOCITY_STATE
		ojCmptAddServiceOutputMessage(result, JAUS_EXPERIMENTAL_MOTION_PROFILE_DRIVER,
		                              JAUS_REPORT_VELOCITY_STATE, 0xFF);

		// JAUS_EXPERIMENTAL_REPORT_CURVATURE
		ojCmptAddServiceOutputMessage(result, JAUS_EXPERIMENTAL_MOTION_PROFILE_DRIVER,
		                              JAUS_EXPERIMENTAL_REPORT_CURVATURE, 0xFF);
		// tell the world that we support this service connection (I guess?)
		ojCmptAddSupportedSc(result, JAUS_EXPERIMENTAL_REPORT_CURVATURE);
		ojCmptAddSupportedSc(result, JAUS_REPORT_VELOCITY_STATE);

		// Register function callback for the process message state
		ojCmptSetMessageProcessorCallback(result, VehicleController::processMessage);
		// Register function callback for the standby state
		ojCmptSetStateCallback(result, JAUS_STANDBY_STATE, VehicleController::standbyState);
		// Register function callback for the ready state
		ojCmptSetStateCallback(result, JAUS_READY_STATE, VehicleController::readyState);
		// Register function callback for the initialize state
		ojCmptSetStateCallback(result, JAUS_INITIALIZE_STATE, VehicleController::initState);
		ojCmptSetState(result, JAUS_INITIALIZE_STATE);

		// our address
		vcAddr = ojCmptGetAddress(result);

		//initialize ros client component in the data struct
		//printf("instantiating nodehandle and publisher\n");
		*(data->rosPathClient) = data->rosServiceNode->serviceClient<beginner_tutorials::savePath>("save_path");
		*(data->rosSteerClient) = data->rosServiceNode->serviceClient<beginner_tutorials::steeringControl>("steering_control");
		//printf("finished instantiating\n");
		data->setMotionProfile = setMotionProfileMessageCreate();
		data->reportWrench = reportWrenchEffortMessageCreate();
		data->reportVss = reportVelocityStateMessageCreate();
		data->setWrenchEffort = setWrenchEffortMessageCreate();
		data->setSpeed = setTravelSpeedMessageCreate();
		data->discreteDevices = setDiscreteDevicesMessageCreate();
		data->pdWrenchSc = -1;
		data->pdStatusSc = -1;
		data->vssSc = -1;
		data->mpdCurvatureSc = -1;
		data->mpdStatusSc = -1;

		data->discreteDevices->mainPropulsion = JAUS_TRUE;
		data->discreteDevices->mainFuelSupply = JAUS_TRUE;
		data->discreteDevices->powerAuxDevices = JAUS_TRUE;
		data->discreteDevices->parkingBrake = JAUS_FALSE;

		data->motionExpireTime = jausTimeCreate();

		data->pdAddress = jausAddressCreate();
		data->pdAddress->node = 1;
		data->pdAddress->instance = 1;
		data->pdAddress->subsystem = 1;
		data->pdAddress->component = JAUS_PRIMITIVE_DRIVER;
		data->pdState = JAUS_UNDEFINED_STATE;
		data->vssAddress = jausAddressCreate();
		data->vssAddress->subsystem = 1;
		data->vssAddress->node = 1;
		data->vssAddress->instance = 1;
		data->vssAddress->component = JAUS_VELOCITY_STATE_SENSOR;

		data->mpdAddress = jausAddressCreate();
		data->mpdAddress->subsystem = 1;
		data->mpdAddress->node = 1;
		data->mpdAddress->instance = 1;
		data->mpdAddress->component = JAUS_EXPERIMENTAL_MOTION_PROFILE_DRIVER;
		data->mpdState = JAUS_UNDEFINED_STATE;


		data->inControl = JAUS_FALSE;
		data->requestControl = JAUS_TRUE;

		// OK, that's initialized ... now, let's establish service connections

		JausAddress addr = jausAddressCreate();
		addr->subsystem = 1;
		addr->node = 1;
		addr->instance = 1;

		addr->component = JAUS_PRIMITIVE_DRIVER;
		data->pdWrenchSc = ojCmptEstablishSc(result,
		                                     JAUS_REPORT_WRENCH_EFFORT,
		                                     VC_INC_SC_PRESENCE_VECTOR,
		                                     addr,
		                                     VC_INC_SC_UPDATE_RATE_HZ,
		                                     VC_INC_SC_TIMEOUT_SEC,
		                                     VC_INC_SC_QUEUE_SIZE);
		data->pdStatusSc = ojCmptEstablishSc(result,
		                                     JAUS_REPORT_COMPONENT_STATUS,
		                                     VC_INC_SC_PRESENCE_VECTOR,
		                                     addr,
		                                     VC_INC_SC_UPDATE_RATE_HZ,
		                                     VC_INC_SC_TIMEOUT_SEC,
		                                     VC_INC_SC_QUEUE_SIZE);

		// service connections from the MotionProfileDriver
		addr->component = JAUS_EXPERIMENTAL_MOTION_PROFILE_DRIVER;
		data->mpdCurvatureSc = ojCmptEstablishSc(result,
		                       JAUS_EXPERIMENTAL_REPORT_CURVATURE,
		                       VC_INC_SC_PRESENCE_VECTOR,
		                       addr,
		                       VC_INC_SC_UPDATE_RATE_HZ,
		                       VC_INC_SC_TIMEOUT_SEC,
		                       VC_INC_SC_QUEUE_SIZE);
		data->mpdStatusSc = ojCmptEstablishSc(result,
		                                      JAUS_REPORT_COMPONENT_STATUS,
		                                      VC_INC_SC_PRESENCE_VECTOR,
		                                      addr,
		                                      VC_INC_SC_UPDATE_RATE_HZ,
		                                      VC_INC_SC_TIMEOUT_SEC,
		                                      VC_INC_SC_QUEUE_SIZE);


		addr->component = JAUS_VELOCITY_STATE_SENSOR;
		data->vssSc = ojCmptEstablishSc(result,
		                                JAUS_REPORT_VELOCITY_STATE,
		                                VC_INC_SC_PRESENCE_VECTOR,
		                                addr,
		                                VC_INC_SC_UPDATE_RATE_HZ,
		                                VC_INC_SC_TIMEOUT_SEC,
		                                VC_INC_SC_QUEUE_SIZE);
		jausAddressDestroy(addr);
		jausAddressDestroy(vcAddr);

		data->gear = NEUTRAL;

		data->last_message = 0;
		data->vssReceived = 0;
		data->wrenchReceived = 0;

		data->x_pos = 0.0;
		data->y_pos = 0.0;
		data->heading = 0.0;

		data->published_data = 0;
		data->debug_index = 0;

		// create the path that we will attempt to follow
		// SINUSOID
		if (path_type == PATH_SIN) {
			for (int ii = 0; ii < APPROX_PATH_LENGTH / PATH_STEP; ii++) {
				data->path_x[ii] = ii * PATH_STEP;
				data->path_y[ii] = PATH_AMPLITUDE * sin(PATH_FREQUENCY * ii * PATH_STEP);
			}
		}

		// STRAIGHT
		else if (path_type == PATH_STRAIGHT) {
			for (int ii = 0; ii < APPROX_PATH_LENGTH / PATH_STEP; ii++) {
				data->path_x[ii] = ii * PATH_STEP;
				data->path_y[ii] = 0;
			}
		}

		// LEFT TURN
		else if (path_type == PATH_LEFT_TURN) {
			int ii;
			double x_start = 0.0;
			double y_start = 0.0;
			for (ii = 0; ii < APPROX_PATH_LENGTH / PATH_STEP / 3; ii++) {
				data->path_x[ii] = ii * PATH_STEP;
				x_start = data->path_x[ii];
				data->path_y[ii] = 0;
				y_start = data->path_y[ii];
			}
			double x_start_2 = 0.0;
			double y_start_2 = 0.0;
			for (ii = ii; ii < 2 * APPROX_PATH_LENGTH / PATH_STEP / 3; ii++) {
				data->path_x[ii] = ii * PATH_STEP;
				x_start_2 = data->path_x[ii];
				data->path_y[ii] = -5 + sqrt(fabs(25 - (data->path_x[ii] - x_start) * (data->path_x[ii] - x_start)));
				y_start_2 = data->path_y[ii];
			}
			for (ii = ii; ii < APPROX_PATH_LENGTH / PATH_STEP; ii++) {
				data->path_x[ii] = x_start_2;
				data->path_y[ii] = y_start_2 - (ii - 2 * APPROX_PATH_LENGTH / PATH_STEP / 3) * PATH_STEP;
			}
		}

		// RIGHT TURN
		else if (path_type == PATH_RIGHT_TURN) {
			int ii;
			double x_start = 0.0;
			double y_start = 0.0;
			for (ii = 0; ii < APPROX_ARC1_LENGTH / PATH_STEP; ii++) {
				data->path_x[ii] = ii * PATH_STEP;
				x_start = data->path_x[ii];
				data->path_y[ii] = 0;
				y_start = data->path_y[ii];
				printf("ii: %d\tx: %lf\ty: %lf\n", ii, data->path_x[ii], data->path_y[ii]);
			}
			double x_start_2 = 0.0;
			double y_start_2 = 0.0;
			for (ii = ii; ii < APPROX_ARC2_LENGTH / PATH_STEP + APPROX_ARC1_LENGTH / PATH_STEP; ii++) {
				data->path_x[ii] = ii * PATH_STEP;
				x_start_2 = data->path_x[ii];
				data->path_y[ii] = -(-APPROX_ARC2_LENGTH + sqrt(fabs(APPROX_ARC2_LENGTH * APPROX_ARC2_LENGTH - (data->path_x[ii] - x_start) * (data->path_x[ii] - x_start))));
				y_start_2 = data->path_y[ii];
				printf("ii: %d\tx: %lf\ty: %lf\n", ii, data->path_x[ii], data->path_y[ii]);
			}
			for (ii = ii; ii < APPROX_ARC3_LENGTH / PATH_STEP + APPROX_ARC2_LENGTH / PATH_STEP + APPROX_ARC1_LENGTH / PATH_STEP; ii++) {
				data->path_x[ii] = x_start_2;
				data->path_y[ii] = y_start_2 + (ii - APPROX_ARC2_LENGTH / PATH_STEP - APPROX_ARC1_LENGTH / PATH_STEP) * PATH_STEP;
				printf("ii: %d\tx: %lf\ty: %lf\n", ii, data->path_x[ii], data->path_y[ii]);
			}
		}
		printf("path: %d\tarc1: %d\tarc2: %d\tarc3: %d\tstep: %lf\n", APPROX_PATH_LENGTH, APPROX_ARC1_LENGTH, APPROX_ARC2_LENGTH, APPROX_ARC3_LENGTH, PATH_STEP);
		printf("outsize: %lf\n", (APPROX_PATH_LENGTH / PATH_STEP));
		printf("outsize2: %lf\n", (50 / 0.01));

//<ROS2JAUS>
		//publish path to ros
		beginner_tutorials::savePath srv;

		for (int xdex = 0; xdex < (int)(APPROX_PATH_LENGTH / PATH_STEP); ++xdex) {
			srv.request.path.data.push_back(data->path_x[xdex]);
		}

		if (data->rosPathClient->call(srv)) {ROS_INFO("X Service call succeeded");}
		else {ROS_INFO("X Service call succeeded");}
		srv.request.path.data.clear();

		for (int ydex = 0; ydex < (int)(APPROX_PATH_LENGTH / PATH_STEP); ++ydex) {
			srv.request.path.data.push_back(data->path_y[ydex]);
		}

		if (data->rosPathClient->call(srv)) {ROS_INFO("X Service call succeeded");}
		else {ROS_INFO("X Service call succeeded");}

		// start the path at the first index
		data->path_index = 0;

		char test_filename[1000];
		strcpy(test_filename, "outputs/path_x_test2.txt");
		save_data(test_filename, data->path_x, (int)APPROX_PATH_LENGTH / PATH_STEP);
		strcpy(test_filename, "outputs/path_y_test2.txt");
		save_data(test_filename, data->path_y, (int)APPROX_PATH_LENGTH / PATH_STEP);
//<ROS2JAUS>
		char lod_x_filename[100];
		if (lod_type == LOD_PARKING_LOT) {
			strcpy(lod_x_filename, "lod_pl_x.mat");
		} else {
			strcpy(lod_x_filename, "lod_x.mat");
		}
		char lod_y_filename[100];
		if (lod_type == LOD_PARKING_LOT) {
			strcpy(lod_y_filename, "lod_pl_y.mat");
		} else {
			strcpy(lod_y_filename, "lod_y.mat");
		}
		load_data(lod_x_filename, data->lod_x);
		load_data(lod_y_filename, data->lod_y);


		ojCmptSetUserData(result, (void *)data);
	}
	return result;


	fprintf(stderr, "Component Created\n");
}

void VehicleController::load_data(char *filename, double *data) {
	fprintf(stderr, "Load data %s\n", filename);
	FILE *fp;
	fp = fopen(filename, "r");
	int c;
	char value[100];
	char add[2];
	add[1] = '\0';
	strcpy(value, "");
	int index = 0;
	do {
		c = fgetc(fp);
		if (c == ',' || c == EOF) {
			data[index] = atof(value);
			index++;
			strcpy(value, "");
		} else {
			add[0] = c;
			strcat(value, add);
		}
	} while (c != EOF);
	fclose(fp);
}

void VehicleController::save_data(char *filename, double *data, int length) {
	fprintf(stderr, "Save data %s\n", filename);
	FILE *fp;
	fp = fopen(filename, "a");
	for (int ii = 0; ii < length; ii++) {
		fprintf(fp, "%lf", data[ii]);
		if (ii < length - 1) {
			fprintf(fp, ",");
		}
	}
	fclose(fp);
}

void VehicleController::run(OjCmpt cmpt) {
	fprintf(stderr, "Run Component\n");
	ojCmptRun(cmpt);
	while (true) {
		// doing nothing...
		ojSleepMsec(100);
	}
	//getchar();
}

void VehicleController::destroy(OjCmpt cmpt) {
	fprintf(stderr, "Destroy Component\n");
	ReleaseComponentControlMessage releaseControl = NULL;
	JausMessage txMessage;
	JausAddress address;
	VcData *data;
	data = (VcData *)ojCmptGetUserData(cmpt);

	// check all of our active service connections, and quit them
	if (ojCmptIsIncomingScActive(cmpt, data->pdStatusSc)) {
		ojCmptTerminateSc(cmpt, data->pdStatusSc);
	}
	if (ojCmptIsIncomingScActive(cmpt, data->pdWrenchSc)) {
		ojCmptTerminateSc(cmpt, data->pdWrenchSc);
	}
	if (ojCmptIsIncomingScActive(cmpt, data->mpdCurvatureSc)) {
		ojCmptTerminateSc(cmpt, data->mpdCurvatureSc);
	}
	if (ojCmptIsIncomingScActive(cmpt, data->mpdStatusSc)) {
		ojCmptTerminateSc(cmpt, data->mpdStatusSc);
	}

	if (data->inControl) {
		// now we have to give it up...
		releaseControl = releaseComponentControlMessageCreate();
		address = ojCmptGetAddress(cmpt);
		jausAddressCopy(releaseControl->source, address);
		jausAddressDestroy(address);
		jausAddressCopy(releaseControl->destination, data->pdAddress);

		txMessage = releaseComponentControlMessageToJausMessage(releaseControl);
		ojCmptSendMessage(cmpt, txMessage);
		jausMessageDestroy(txMessage);

		releaseComponentControlMessageDestroy(releaseControl);

	}

	// release all global messages we allocated earlier in the data
	if (data->setMotionProfile) {
		setMotionProfileMessageDestroy(data->setMotionProfile);
	}
	if (data->reportWrench) {
		reportWrenchEffortMessageDestroy(data->reportWrench);
	}
	if (data->reportVss) {
		reportVelocityStateMessageDestroy(data->reportVss);
	}
	if (data->setSpeed) {
		setTravelSpeedMessageDestroy(data->setSpeed);
	}
	if (data->setWrenchEffort) {
		setWrenchEffortMessageDestroy(data->setWrenchEffort);
	}

	jausAddressDestroy(data->pdAddress);

	// finally, release the data that we malloc'd
	free(data);

	return;
}

void VehicleController::initState(OjCmpt cmpt) {
	std::cout << "[INITIALIZING]" << std::endl;
	VcData *data = NULL;
	data = (VcData *)ojCmptGetUserData(cmpt);

	if (data) {
		if (data->pdAddress->node == 0) {
			if (ojCmptLookupAddress(cmpt, data->pdAddress)) {
				jausAddressCopy(data->setWrenchEffort->destination, data->pdAddress);
			} else {
				// bad things, man...
			}

			if (ojCmptLookupAddress(cmpt, data->mpdAddress)) {
				jausAddressCopy(data->setMotionProfile->destination, data->mpdAddress);
			} else {
				// bad things, man...
			}
		}

		data->requestControl = JAUS_TRUE;
		manageControl(cmpt);

		ojCmptSetState(cmpt, JAUS_STANDBY_STATE);
	} else {
		// bad things, man...
	}

}

void VehicleController::resume(OjCmpt cmpt) {
	VcData *data = (VcData *)ojCmptGetUserData(cmpt);
	JausMessage resMsg;
	ResumeMessage resume = resumeMessageCreate();

	//jausAddressCopy(resume->destination, data->mpdAddress);
	// transform into jaus generic message
	resMsg = resumeMessageToJausMessage(resume);

	jausAddressCopy(resMsg->destination, data->pdAddress);
	char resAddr[1000];
	jausAddressToString(resMsg->destination, resAddr);
	fprintf(stderr, "Sending resume commands: %s.\n", resAddr);
	ojCmptSendMessage(cmpt, resMsg);
	jausMessageDestroy(resMsg);
}

void VehicleController::readyState(OjCmpt cmpt) {
	VcData *data = NULL;
	data = (VcData *)ojCmptGetUserData(cmpt);
	if (data) {
		// do nothing in the ready state
	} else {
		std::cout << "Problem getting data..." << std::endl;
	}

}

void VehicleController::manageControl(OjCmpt cmpt) {

	RequestComponentControlMessage requestControl = NULL;
	ReleaseComponentControlMessage releaseControl = NULL;
	JausAddress address;
	JausMessage message = NULL;
	VcData *data = (VcData *)ojCmptGetUserData(cmpt);
	if (data->pdState != JAUS_READY_STATE) {
		resume(cmpt);
	}
	if (data->requestControl) {
		fprintf(stderr, "Requesting Control of PD.\n");
		requestControl = requestComponentControlMessageCreate();
		address = ojCmptGetAddress(cmpt);
		jausAddressCopy(requestControl->source, address);
		jausAddressDestroy(address);
		jausAddressCopy(requestControl->destination, data->pdAddress);
		requestControl->authorityCode = ojCmptGetAuthority(cmpt);

		message = requestComponentControlMessageToJausMessage(requestControl);
		ojCmptSendMessage(cmpt, message);
		jausMessageDestroy(message);

		requestComponentControlMessageDestroy(requestControl);
	} else {
		// we are not in control, make sure we release it
		if (data->inControl) {
			// Release Control
			releaseControl = releaseComponentControlMessageCreate();
			address = ojCmptGetAddress(cmpt);
			jausAddressCopy(releaseControl->source, address);
			jausAddressDestroy(address);
			jausAddressCopy(releaseControl->destination, data->pdAddress);

			message = releaseComponentControlMessageToJausMessage(releaseControl);
			ojCmptSendMessage(cmpt, message);
			jausMessageDestroy(message);

			releaseComponentControlMessageDestroy(releaseControl);

			data->inControl = JAUS_FALSE;
		}
	}


}

void VehicleController::standbyState(OjCmpt cmpt) {
	VcData *data = (VcData *)ojCmptGetUserData(cmpt);
	ojCmptSetState(cmpt, JAUS_READY_STATE);
	if (data) {
		if (data->requestControl == JAUS_FALSE) {
			data->requestControl = JAUS_TRUE;
			manageControl(cmpt);
		}
	}
}

#define MAX( A, B ) (A > B ? A : B)
#define MIN( A, B ) (A < B ? A : B)

void VehicleController::processMessage(OjCmpt cmpt, JausMessage msg) {
	ReportComponentStatusMessage reportComponentStatus;
	ConfirmComponentControlMessage confirmComponentControl;
	RejectComponentControlMessage rejectComponentControl;
	ReportVelocityStateMessage reportVss;
	ReportWrenchEffortMessage reportWrench;
	double time_diff;
	double current_time;
	VcData *data = NULL;

	data = (VcData *)ojCmptGetUserData(cmpt);

	switch (msg->commandCode) {
		case JAUS_REPORT_COMPONENT_STATUS:

			reportComponentStatus = reportComponentStatusMessageFromJausMessage(msg);
			if (reportComponentStatus) {
				if (jausAddressEqual(reportComponentStatus->source, data->pdAddress)) {
					data->pdState = reportComponentStatus->primaryStatusCode;
					std::cout << "new pdState = " << data->pdState << std::endl;
				}
				if (jausAddressEqual(reportComponentStatus->source, data->mpdAddress)) {
					data->mpdState = reportComponentStatus->primaryStatusCode;
					std::cout << "new mpdState = " << data->mpdState << std::endl;
				}
				reportComponentStatusMessageDestroy(reportComponentStatus);
				ojCmptDefaultMessageProcessor(cmpt, msg);
			}

			break;
		case JAUS_EXPERIMENTAL_SET_MOTION_PROFILE:
			// do nothing
			break;
		case JAUS_REQUEST_COMPONENT_CONTROL:
			// do nothing
			break;
		case JAUS_CONFIRM_COMPONENT_CONTROL:

			confirmComponentControl = confirmComponentControlMessageFromJausMessage(msg);
			if (confirmComponentControl) {
				if (jausAddressEqual(confirmComponentControl->source, data->pdAddress)) {
					std::cout << "vc: Confirmed control of PD\n" << std::endl;
					data->inControl = JAUS_TRUE;
				}
				if (jausAddressEqual(confirmComponentControl->source, data->mpdAddress)) {
					std::cout << "mpt: Confirmed control of MPD\n" << std::endl;
					data->inControl = JAUS_TRUE;
				}
				confirmComponentControlMessageDestroy(confirmComponentControl);
			}

			break;

		case JAUS_REJECT_COMPONENT_CONTROL:

			rejectComponentControl = rejectComponentControlMessageFromJausMessage(msg);
			if (rejectComponentControl) {
				if (jausAddressEqual(rejectComponentControl->source, data->pdAddress)) {
					std::cout << "vc: Lost control of PD\n" << std::endl;
					data->inControl = JAUS_FALSE;
				}
				if (jausAddressEqual(rejectComponentControl->source, data->mpdAddress)) {
					std::cout << "mpt: Lost control of MPD\n" << std::endl;
					data->inControl = JAUS_FALSE;
				}
				rejectComponentControlMessageDestroy(rejectComponentControl);
			}

			break;

		case JAUS_SET_TRAVEL_SPEED:
			// do nothing
			break;

		case JAUS_REPORT_VELOCITY_STATE:
			fprintf(stderr, "Report Velocity State Received\n");
			data->vssReceived = 1;
			timeval tv;
			gettimeofday(&tv, NULL);
			current_time = tv.tv_sec + tv.tv_usec / 1000000.0;
			time_diff = current_time;
			if (data->last_message != 0) {
				time_diff = current_time - data->last_message;
			}
			data->last_message = current_time;
			reportVss = reportVelocityStateMessageFromJausMessage(msg);
			if (reportVss) {
				reportVelocityStateMessageDestroy(data->reportVss);
				data->reportVss = reportVss;

				// handle all the control stuff!
				if (data->wrenchReceived > 0 && time_diff != current_time) {

					if (data->pdState != JAUS_READY_STATE) {
						//resume(cmpt);
						manageControl(cmpt);
					}

					fprintf(stderr, "pd   :\t%d, %d, %d, %d\n", data->pdState, JAUS_READY_STATE, JAUS_STANDBY_STATE, JAUS_UNDEFINED_STATE);
					fprintf(stderr, "mpd  :\t%d, %d, %d, %d\n", data->mpdState, JAUS_READY_STATE, JAUS_STANDBY_STATE, JAUS_UNDEFINED_STATE);

					if (data->pdState == 5 && data->published_data <= 2) {
						data->published_data = 2;
						if (data->published_data == 2) {
							data->published_data += 1;
							char filename[1000];
							sprintf(filename, "outputs/path_x_%ld.txt", tv.tv_sec);
							save_data(filename, data->path_x, (int)(APPROX_PATH_LENGTH / PATH_STEP));
							strcpy(filename, "");
							sprintf(filename, "outputs/path_y_%ld.txt", tv.tv_sec);
							save_data(filename, data->path_y, (int)(APPROX_PATH_LENGTH / PATH_STEP));
							strcpy(filename, "");
							sprintf(filename, "outputs/actual_x_%ld.txt", tv.tv_sec);
							save_data(filename, data->actual_x, (int)(APPROX_PATH_LENGTH / PATH_STEP));
							strcpy(filename, "");
							sprintf(filename, "outputs/actual_y_%ld.txt", tv.tv_sec);
							save_data(filename, data->actual_y, (int)(APPROX_PATH_LENGTH / PATH_STEP));
							strcpy(filename, "");
							sprintf(filename, "outputs/actual_acceleration_%ld.txt", tv.tv_sec);
							save_data(filename, data->actual_acceleration, (int)(APPROX_PATH_LENGTH / PATH_STEP));
							strcpy(filename, "");
							sprintf(filename, "outputs/actual_propulsive_%ld.txt", tv.tv_sec);
							save_data(filename, data->actual_propulsive, (int)(APPROX_PATH_LENGTH / PATH_STEP));
							strcpy(filename, "");
							sprintf(filename, "outputs/actual_resistive_%ld.txt", tv.tv_sec);
							save_data(filename, data->actual_resistive, (int)(APPROX_PATH_LENGTH / PATH_STEP));
							strcpy(filename, "");
							sprintf(filename, "outputs/actual_velocity_%ld.txt", tv.tv_sec);
							save_data(filename, data->actual_velocity, (int)(APPROX_PATH_LENGTH / PATH_STEP));
							strcpy(filename, "");
							sprintf(filename, "outputs/actual_tire_angle_%ld.txt", tv.tv_sec);
							save_data(filename, data->actual_tire_angle, (int)(APPROX_PATH_LENGTH / PATH_STEP));
							strcpy(filename, "");
							sprintf(filename, "outputs/actual_tire_angle_percent_%ld.txt", tv.tv_sec);
							save_data(filename, data->actual_tire_angle_percent, (int)(APPROX_PATH_LENGTH / PATH_STEP));
							strcpy(filename, "");
							sprintf(filename, "outputs/lod_x_%ld.txt", tv.tv_sec);
							save_data(filename, data->lod_x, LOD_LENGTH);
							strcpy(filename, "");
							sprintf(filename, "outputs/lod_y_%ld.txt", tv.tv_sec);
							save_data(filename, data->lod_y, LOD_LENGTH);
						}
					}

					if (data->pdState == JAUS_READY_STATE) {
						if (data->gear != DRIVE) {
							// if we're not in drive, set it to drive!
							SetDiscreteDevicesMessage discreteDevices = data->discreteDevices;
							discreteDevices->gear = newJausByte(DRIVE);
							JausMessage ddmsg = setDiscreteDevicesMessageToJausMessage(discreteDevices);
							jausAddressCopy(ddmsg->destination, data->pdAddress);
							ojCmptSendMessage(cmpt, ddmsg);
							jausMessageDestroy(ddmsg);
							data->gear = DRIVE;

						} else {

							// control variables
							// I don't think acceleration or omega are used other than when we generate them in the controllers
							// But we'll calculate them anyway.
							double velocity = data->reportVss->velocityXMps;
							double acceleration = (velocity - data->velocity) / time_diff;
							double wangle = data->reportWrench->propulsiveRotationalEffortZPercent;
							double tire_angle = data->reportWrench->propulsiveRotationalEffortZPercent * MAX_TIRE_ANGLE / 100.0;
							double omega = (tire_angle - data->tire_angle) / time_diff;
							double x = data->x_pos;
							double y = data->y_pos;
							double heading = data->heading;
							double L = 2.6187;
							int path_index = data->path_index;
							int start_index = path_index;

							// update vehicle model
							double x_next = x + time_diff * velocity * cos(heading);
							double y_next = y + time_diff * velocity * sin(heading);
							double heading_next = wrapToPi(heading + time_diff * velocity * (1 / L) * sin(tire_angle));
							x = x_next;
							y = y_next;
							heading = heading_next;

							// steering control
							double delta_toss;
							steering_controller(&omega, &delta_toss, &path_index, x, y, velocity, heading, tire_angle, data->path_x, data->path_y, 0, data);

							fprintf(stderr, "wangl:\t%lf\n", data->reportWrench->propulsiveRotationalEffortZPercent);
							fprintf(stderr, "wplxe:\t%lf\n", data->reportWrench->propulsiveLinearEffortXPercent);
							fprintf(stderr, "time :\t%lf\n", time_diff);
							fprintf(stderr, "x    :\t%lf\n", x);
							fprintf(stderr, "y    :\t%lf\n", y);
							fprintf(stderr, "veloc:\t%lf\n", velocity);
							fprintf(stderr, "head :\t%lf\n", heading);
							fprintf(stderr, "omega:\t%lf\n", omega);
							fprintf(stderr, "tirea:\t%lf\n", tire_angle);

							// comfort control
							comfort_controller(&acceleration, path_index, x, y, velocity, heading, tire_angle, data->path_x, data->path_y, data->lod_x, data->lod_y, data);
							//acceleration = MAX_ACCELERATION;

							fprintf(stderr, "accel:\t%lf\n", acceleration);
							fprintf(stderr, "pathi:\t%d\n", path_index);

							// save vehicle model (x, y, and heading)
							data->heading = heading;
							data->x_pos = x;
							data->y_pos = y;
							data->velocity = velocity;
							data->tire_angle = tire_angle;
							data->path_index = path_index;

							// build the set wrench effort message
							SetWrenchEffortMessage msg = setWrenchEffortMessageCreate();

							// omega needs to be scaled by the time for effort mode
#ifdef EFFORT_MODE
							omega = omega / time_diff;
							omega = MIN(MAX(omega / (2 * MAX_TIRE_ANGLE) * 100, -99.0), 99.0);
#endif
							// omega does not need to be scaled for position mode
							// but it does need to be damped
#ifdef POSITION_MODE
							omega = MIN(MAX(omega / MAX_TIRE_ANGLE * 100, -99.0), 99);
							//omega = tire_angle + OMEGA_GAIN * omega * time_diff;
#endif
							msg->propulsiveRotationalEffortZPercent = omega;
							if (acceleration > 0) {
								msg->propulsiveLinearEffortXPercent = acceleration / MAX_ACCELERATION * 100;
								msg->resistiveLinearEffortXPercent = 0.0;
							} else {
								msg->propulsiveLinearEffortXPercent = 0.0;
								msg->resistiveLinearEffortXPercent = -acceleration / MAX_ACCELERATION * 100;
							}

							// debug save
							if (start_index != path_index) {
								if (data->debug_index < (APPROX_PATH_LENGTH / PATH_STEP)) {
									data->actual_x[data->debug_index] = x;
									data->actual_y[data->debug_index] = y;
									data->actual_velocity[data->debug_index] = velocity;
									data->actual_acceleration[data->debug_index] = acceleration;
									data->actual_tire_angle[data->debug_index] = tire_angle;
									data->actual_propulsive[data->debug_index] = data->reportWrench->propulsiveLinearEffortXPercent;
									data->actual_resistive[data->debug_index] = data->reportWrench->resistiveLinearEffortXPercent;
									data->actual_tire_angle_percent[data->debug_index] = wangle;
									data->debug_index += 1;
								}
							}

							// if we're near the end of the path then set our desired velocity and tire angle back to 0
							if (path_index > 0.95 * APPROX_PATH_LENGTH / PATH_STEP) {
								msg->propulsiveRotationalEffortZPercent = 0.0;
								msg->propulsiveLinearEffortXPercent = 0.0;
								msg->resistiveLinearEffortXPercent = 30.0 / BRAKING_LIMIT;
								data->published_data += 1;
								if (data->published_data == 2) {
									char filename[1000];
									sprintf(filename, "outputs/path_x_%ld.txt", tv.tv_sec);
									save_data(filename, data->path_x, (int)(APPROX_PATH_LENGTH / PATH_STEP));
									strcpy(filename, "");
									sprintf(filename, "outputs/path_y_%ld.txt", tv.tv_sec);
									save_data(filename, data->path_y, (int)(APPROX_PATH_LENGTH / PATH_STEP));
									strcpy(filename, "");
									sprintf(filename, "outputs/actual_x_%ld.txt", tv.tv_sec);
									save_data(filename, data->actual_x, (int)(APPROX_PATH_LENGTH / PATH_STEP));
									strcpy(filename, "");
									sprintf(filename, "outputs/actual_y_%ld.txt", tv.tv_sec);
									save_data(filename, data->actual_y, (int)(APPROX_PATH_LENGTH / PATH_STEP));
									strcpy(filename, "");
									sprintf(filename, "outputs/actual_acceleration_%ld.txt", tv.tv_sec);
									save_data(filename, data->actual_acceleration, (int)(APPROX_PATH_LENGTH / PATH_STEP));
									strcpy(filename, "");
									sprintf(filename, "outputs/actual_velocity_%ld.txt", tv.tv_sec);
									save_data(filename, data->actual_velocity, (int)(APPROX_PATH_LENGTH / PATH_STEP));
									strcpy(filename, "");
									sprintf(filename, "outputs/actual_tire_angle_%ld.txt", tv.tv_sec);
									save_data(filename, data->actual_tire_angle, (int)(APPROX_PATH_LENGTH / PATH_STEP));
									strcpy(filename, "");
									sprintf(filename, "outputs/actual_propulsive_%ld.txt", tv.tv_sec);
									save_data(filename, data->actual_propulsive, (int)(APPROX_PATH_LENGTH / PATH_STEP));
									strcpy(filename, "");
									sprintf(filename, "outputs/actual_resistive_%ld.txt", tv.tv_sec);
									save_data(filename, data->actual_resistive, (int)(APPROX_PATH_LENGTH / PATH_STEP));
									strcpy(filename, "");
									sprintf(filename, "outputs/actual_tire_angle_percent_%ld.txt", tv.tv_sec);
									save_data(filename, data->actual_tire_angle_percent, (int)(APPROX_PATH_LENGTH / PATH_STEP));
									strcpy(filename, "");
									sprintf(filename, "outputs/lod_x_%ld.txt", tv.tv_sec);
									save_data(filename, data->lod_x, LOD_LENGTH);
									strcpy(filename, "");
									sprintf(filename, "outputs/lod_y_%ld.txt", tv.tv_sec);
									save_data(filename, data->lod_y, LOD_LENGTH);
								}
								if (velocity == 0 && data->gear != NEUTRAL) {
									// if we're not in park, set it to park!
									SetDiscreteDevicesMessage discreteDevices = data->discreteDevices;
									discreteDevices->gear = newJausByte(NEUTRAL);
									JausMessage ddmsg = setDiscreteDevicesMessageToJausMessage(discreteDevices);
									jausAddressCopy(ddmsg->destination, data->pdAddress);
									ojCmptSendMessage(cmpt, ddmsg);
									jausMessageDestroy(ddmsg);
									data->gear = NEUTRAL;
								}
							}

							msg->propulsiveLinearEffortXPercent *= ACCELERATION_LIMIT;
							msg->resistiveLinearEffortXPercent *= BRAKING_LIMIT;
							fprintf(stderr, "aperc:\t%lf\n", msg->propulsiveLinearEffortXPercent);
							fprintf(stderr, "bperc:\t%lf\n", msg->resistiveLinearEffortXPercent);
							fprintf(stderr, "operc:\t%lf\n", msg->propulsiveRotationalEffortZPercent);
							fprintf(stderr, "addr:\t%d.%d.%d.%d\n", data->pdAddress->subsystem, data->pdAddress->node, data->pdAddress->component, data->pdAddress->instance);

							// send message
							JausMessage txMessage = setWrenchEffortMessageToJausMessage(msg);
							jausAddressCopy(txMessage->destination, data->pdAddress);
							ojCmptSendMessage(cmpt, txMessage);
							jausMessageDestroy(txMessage);
						}
					}
				}
			}
			break;

		case JAUS_REPORT_WRENCH_EFFORT:
			char wrenchAddr[1000];
			jausAddressToString(msg->source, wrenchAddr);
			fprintf(stderr, "Report Wrench Effort Received: %s\n", wrenchAddr);
			data->wrenchReceived = 1;
			reportWrench = reportWrenchEffortMessageFromJausMessage(msg);
			if (reportWrench) {
				data->reportWrench = reportWrench;
			}
			break;

		case JAUS_RESUME:
			if (ojCmptGetState(cmpt) == JAUS_STANDBY_STATE) {
				ojCmptSetState(cmpt, JAUS_READY_STATE);
			} else {
				std::cout << "Cannot resume from non-standby state" << std::endl;
			}
			break;
		case JAUS_EXPERIMENTAL_QUERY_CURVATURE:
			std::cout << "Experimental query curvature..." << std::endl;
			break;
		default:
			if (msg->commandCode != JAUS_REPORT_HEARTBEAT_PULSE) {
				std::cout << "Received message..." << std::endl;
				std::cout << "Command code=" << jausCommandCodeString(msg->commandCode) << std::endl;
			}

			ojCmptDefaultMessageProcessor(cmpt, msg);
	}

}

void VehicleController::steering_controller(double *omega, double *delta_d, int *path_index, double x, double y, double velocity, double heading, double tire_angle, double *path_x, double *path_y, int flag, VcData *data) {
//<ROS2JAUS>
	beginner_tutorials::steeringControl srv;
	srv.request.omega = *omega;
	srv.request.delta_d = *delta_d;
	srv.request.path_index = *path_index;
	srv.request.x = x;
	srv.request.y = y;
	srv.request.velocity = velocity;
	srv.request.heading = heading;
	srv.request.tire_angle = tire_angle;
	srv.request.flag = flag;
	if (data->rosSteerClient->call(srv)) {
		*omega = srv.response.omega;
		*delta_d = srv.response.delta_d;
		*path_index = srv.response.path_index;
		printf("return success!\n");
	}
	else {
		printf("failed somehow\n");
	}
//</ROS2JAUS>
}

double VehicleController::fromAngleToAngle(double from, double to) {
	from = wrapTo2Pi(from);
	to = wrapTo2Pi(to);
	return wrapToPi(to - from);
}

double VehicleController::wrapTo2Pi(double angle) {
	while (angle > 2 * PI) {
		angle -= 2 * PI;
	}
	while (angle < 0) {
		angle += 2 * PI;
	}
	return angle;
}

double VehicleController::wrapToPi(double angle) {
	while (angle > PI) {
		angle -= 2 * PI;
	}
	while (angle < -PI) {
		angle += 2 * PI;
	}
	return angle;
}

void VehicleController::comfort_controller(double *acceleration, int path_index, double x, double y, double velocity, double heading, double tire_angle, double *path_x, double *path_y, double *lod_x, double *lod_y, VcData *data) {
	double a = *acceleration;

	fprintf(stderr, "checking acceleration.\n");
	if (velocity < MIN_VELOCITY) {
		*acceleration = MAX_ACCELERATION * 0.9;
		return;
	}

	if (velocity > MAX_VELOCITY && fabs(tire_angle) < 0.01 * MAX_TIRE_ANGLE) {
		*acceleration = 0;
		return;
	}
	fprintf(stderr, "acceleration is not maximum or minimum.\n");


	fprintf(stderr, "setting lookahead step.\n");
	double look_ahead_seconds = 3.25;
	double stepsPerSecond = velocity / (PATH_STEP / 100 + sqrt(((path_x[path_index] - path_x[path_index + 1]) * (path_x[path_index] - path_x[path_index + 1])) + ((path_y[path_index] - path_y[path_index + 1]) * (path_y[path_index] - path_y[path_index + 1]))));
	int look_ahead = path_index + round(look_ahead_seconds * stepsPerSecond);
	double n = APPROX_PATH_LENGTH / PATH_STEP;
	double delta_d = 0;
	double omega = 0;
	int path_index_placeholder = path_index;
	if (look_ahead < n) {
		path_index_placeholder = look_ahead;
		fprintf(stderr, "steering control with the lookahead.\n");
		steering_controller(&omega, &delta_d, &path_index_placeholder, x, y, velocity, heading, tire_angle, path_x, path_y, 1, data);
	} else {
		fprintf(stderr, "steering control at the end point.\n");
		path_index_placeholder = n - 2;
		steering_controller(&omega, &delta_d, &path_index_placeholder, x, y, velocity, heading, tire_angle, path_x, path_y, 1, data);
	}
	fprintf(stderr, "look ahead set to %d.\n", look_ahead);

	double K_a = 1.0;
	double K_d = 2.0;
	double K_LOD = 0.9;

	double d_a_operating_region_length = 1;
	double d_d_operating_region_length = 1;
	double v_a_operating_region_length = 1;

	fprintf(stderr, "calculating actual steering operating point.\n");
	double delta_a = fabs(tire_angle) / MAX_TIRE_ANGLE * 100;
	double delta_a_operating = floor(delta_a / d_a_operating_region_length) * d_a_operating_region_length + d_a_operating_region_length / 2;
	int index = indexClosest(lod_y, delta_a_operating, LOD_LENGTH);
	fprintf(stderr, "ind_a:\t%d\n", index);
	double M_a = (lod_y[index] - lod_y[index - 1]) / (lod_x[index] - lod_x[index - 1]);
	double delta_a_max = K_LOD * (lod_y[index] - M_a * lod_x[index]);

	fprintf(stderr, "calculating velocity operating point.\n");
	double v_a_operating = floor(velocity / v_a_operating_region_length) * v_a_operating_region_length + v_a_operating_region_length / 2;
	delta_d = fabs(delta_d) / MAX_TIRE_ANGLE * 100;

	fprintf(stderr, "calculating desired operating point.\n");
	double delta_d_operating = floor(delta_d / d_d_operating_region_length) * d_d_operating_region_length + d_d_operating_region_length / 2;
	index = indexClosest(lod_y, delta_d_operating, LOD_LENGTH);
	fprintf(stderr, "ind_d:\t%d\n", index);
	double M_d = (lod_y[index] - lod_y[index - 1]) / (lod_x[index] - lod_x[index - 1]);
	double delta_d_max = K_LOD * (lod_y[index] - M_d * lod_x[index]);

	fprintf(stderr, "vaop :\t%lf\n", v_a_operating);
	fprintf(stderr, "da   :\t%lf\n", delta_a);
	fprintf(stderr, "damax:\t%lf\n", delta_a_max);
	fprintf(stderr, "daop :\t%lf\n", delta_a_operating);
	fprintf(stderr, "dd   :\t%lf\n", delta_d);
	fprintf(stderr, "ddmax:\t%lf\n", delta_d_max);
	fprintf(stderr, "ddop :\t%lf\n", delta_d_operating);
	fprintf(stderr, "ma   :\t%lf\n", M_a);
	fprintf(stderr, "md   :\t%lf\n", M_d);


	//if (M_d > -1) {
	if (delta_d < 4) {
		K_d = 0.0;
	}
	//if (M_a > -1) {
	if (delta_a < 4) {
		K_a = 0.0;
	}
	/*
	if (delta_d / MAX_TIRE_ANGLE * 100 < 0.05) {
		K_d = 0.0;
	}
	if (delta_a / MAX_TIRE_ANGLE * 100 < 0.05) {
		K_a = 0.0;
	}
	 */

	if (delta_d < delta_a) {
		K_d = 0.0;
	}


	fprintf(stderr, "calculating control values.\n");
	double change_delta_a = delta_a - delta_a_operating;
	double change_delta_d = delta_d - delta_d_operating;
	double change_v_a = velocity - v_a_operating;

	double change_out = K_a * (change_delta_a / M_a - change_v_a) + K_d * (change_delta_d / M_d - change_v_a);
	double out_operating = K_a * ((delta_a_operating - delta_a_max) / M_a - v_a_operating) + K_d * ((delta_d_operating - delta_d_max) / M_d - v_a_operating);
	double out = out_operating + change_out;

	fprintf(stderr, "control output.\n");

	a = out;

	if (K_d == 0 && K_a == 0) {
		a = MAX_ACCELERATION;
	}

	fprintf(stderr, "K_d  :\t%lf\n", K_d);
	fprintf(stderr, "K_a :\t%lf\n", K_a);
	fprintf(stderr, "cout :\t%lf\n", change_out);
	fprintf(stderr, "outo :\t%lf\n", out_operating);
	fprintf(stderr, "a    :\t%lf\n", a);

	*acceleration = a;
}

int VehicleController::indexClosest(double *array, double value, int length) {
	int index = 0;
	double min = fabs(array[0] - value);
	for (int ii = 1; ii < length; ii++) {
		if (fabs(array[ii] - value) < min) {
			min = fabs(array[ii] - value);
			index = ii;
		}
	}
	return index;
}
