#ifndef AZCAR_COMMON_H
#define AZCAR_COMMON_H

#include "ThesisMessages_common.h"

// This is the component ID as identified in the
// ByWireXGV User Manual, p39
#define JAUS_EXPERIMENTAL_MOTION_PROFILE_DRIVER 10
// HACK: figure out how to do this...
#define XGVLISTENER_COMPONENT_ID 91
#define JAUS_EXPERIMENTAL_LISTENER XGVLISTENER_COMPONENT_ID

// create the motion profile message
#define JAUS_EXPERIMENTAL_SET_MOTION_PROFILE JAUS_EXPERIMENTAL_SETMOTIONPROFILEMESSAGE
#define JAUS_SET_MOTION_PROFILE_MESSAGE JAUS_EXPERIMENTAL_SET_MOTION_PROFILE
#define JAUS_EXPERIMENTAL_QUERY_CURVATURE 0xE255
#define JAUS_EXPERIMENTAL_REPORT_CURVATURE JAUS_EXPERIMENTAL_REPORTCURVATUREMESSAGE
// Heaven knows whether these are appropriate message values...
#define JAUS_EXPERIMENTAL_QUERY_VELODYNE_DATA 0xE256
#define JAUS_EXPERIMENTAL_REPORT_VELODYNE_DATA 0xE456
#define JAUS_EXPERIMENTAL_WAY_POINT_REQUEST_DATA 0xE656
#define JAUS_EXPERIMENTAL_VERIFY_PATH_REQUEST_DATA 0xE856
#define JAUS_EXPERIMENTAL_PREDICTION_DATA 0xEA56

#include "ThesisMessages_common.h"
#include "setMotionProfileMessage.h"
#include "motionCommand.h"
#include "velodyneDataPoint.h"
#include "velodyneDataSample.h"
#include "velodyneDataMessage.h"

#endif // AZCAR_COMMON_H
