/*
 *  motionCommand.h
 *  OpenJaus
 *
 *  Created by JausMessageML_Interpreter on 07/23/11.
 *
 */


#ifndef _MotionCommand_H
#define _MotionCommand_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <jaus.h>
#include <string.h>

#include "ThesisMessages_common.h"

typedef struct {
	// generated code
	JausDouble desiredVelocity;
	JausDouble vDotMax;
	JausDouble desiredCurvature;
	JausDouble curvatureDotMax;
	JausUnsignedShort timeDuration;

	// end generated code
} MotionCommandStruct;

typedef MotionCommandStruct *MotionCommand;

// MotionCommand Constructor
JAUS_EXPORT MotionCommand motionCommandCreate(void);

// MotionCommand Constructor (from Buffer)
JAUS_EXPORT JausBoolean motionCommandFromBuffer(MotionCommand *objectPointer, unsigned char *buffer, unsigned int bufferSizeBytes);

// MotionCommand To Buffer
JAUS_EXPORT JausBoolean motionCommandToBuffer(MotionCommand object, unsigned char *buffer, unsigned int bufferSizeBytes);

// MotionCommand Destructor
JAUS_EXPORT void motionCommandDestroy(MotionCommand object);

// MotionCommand Buffer Size
JAUS_EXPORT unsigned int motionCommandSize(MotionCommand object);

// MotionCommand Data To String.
JAUS_EXPORT char *motionCommandToString(MotionCommand object);

#ifdef __cplusplus
}
#endif


#endif // _MotionCommand_H

