/*
 *  motionCommand.c
 *  OpenJaus
 *
 *  Created by JausMessageML_Interpreter on 07/23/11.
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <jaus.h>
#include <openJaus.h>
#include "motionCommand.h"

// wayPoint Constructor
MotionCommand motionCommandCreate(void) {
	MotionCommand object;

	object = (MotionCommand) malloc(sizeof(MotionCommandStruct));
	if (object) {
		// generated code
		object->desiredVelocity = newJausDouble(0);
		object->vDotMax = newJausDouble(0);
		object->desiredCurvature = newJausDouble(0);
		object->curvatureDotMax = newJausDouble(0);
		object->timeDuration = newJausUnsignedShort(0);

		// end generated code
		return object;
	} else {
		return NULL;
	}
}

// WayPoint Constructor (from Buffer)
JausBoolean motionCommandFromBuffer(MotionCommand *objectPointer, unsigned char *buffer, unsigned int bufferSizeBytes) {
	unsigned int index = 0;
	MotionCommand object = motionCommandCreate();

	if (object != NULL) {
		// generated code
		if (!jausDoubleFromBuffer(&object->desiredVelocity, buffer + index, bufferSizeBytes - index)) {
			printf("Error removing object->desiredVelocity from the buffer!!\n");
			return JAUS_FALSE;
		}
		index += JAUS_DOUBLE_SIZE_BYTES;
		if (!jausDoubleFromBuffer(&object->vDotMax, buffer + index, bufferSizeBytes - index)) {
			printf("Error removing object->vDotMax from the buffer!!\n");
			return JAUS_FALSE;
		}
		index += JAUS_DOUBLE_SIZE_BYTES;
		if (!jausDoubleFromBuffer(&object->desiredCurvature, buffer + index, bufferSizeBytes - index)) {
			printf("Error removing object->desiredCurvature from the buffer!!\n");
			return JAUS_FALSE;
		}
		index += JAUS_DOUBLE_SIZE_BYTES;
		if (!jausDoubleFromBuffer(&object->curvatureDotMax, buffer + index, bufferSizeBytes - index)) {
			printf("Error removing object->curvatureDotMax from the buffer!!\n");
			return JAUS_FALSE;
		}
		index += JAUS_DOUBLE_SIZE_BYTES;
		if (!jausUnsignedShortFromBuffer(&object->timeDuration, buffer + index, bufferSizeBytes - index)) {
			printf("Error removing object->timeDuration from the buffer!!\n");
			return JAUS_FALSE;
		}
		index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;

		// end generated code

		*objectPointer = object;
		return JAUS_TRUE;
	} else {
		printf("Problem creating way point\n");
		return JAUS_FALSE;
	}
}

// Returns number of bytes put into the buffer
static unsigned int dataSize(MotionCommand object) {
	unsigned int size = 0;

	// generated code
	size += JAUS_DOUBLE_SIZE_BYTES;
	size += JAUS_DOUBLE_SIZE_BYTES;
	size += JAUS_DOUBLE_SIZE_BYTES;
	size += JAUS_DOUBLE_SIZE_BYTES;
	size += JAUS_UNSIGNED_SHORT_SIZE_BYTES;

	// end generated code

	return size;
}

// External interface to tell how much data in a way point
unsigned int motionCommandSize(MotionCommand object) {
	return dataSize(object);
}

// WayPoint To Buffer
JausBoolean motionCommandToBuffer(MotionCommand object, unsigned char *buffer, unsigned int bufferSizeBytes) {
	unsigned int index = 0;
	if (object && (bufferSizeBytes >= dataSize(object))) {
		// generated code
		if (!jausDoubleToBuffer(object->desiredVelocity, buffer + index, bufferSizeBytes - index)) {
			printf("Error placing object->desiredVelocity into the buffer!!\n");
			return JAUS_FALSE;
		}
		index += JAUS_DOUBLE_SIZE_BYTES;
		if (!jausDoubleToBuffer(object->vDotMax, buffer + index, bufferSizeBytes - index)) {
			printf("Error placing object->vDotMax into the buffer!!\n");
			return JAUS_FALSE;
		}
		index += JAUS_DOUBLE_SIZE_BYTES;
		if (!jausDoubleToBuffer(object->desiredCurvature, buffer + index, bufferSizeBytes - index)) {
			printf("Error placing object->desiredCurvature into the buffer!!\n");
			return JAUS_FALSE;
		}
		index += JAUS_DOUBLE_SIZE_BYTES;
		if (!jausDoubleToBuffer(object->curvatureDotMax, buffer + index, bufferSizeBytes - index)) {
			printf("Error placing object->curvatureDotMax into the buffer!!\n");
			return JAUS_FALSE;
		}
		index += JAUS_DOUBLE_SIZE_BYTES;
		if (!jausUnsignedShortToBuffer(object->timeDuration, buffer + index, bufferSizeBytes - index)) {
			printf("Error placing object->timeDuration into the buffer!!\n");
			return JAUS_FALSE;
		}
		index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;

		// end generated code
		return JAUS_TRUE;
	}
	return JAUS_FALSE;
}

// WayPoint Destructor
void motionCommandDestroy(MotionCommand object) {
	// nothing to see here...
	if (object) {
		// generated code

		// end generated code
	}
}

char *motionCommandToString(MotionCommand object) {
	unsigned int bufSize = 128 * dataSize(object);
	char *tempStr = (char *)malloc(sizeof(char) * bufSize);
	// generated code
	char *strdesiredVelocity = (char *)malloc(sizeof(char) * 100);
	jausDoubleToString(object->desiredVelocity, strdesiredVelocity);
	strcat(tempStr, strdesiredVelocity);
	free(strdesiredVelocity);
	strcat(tempStr, "\n");
	char *strvDotMax = (char *)malloc(sizeof(char) * 100);
	jausDoubleToString(object->vDotMax, strvDotMax);
	strcat(tempStr, strvDotMax);
	free(strvDotMax);
	strcat(tempStr, "\n");
	char *strdesiredCurvature = (char *)malloc(sizeof(char) * 100);
	jausDoubleToString(object->desiredCurvature, strdesiredCurvature);
	strcat(tempStr, strdesiredCurvature);
	free(strdesiredCurvature);
	strcat(tempStr, "\n");
	char *strcurvatureDotMax = (char *)malloc(sizeof(char) * 100);
	jausDoubleToString(object->curvatureDotMax, strcurvatureDotMax);
	strcat(tempStr, strcurvatureDotMax);
	free(strcurvatureDotMax);
	strcat(tempStr, "\n");
	char *strtimeDuration = (char *)malloc(sizeof(char) * 100);
	jausUnsignedShortToString(object->timeDuration, strtimeDuration);
	strcat(tempStr, strtimeDuration);
	free(strtimeDuration);
	strcat(tempStr, "\n");

	// end generated code
	return tempStr;
}

