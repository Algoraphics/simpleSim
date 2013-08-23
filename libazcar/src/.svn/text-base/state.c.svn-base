/*
 *  state.c
 *  OpenJaus
 *
 *  Created by Sean Whitsitt on 11/15/10.
 *  Copyright 2010 University of Arizona. All rights reserved.
 *
 */

#define DISTANCE_MAX 10.0
#define DISTANCE_MIN 0.0
#define ANGLE_MAX (JAUS_PI/2)
#define ANGLE_MIN -ANGLE_MAX

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <jaus.h>
#include <openJaus.h>
#include <math.h>
#include "state.h"

// State Constructor
State stateCreate(void) {
	State object;

	object = (State) malloc(sizeof(StateStruct));
	if (object) {
		object->x = newJausDouble(0);
		object->y = newJausDouble(0);
		object->z = newJausDouble(0);
		object->tireAngle = newJausDouble(0);
		object->carAngle = newJausDouble(0);
		return object;
	} else {
		return NULL;
	}
}

// State Constructor (from Buffer)
JausBoolean stateFromBuffer(State *statePointer, unsigned char *buffer, unsigned int bufferSizeBytes) {
	unsigned int ii = 0;
	State state = NULL;

	state = stateCreate();

	if (state) {
		JausUnsignedShort scale = 0.0;
		// Unpack Fields from the Buffer, note that there are 5 JausDoubles
		// TODO: Determine whether this is really stored as an unsigned short, or as
		// a signed integer...I think it is actually a signed int
		for (ii = 0; ii < 5 * JAUS_UNSIGNED_SHORT_SIZE_BYTES; ii += JAUS_UNSIGNED_SHORT_SIZE_BYTES) {
			if (!jausUnsignedShortFromBuffer(&scale, buffer + ii, bufferSizeBytes - ii)) {
				printf("Leaving with %d\n", ii / JAUS_UNSIGNED_SHORT_SIZE_BYTES);
				free(state);
				return JAUS_FALSE;
			}
			if (ii == 0) {
				state->x = jausUnsignedShortToDouble(scale, DISTANCE_MIN, DISTANCE_MAX);
			} else if (ii == 1 * JAUS_UNSIGNED_SHORT_SIZE_BYTES) {
				state->y = jausUnsignedShortToDouble(scale, DISTANCE_MIN, DISTANCE_MAX);
			} else if (ii == 2 * JAUS_UNSIGNED_SHORT_SIZE_BYTES) {
				state->z = jausUnsignedShortToDouble(scale, DISTANCE_MIN, DISTANCE_MAX);
			} else if (ii == 3 * JAUS_UNSIGNED_SHORT_SIZE_BYTES) {
				state->tireAngle = tan(jausUnsignedShortToDouble(scale, ANGLE_MIN, ANGLE_MAX));
			} else if (ii == 4 * JAUS_UNSIGNED_SHORT_SIZE_BYTES) {
				state->carAngle = tan(jausUnsignedShortToDouble(scale, ANGLE_MIN, ANGLE_MAX));
			}
		}

		*statePointer = state;
		return JAUS_TRUE;
	} else {
		printf("Problem creating state\n");
		return JAUS_FALSE;
	}
}

// Returns number of bytes put into the buffer
static unsigned int dataSize(State point) {
	int index = 0;
	// x
	index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
	// y
	index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
	// z
	index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
	// tireAngle
	index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
	// carAngle
	index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;

	return index;
}

// External interface to tell how much data in a way point
unsigned int stateSize(State object) {
	int result = 0;
	result = dataSize(object);
	return result;
}

// WayPoint To Buffer
JausBoolean stateToBuffer(State state, unsigned char *buffer, unsigned int bufferSizeBytes) {
	unsigned int index = 0;
	printf("Entering wayPointToBuffer\n");
	if (state && (bufferSizeBytes >= dataSize(state))) {
		JausUnsignedShort scale = 0;
		// Pack Message Fields to Buffer

		// TODO: Determine whether this is really stored as an unsigned short, or as
		// a signed integer...I think it is actually a signed int
		scale = jausUnsignedShortFromDouble(state->x, DISTANCE_MIN, DISTANCE_MAX);
		if (!jausUnsignedShortToBuffer(scale, buffer + index, bufferSizeBytes - index)) {
			return JAUS_FALSE;
		}
		index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;

		// Pack Message Fields to Buffer
		scale = jausUnsignedShortFromDouble(state->y, DISTANCE_MIN, DISTANCE_MAX);
		if (!jausUnsignedShortToBuffer(scale, buffer + index, bufferSizeBytes - index)) {
			return JAUS_FALSE;
		}
		index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;

		// Pack Message Fields to Buffer
		scale = jausUnsignedShortFromDouble(state->z, DISTANCE_MIN, DISTANCE_MAX);
		if (!jausUnsignedShortToBuffer(scale, buffer + index, bufferSizeBytes - index)) {
			return JAUS_FALSE;
		}
		index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;

		// Pack Message Fields to Buffer
		// take the atan, as per the TORC docco
		scale = jausUnsignedShortFromDouble(atan(state->tireAngle), ANGLE_MIN, ANGLE_MAX);
		if (!jausUnsignedShortToBuffer(scale, buffer + index, bufferSizeBytes - index)) {
			return JAUS_FALSE;
		}
		index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;

		// Pack Message Fields to Buffer
		// take the atan, as per the TORC docco
		scale = jausUnsignedShortFromDouble(atan(state->carAngle), ANGLE_MIN, ANGLE_MAX);
		if (!jausUnsignedShortToBuffer(scale, buffer + index, bufferSizeBytes - index)) {
			return JAUS_FALSE;
		}
		index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;

		return JAUS_TRUE;
	}
	printf("Leaving stateToBuffer\n");
	return JAUS_FALSE;
}

// State Destructor
void stateDestroy(State object) {
	// nothing to see here...
	if (object) {
		free(object);
	}
}

char *stateToString(State object) {
	char *buf = NULL;
	//Setup temporary string buffer
	//Fill in maximum size of output string
	// HACK: not sure what this size should be...
	unsigned int bufSize = 512;
	buf = (char *)malloc(sizeof(char) * bufSize);
	// we use this to (occasionally) store data into
	char *tempStr;
	// 25 is chosen to ensure we can do a double
	tempStr = (char *)malloc(sizeof(char) * 25);

	strcat(buf, "x=");
	jausDoubleToString(object->x, tempStr);
	strcat(buf, tempStr);
	strcat(buf, "(m), \\dot y=");
	jausDoubleToString(object->y, tempStr);
	strcat(buf, tempStr);
	strcat(buf, "(m), \\dot \\z=");
	jausDoubleToString(object->z, tempStr);
	strcat(buf, tempStr);
	strcat(buf, "(m)  \\ddot \\tireAngle=");
	jausDoubleToString(object->tireAngle, tempStr);
	strcat(buf, tempStr);
	strcat(buf, "(rad)  \\ddot \\carAngle=");
	jausDoubleToString(object->carAngle, tempStr);
	strcat(buf, tempStr);

	free(tempStr);

	return buf;

}
