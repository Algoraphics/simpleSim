/*
 *  wayPoint.c
 *  OpenJaus
 *
 *  Created by Sean Whitsitt on 11/15/10.
 *  Copyright 2010 University of Arizona. All rights reserved.
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <jaus.h>
#include <openJaus.h>
#include <math.h>
#include "wayPoint.h"

// wayPoint Constructor
WayPoint wayPointCreate(void) {
	WayPoint object;

	object = (WayPoint) malloc(sizeof(WayPointStruct));
	if (object) {
		object->x = newJausDouble(0);
		object->y = newJausDouble(0);
		object->z = newJausDouble(0);
		object->theta = newJausDouble(0);
		return object;
	} else {
		return NULL;
	}
}

// WayPoint Constructor (from Buffer)
JausBoolean wayPointFromBuffer(WayPoint *pointPointer, unsigned char *buffer, unsigned int bufferSizeBytes) {
	unsigned int ii = 0;
	WayPoint point = NULL;

	point = wayPointCreate();

	if (point) {
		JausUnsignedShort scale = 0.0;
		// Unpack Fields from the Buffer, note that there are 4 JausDoubles
		// TODO: Determine whether this is really stored as an unsigned short, or as
		// a signed integer...I think it is actually a signed int
		for (ii = 0; ii < 4 * JAUS_UNSIGNED_SHORT_SIZE_BYTES; ii += JAUS_UNSIGNED_SHORT_SIZE_BYTES) {
			if (!jausUnsignedShortFromBuffer(&scale, buffer + ii, bufferSizeBytes - ii)) {
				printf("Leaving with %d\n", ii / JAUS_UNSIGNED_SHORT_SIZE_BYTES);
				free(point);
				return JAUS_FALSE;
			}
			if (ii == 0) {
				point->x = jausUnsignedIntegerToDouble(scale, DISTANCE_MIN, DISTANCE_MAX);
			} else if (ii == 1 * JAUS_UNSIGNED_INTEGER_SIZE_BYTES) {
				point->y = jausUnsignedIntegerToDouble(scale, DISTANCE_MIN, DISTANCE_MAX);
			} else if (ii == 2 * JAUS_UNSIGNED_INTEGER_SIZE_BYTES) {
				point->z = jausUnsignedIntegerToDouble(scale, DISTANCE_MIN, DISTANCE_MAX);
			} else if (ii == 3 * JAUS_UNSIGNED_INTEGER_SIZE_BYTES) {
				point->theta = tan(jausUnsignedShortToDouble(scale, ANGLE_MIN, ANGLE_MAX));
			}
		}

		*pointPointer = point;
		return JAUS_TRUE;
	} else {
		printf("Problem creating way point\n");
		return JAUS_FALSE;
	}
}

// Returns number of bytes put into the buffer
static unsigned int dataSize(WayPoint point) {
	int index = 0;
	// x
	index += JAUS_UNSIGNED_INTEGER_SIZE_BYTES;
	// y
	index += JAUS_UNSIGNED_INTEGER_SIZE_BYTES;
	// z
	index += JAUS_UNSIGNED_INTEGER_SIZE_BYTES;
	// theta
	index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;

	return index;
}

// External interface to tell how much data in a way point
unsigned int wayPointSize(WayPoint object) {
	int result = 0;
	result = dataSize(object);
	return result;
}

// WayPoint To Buffer
JausBoolean wayPointToBuffer(WayPoint point, unsigned char *buffer, unsigned int bufferSizeBytes) {
	unsigned int index = 0;
	if (point && (bufferSizeBytes >= dataSize(point))) {
		JausUnsignedShort scale = 0;
		// Pack Message Fields to Buffer

		// TODO: Determine whether this is really stored as an unsigned short, or as
		// a signed integer...I think it is actually a signed int
		scale = jausUnsignedIntegerFromDouble(point->x, DISTANCE_MIN, DISTANCE_MAX);
		if (!jausUnsignedIntegerToBuffer(scale, buffer + index, bufferSizeBytes - index)) {
			return JAUS_FALSE;
		}
		index += JAUS_UNSIGNED_INTEGER_SIZE_BYTES;
		// Pack Message Fields to Buffer
		scale = jausUnsignedIntegerFromDouble(point->y, DISTANCE_MIN, DISTANCE_MAX);
		if (!jausUnsignedIntegerToBuffer(scale, buffer + index, bufferSizeBytes - index)) {
			return JAUS_FALSE;
		}
		index += JAUS_UNSIGNED_INTEGER_SIZE_BYTES;
		// Pack Message Fields to Buffer
		scale = jausUnsignedIntegerFromDouble(point->z, DISTANCE_MIN, DISTANCE_MAX);
		if (!jausUnsignedIntegerToBuffer(scale, buffer + index, bufferSizeBytes - index)) {
			return JAUS_FALSE;
		}
		index += JAUS_UNSIGNED_INTEGER_SIZE_BYTES;
		// Pack Message Fields to Buffer
		// take the atan, as per the TORC docco
		scale = jausUnsignedShortFromDouble(atan(point->theta), ANGLE_MIN, ANGLE_MAX);
		if (!jausUnsignedShortToBuffer(scale, buffer + index, bufferSizeBytes - index)) {
			return JAUS_FALSE;
		}
		index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
		return JAUS_TRUE;
	}
	return JAUS_FALSE;
}

// WayPoint Destructor
void wayPointDestroy(WayPoint object) {
	// nothing to see here...
	if (object) {
		free(object);
	}
}

char *wayPointToString(WayPoint object) {
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
	strcat(buf, "(m)  \\ddot \\theta=");
	jausDoubleToString(object->theta, tempStr);
	strcat(buf, tempStr);

	free(tempStr);

	return buf;

}
