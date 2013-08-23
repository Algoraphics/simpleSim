/*****************************************************************************
 *  Copyright (c) 2009, OpenJAUS.com
 *  All rights reserved.
 *
 *  This file is part of OpenJAUS.  OpenJAUS is distributed under the BSD
 *  license.  See the LICENSE file for details.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of the University of Florida nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/
// File Name: velodyneDataPoint.c
//
// Written By: Danny Kent (jaus AT dannykent DOT com), Tom Galluzzo (galluzzo AT gmail DOT com)
//
// Version: 3.3.0b
//
// Date: 09/08/09
//
// Description: Stores a jaus object in a mission with the
//                additional data fields associated with that object
// Modified by: Luke Roseberry (MountainTop Technology, Inc) to add Planner
//              messages to OpenJAUS.
// Modified by: Jonathan Sprinkle (University of Arizona) to
//              support the use of VelodyneDataPoint messages

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <jaus.h>
#include <openJaus.h>
#include "velodyneDataPoint.h"

// VelodyneDataPoint Constructor
VelodyneDataPoint velodyneDataPointCreate(void) {
	VelodyneDataPoint object;

	object = (VelodyneDataPoint) malloc(sizeof(VelodyneDataPointStruct));
	if (object) {
		object->x = newJausFloat(0);
		object->y = newJausFloat(0);
		object->z = newJausFloat(0);
		object->c = newJausFloat(0);
		return object;
	} else {
		return NULL;
	}
}

// VelodyneDataPoint Constructor (from Buffer)
JausBoolean velodyneDataPointFromBuffer(VelodyneDataPoint *objectPointer, unsigned char *buffer, unsigned int bufferSizeBytes) {
	unsigned int index = 0;
	VelodyneDataPoint object = NULL;
	//  int j=0;

	object = velodyneDataPointCreate();
	if (object) {
		if (!jausFloatFromBuffer(&object->x, buffer + index, bufferSizeBytes - index)) {
			printf("Error: Problem retrieving x\n");
			return JAUS_FALSE;
		}
		index += JAUS_FLOAT_SIZE_BYTES;

		if (!jausFloatFromBuffer(&object->y, buffer + index, bufferSizeBytes - index)) {
			printf("Error: Problem retrieving y\n");
			return JAUS_FALSE;
		}
		index += JAUS_FLOAT_SIZE_BYTES;

		if (!jausFloatFromBuffer(&object->z, buffer + index, bufferSizeBytes - index)) {
			printf("Error: Problem retrieving z\n");
			return JAUS_FALSE;
		}
		index += JAUS_FLOAT_SIZE_BYTES;

		if (!jausFloatFromBuffer(&object->c, buffer + index, bufferSizeBytes - index)) {
			printf("Error: Problem retrieving c\n");
			return JAUS_FALSE;
		}
		index += JAUS_FLOAT_SIZE_BYTES;

		*objectPointer = object;
		return JAUS_TRUE;
	} else {
		printf("Problem creating a float from buffer\n");
		return JAUS_FALSE;
	}
}

// Returns number of bytes put into the buffer
static unsigned int dataSize(VelodyneDataPoint object) {
	int index = 0;
	// x
	index += JAUS_FLOAT_SIZE_BYTES;
	// y
	index += JAUS_FLOAT_SIZE_BYTES;
	// z
	index += JAUS_FLOAT_SIZE_BYTES;
	// c
	index += JAUS_FLOAT_SIZE_BYTES;

	return index;
}

// External interface to tell how much data in a data point
unsigned int velodyneDataPointSize(VelodyneDataPoint object) {
	int result = 0;
	result = dataSize(object);
	return result;
}

// VelodyneDataPoint To Buffer
JausBoolean velodyneDataPointToBuffer(VelodyneDataPoint object, unsigned char *buffer, unsigned int bufferSizeBytes) {
	unsigned int index = 0;
	//  printf("Entering velodyneDataPointToBuffer\n");
	if (object && (bufferSizeBytes >= dataSize(object))) {
		//    JausUnsignedShort scale = 0;
		// Pack Message Fields to Buffer

		if (!jausFloatToBuffer(object->x, buffer + index, bufferSizeBytes - index)) {
			printf("Error: Problem setting x\n");
			return JAUS_FALSE;
		}

		index += JAUS_FLOAT_SIZE_BYTES;

		if (!jausFloatToBuffer(object->y, buffer + index, bufferSizeBytes - index)) {
			printf("Error: Problem setting y\n");
			return JAUS_FALSE;
		}
		index += JAUS_FLOAT_SIZE_BYTES;

		if (!jausFloatToBuffer(object->z, buffer + index, bufferSizeBytes - index)) {
			printf("Error: Problem setting z\n");
			return JAUS_FALSE;
		}
		index += JAUS_FLOAT_SIZE_BYTES;

		if (!jausFloatToBuffer(object->c, buffer + index, bufferSizeBytes - index)) {
			printf("Error: Problem setting c\n");
			return JAUS_FALSE;
		}
		index += JAUS_FLOAT_SIZE_BYTES;

		return JAUS_TRUE;
	}
	return JAUS_FALSE;
}

// VelodyneDataPoint Destructor
void velodyneDataPointDestroy(VelodyneDataPoint object) {
	// nothing to see here...
	free(object);
}

char *velodyneDataPointToString(VelodyneDataPoint object) {
	char *buf = NULL;
	//Setup temporary string buffer
	//Fill in maximum size of output string
	// HACK: not sure what this size should be...
	unsigned int bufSize = 128;
	buf = (char *)malloc(sizeof(char) * bufSize);
	// we use this to (occasionally) store data into
	char *tempStr;
	// 8 is chosen to ensure we can do a double
	tempStr = (char *)malloc(sizeof(char) * 8);

	strcat(buf, "(");
	jausFloatToString(object->x, tempStr);
	strcat(buf, tempStr);
	strcat(buf, ",");
	jausFloatToString(object->y, tempStr);
	strcat(buf, tempStr);
	strcat(buf, ",");
	jausFloatToString(object->z, tempStr);
	strcat(buf, tempStr);
	strcat(buf, ",");
	jausFloatToString(object->c, tempStr);
	strcat(buf, tempStr);
	strcat(buf, ")");

	free(tempStr);

	return buf;

}
