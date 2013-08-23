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
// File Name: velodyneDataSample.c
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
//              support the use of TORC MotionProfile:VelodyneDataSample messages

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <jaus.h>
#include <openJaus.h>
#include <math.h>
#include "velodyneDataSample.h"
#include "velodyneDataPoint.h"

// VelodyneDataSample Constructor
VelodyneDataSample velodyneDataSampleCreate(void) {
	VelodyneDataSample object;

	object = (VelodyneDataSample) malloc(sizeof(VelodyneDataSampleStruct));
	if (object) {
		object->numPoints = newJausInteger(0);
		object->points = jausArrayCreate();
		return object;
	} else {
		return NULL;
	}
}

// VelodyneDataSample Constructor (from Buffer)
JausBoolean velodyneDataSampleFromBuffer(VelodyneDataSample *objectPointer, unsigned char *buffer, unsigned int bufferSizeBytes) {
	unsigned int index = 0;
	int i = 0; // iterator through the number of samples
	VelodyneDataSample object = NULL;
	object = velodyneDataSampleCreate();
	if (object) {
		if (!jausIntegerFromBuffer(&object->numPoints, buffer + index, bufferSizeBytes - index)) {
			return JAUS_FALSE;
		}
		index += JAUS_INTEGER_SIZE_BYTES;
		//    printf("Unpacking buffer, number of points is %d\n", object->numPoints);

		for (i = 0; i < object->numPoints; i++) {
			VelodyneDataPoint point = NULL;
			if (!velodyneDataPointFromBuffer(&point, buffer + index, bufferSizeBytes - index)) {
				printf("Error getting velodyneDataPoint from Buffer.\n");
				return JAUS_FALSE;
			} else {
				jausArrayAdd(object->points, point);
			}
			index += velodyneDataPointSize(point);

		}
		*objectPointer = object;
		return JAUS_TRUE;
	} else {
		printf("Problem creating velodyneDataSampleFromBuffer\n");
		return JAUS_FALSE;
	}
}

// Returns number of bytes put into the buffer
static unsigned int dataSize(VelodyneDataSample object) {
	int index = 0;
	// integer for storing number of points
	index += JAUS_INTEGER_SIZE_BYTES;
	// we dont actually need a datapoint to check its size, it's static
	index += (object->numPoints) * velodyneDataPointSize(NULL);

	return index;
}

// External interface to tell how much data in a data sample
unsigned int velodyneDataSampleSize(VelodyneDataSample object) {
	int result = 0;
	result = dataSize(object);
	return result;
}

// VelodyneDataSample To Buffer
JausBoolean velodyneDataSampleToBuffer(VelodyneDataSample object, unsigned char *buffer, unsigned int bufferSizeBytes) {
	unsigned int index = 0;
	int i = 0; // iterator through points array
	//  printf("Entering velodyneDataSampleToBuffer\n");
	if (object && (bufferSizeBytes >= dataSize(object))) {
		// Pack Message Fields to Buffer

		if (!jausIntegerToBuffer(object->numPoints, buffer + index, bufferSizeBytes - index)) {
			return JAUS_FALSE;
		}
		index += JAUS_INTEGER_SIZE_BYTES;

		for (i = 0; i < object->numPoints; i++) {
			VelodyneDataPoint point = (VelodyneDataPoint)object->points->elementData[i];
			if (!velodyneDataPointToBuffer(point, buffer + index, bufferSizeBytes - index)) {
				printf("Error sending velodyneDataPointToBuffer");
				return JAUS_FALSE;
			} else {
				index += velodyneDataPointSize(point);
			}
		}

		//    printf("Leaving velodyneDataSampleToBuffer (SUCCESS)\n");
		return JAUS_TRUE;
	}
	//  printf("Leaving velodyneDataSampleToBuffer (FAILURE)\n");
	return JAUS_FALSE;
}

// VelodyneDataSample Destructor
void velodyneDataSampleDestroy(VelodyneDataSample object) {
	// we have to delete all the data in the array
	jausArrayDestroy(object->points, (void *)velodyneDataPointDestroy);
	// and delete the data we malloc'd
	free(object);
}

char *velodyneDataSampleToString(VelodyneDataSample object) {
	char *buf = NULL;

	//Setup temporary string buffer
	//Fill in maximum size of output string
	int bufSize = 256;
	// allocate the buffer
	(*buf) = (char *)malloc(sizeof(char) * bufSize);

	// Now, we use this to (occasionally) store data into
	char *tempStr;
	// 8 is chosen, based on the string below, but could be less...
	tempStr = (char *)malloc(sizeof(char) * 8);


	//	strcpy(buf, "\nMotion Profile:\n" );
	strcat(buf, "VelodyneDataSample (");
	jausIntegerToString(object->numPoints, tempStr);
	strcat(buf, tempStr);
	strcat(buf, " points included)\n");

	free(tempStr);

	return buf;

}
