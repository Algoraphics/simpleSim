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
// File Name: VelodyneDataSample.h
//
// Written By: Luke Roseberry (MountainTop Technology, Inc)
//
// Version: 3.3.0b
//
// Date: 09/08/09
//
// Description: This file describes all the functionality associated with a VelodyneDataSample.
//                VelodyneDataSamples are used to support the storage and transfer of mission through the planning message set.
//
// Modified by: Luke Roseberry (MountainTop Technology, Inc) to add Planner
//              messages to OpenJAUS.
// Modified by: Jonathan Sprinkle (University of Arizona) to
//              support the integration of Velodyne messages

#ifndef VELODYNE_DATA_SAMPLE_H
#define VELODYNE_DATA_SAMPLE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <jaus.h>
#include <string.h>

// ************************************************************************************************************************************
//			VelodyneDataSample
// ************************************************************************************************************************************
typedef struct {
	JausInteger numPoints;    // number of points in array
	JausArray   points;       // collection of data points
} VelodyneDataSampleStruct;

typedef VelodyneDataSampleStruct *VelodyneDataSample;

// VelodyneDataSample Constructor
JAUS_EXPORT VelodyneDataSample velodyneDataSampleCreate(void);

// VelodyneDataSample Constructor (from Buffer)
JAUS_EXPORT JausBoolean velodyneDataSampleFromBuffer(VelodyneDataSample *messagePointer, unsigned char *buffer, unsigned int bufferSizeBytes);

// VelodyneDataSample To Buffer
JAUS_EXPORT JausBoolean velodyneDataSampleToBuffer(VelodyneDataSample task, unsigned char *buffer, unsigned int bufferSizeBytes);

// VelodyneDataSample Destructor
JAUS_EXPORT void velodyneDataSampleDestroy(VelodyneDataSample object);

// VelodyneDataSample Buffer Size
JAUS_EXPORT unsigned int velodyneDataSampleSize(VelodyneDataSample object);

JAUS_EXPORT char *velodyneDataSampleToString(VelodyneDataSample value);

#ifdef __cplusplus
}
#endif


#endif // VELODYNE_DATA_SAMPLE_H
