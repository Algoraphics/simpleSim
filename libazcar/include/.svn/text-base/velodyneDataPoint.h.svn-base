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
// File Name: VelodyneDataPoint.h
//
// Written By: Luke Roseberry (MountainTop Technology, Inc)
//
// Version: 3.3.0b
//
// Date: 09/08/09
//
// Description: This file describes all the functionality associated with a VelodyneDataPoint.
//                VelodyneDataPoints are used to support the storage and transfer of mission through the planning message set.
//
// Modified by: Luke Roseberry (MountainTop Technology, Inc) to add Planner
//              messages to OpenJAUS.
// Modified by: Jonathan Sprinkle (University of Arizona) to
//              support the integration of Velodyne messages

#ifndef VELODYNE_DATA_POINT_H
#define VELODYNE_DATA_POINT_H

#ifdef __cplusplus
extern "C"
{
#endif



#include <jaus.h>
#include <string.h>

// ************************************************************************************************************************************
//			VelodyneDataPoint
// ************************************************************************************************************************************
typedef struct {
	JausFloat x;   // x pos (m)
	JausFloat y;   // y pos (m)
	JausFloat z;   // z (height) pos (m)
	JausFloat c;   // color (not sure if this is needed or not?)
} VelodyneDataPointStruct;

typedef VelodyneDataPointStruct *VelodyneDataPoint;

// VelodyneDataPoint Constructor
JAUS_EXPORT VelodyneDataPoint velodyneDataPointCreate(void);

// VelodyneDataPoint Constructor (from Buffer)
JAUS_EXPORT JausBoolean velodyneDataPointFromBuffer(VelodyneDataPoint *messagePointer, unsigned char *buffer, unsigned int bufferSizeBytes);

// VelodyneDataPoint To Buffer
JAUS_EXPORT JausBoolean velodyneDataPointToBuffer(VelodyneDataPoint task, unsigned char *buffer, unsigned int bufferSizeBytes);

// VelodyneDataPoint Destructor
JAUS_EXPORT void velodyneDataPointDestroy(VelodyneDataPoint object);

// VelodyneDataPoint Buffer Size
JAUS_EXPORT unsigned int velodyneDataPointSize(VelodyneDataPoint object);

JAUS_EXPORT char *velodyneDataPointToString(VelodyneDataPoint value);

#ifdef __cplusplus
}
#endif


#endif // VELODYNE_DATA_POINT_H
