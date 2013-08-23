/*
 *  wayPoint.h
 *  OpenJaus
 *
 *  Created by Sean Whitsitt on 11/15/10.
 *  Copyright 2010 University of Arizona. All rights reserved.
 *
 */

#ifndef WAY_POINT_H
#define WAY_POINT_H

#define DISTANCE_MAX 10.0
#define DISTANCE_MIN 0.0
#define ANGLE_MAX (JAUS_PI/2)
#define ANGLE_MIN -ANGLE_MAX

#ifdef __cplusplus
extern "C"
{
#endif



#include <jaus.h>
#include <string.h>

// ************************************************************************************************************************************
//			WayPoint
// ************************************************************************************************************************************
typedef struct {
	// HACK: All these types should be confirmed by the TORC folks
	JausDouble x;       // m
	JausDouble y;       // m
	JausDouble z;       // m
	JausDouble theta;   // atan(1/m)
} WayPointStruct;

typedef WayPointStruct *WayPoint;

// WayPoint Constructor
JAUS_EXPORT WayPoint wayPointCreate(void);

// WayPoint Constructor (from Buffer)
JAUS_EXPORT JausBoolean wayPointFromBuffer(WayPoint *messagePointer, unsigned char *buffer, unsigned int bufferSizeBytes);

// WayPoint To Buffer
JAUS_EXPORT JausBoolean wayPointToBuffer(WayPoint task, unsigned char *buffer, unsigned int bufferSizeBytes);

// WayPoint Destructor
JAUS_EXPORT void wayPointDestroy(WayPoint object);

// WayPoint Buffer Size
JAUS_EXPORT unsigned int wayPointSize(WayPoint object);

JAUS_EXPORT char *wayPointToString(WayPoint value);

#ifdef __cplusplus
}
#endif


#endif // WAY_POINT_H
