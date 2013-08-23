/*
 *  state.h
 *  OpenJaus
 *
 *  Created by Sean Whitsitt on 11/15/10.
 *  Copyright 2010 University of Arizona. All rights reserved.
 *
 */

#ifndef STATE_H
#define STATE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <jaus.h>
#include <string.h>

// ************************************************************************************************************************************
//			State
// ************************************************************************************************************************************
typedef struct {
	// HACK: All these types should be confirmed by the TORC folks
	JausDouble x;   // m/s
	JausDouble y;           // m/s/s
	JausDouble z;  // atan( 1/m)
	JausDouble tireAngle;   // in 1/(m s)
	JausDouble carAngle;      // in ms
} StateStruct;

typedef StateStruct *State;

// State Constructor
JAUS_EXPORT State stateCreate(void);

// State Constructor (from Buffer)
JAUS_EXPORT JausBoolean stateFromBuffer(State *messagePointer, unsigned char *buffer, unsigned int bufferSizeBytes);

// State To Buffer
JAUS_EXPORT JausBoolean stateToBuffer(State task, unsigned char *buffer, unsigned int bufferSizeBytes);

// State Destructor
JAUS_EXPORT void stateDestroy(State object);

// State Buffer Size
JAUS_EXPORT unsigned int stateSize(State object);

JAUS_EXPORT char *stateToString(State value);

#ifdef __cplusplus
}
#endif


#endif // STATE_H

