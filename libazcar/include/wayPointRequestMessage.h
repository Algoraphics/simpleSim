/*
 *  waypointRequestMessage.h
 *  OpenJaus
 *
 *  Created by Sean Whitsitt on 11/14/10.
 *  Copyright 2010 University of Arizona. All rights reserved.
 *
 */

#ifndef WAYPOINT_REQUEST_MESSAGE_H
#define WAYPOINT_REQUEST_MESSAGE_H
#include <jaus.h>
#include "common.h"
#include "wayPoint.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct {
	// Include all parameters from a JausMessage structure:
	// Header Properties
	struct {
		// Properties by bit fields
#ifdef JAUS_BIG_ENDIAN
		JausUnsignedShort reserved: 2;
		JausUnsignedShort version: 6;
		JausUnsignedShort expFlag: 1;
		JausUnsignedShort scFlag: 1;
		JausUnsignedShort ackNak: 2;
		JausUnsignedShort priority: 4;
#elif JAUS_LITTLE_ENDIAN
		JausUnsignedShort priority: 4;
		JausUnsignedShort ackNak: 2;
		JausUnsignedShort scFlag: 1;
		JausUnsignedShort expFlag: 1;
		JausUnsignedShort version: 6;
		JausUnsignedShort reserved: 2;
#else
#error "Please define system endianess (see jaus.h)"
#endif
	} properties;

	JausUnsignedShort commandCode;
	JausAddress destination;
	JausAddress source;
	JausUnsignedInteger dataSize;
	JausUnsignedInteger dataFlag;
	JausUnsignedShort sequenceNumber;

	// MESSAGE DATA MEMBERS GO HERE
	WayPoint wayPoint;

} WayPointRequestMessageStruct;

typedef WayPointRequestMessageStruct *WayPointRequestMessage;

JAUS_EXPORT
WayPointRequestMessage wayPointRequestMessageCreate(void);

JAUS_EXPORT
void wayPointRequestMessageDestroy(WayPointRequestMessage);

JAUS_EXPORT
JausBoolean wayPointRequestMessageFromBuffer(WayPointRequestMessage message, unsigned char *buffer, unsigned int bufferBytes);

JAUS_EXPORT
JausBoolean wayPointRequestMessageToBuffer(WayPointRequestMessage message, unsigned char *buffer, unsigned int bufferBytes);

JAUS_EXPORT
WayPointRequestMessage wayPointRequestMessageFromJausMessage(JausMessage jausMessage);

JAUS_EXPORT
JausMessage wayPointRequestMessageToJausMessage(WayPointRequestMessage message);

JAUS_EXPORT
unsigned int wayPointRequestMessageSize(WayPointRequestMessage message);

JAUS_EXPORT
char *wayPointRequestMessageToString(WayPointRequestMessage message);

#ifdef __cplusplus
}
#endif

#endif // WAYPOINT_REQUEST_MESSAGE_H


