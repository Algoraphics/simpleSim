/*
 *  queryVelodyneDataMessage.h
 *  OpenJaus
 *
 *  Created by Jonathan Sprinkle on 10/13/10.
 *  Copyright 2010 University of Arizona. All rights reserved.
 *
 */

#ifndef QUERY_VELODYNE_DATA_MESSAGE_H
#define QUERY_VELODYNE_DATA_MESSAGE_H
#include <jaus.h>
#include "common.h"

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
	JausUnsignedInteger timestamp;

} QueryVelodyneDataMessageStruct;

typedef QueryVelodyneDataMessageStruct *QueryVelodyneDataMessage;

JAUS_EXPORT
QueryVelodyneDataMessage queryVelodyneDataMessageCreate(void);

JAUS_EXPORT
void queryVelodyneDataMessageDestroy(QueryVelodyneDataMessage);

JAUS_EXPORT
JausBoolean queryVelodyneDataMessageFromBuffer(QueryVelodyneDataMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);

JAUS_EXPORT
JausBoolean queryVelodyneDataMessageToBuffer(QueryVelodyneDataMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);

JAUS_EXPORT
QueryVelodyneDataMessage queryVelodyneDataMessageFromJausMessage(JausMessage jausMessage);

JAUS_EXPORT
JausMessage queryVelodyneDataMessageToJausMessage(QueryVelodyneDataMessage message);

JAUS_EXPORT
unsigned int queryVelodyneDataMessageSize(QueryVelodyneDataMessage message);

JAUS_EXPORT
char *queryVelodyneDataMessageToString(QueryVelodyneDataMessage message);

#ifdef __cplusplus
}
#endif

#endif // VELODYNE_DATA_MESSAGE_H


