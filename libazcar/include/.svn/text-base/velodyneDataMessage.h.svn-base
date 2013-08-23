/*
 *  velodyneDataMessage.h
 *  OpenJaus
 *
 *  Created by Jonathan Sprinkle on 10/13/10.
 *  Copyright 2010 University of Arizona. All rights reserved.
 *
 */

#ifndef VELODYNE_DATA_MESSAGE_H
#define VELODYNE_DATA_MESSAGE_H
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
	JausInteger         numSamples;
	JausArray           samples;

} VelodyneDataMessageStruct;

typedef VelodyneDataMessageStruct *VelodyneDataMessage;

JAUS_EXPORT
VelodyneDataMessage velodyneDataMessageCreate(void);

JAUS_EXPORT
void velodyneDataMessageDestroy(VelodyneDataMessage);

JAUS_EXPORT
JausBoolean velodyneDataMessageFromBuffer(VelodyneDataMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);

JAUS_EXPORT
JausBoolean velodyneDataMessageToBuffer(VelodyneDataMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);

JAUS_EXPORT
VelodyneDataMessage velodyneDataMessageFromJausMessage(JausMessage jausMessage);

JAUS_EXPORT
JausMessage velodyneDataMessageToJausMessage(VelodyneDataMessage message);

JAUS_EXPORT
unsigned int velodyneDataMessageSize(VelodyneDataMessage message);

JAUS_EXPORT
char *velodyneDataMessageToString(VelodyneDataMessage message);

#ifdef __cplusplus
}
#endif

#endif // VELODYNE_DATA_MESSAGE_H


