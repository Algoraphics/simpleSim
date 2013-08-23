/*
 *  oneNumberMessage.h
 *  OpenJaus
 *
 *  Created by Sean Whitsitt on 06/21/13.
 *  Copyright 2013 University of Arizona. All rights reserved.
 *
 */

#ifndef ONENUMBER_MESSAGE_H
#define ONENUMBER_MESSAGE_H
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
	JausInteger number;

} OneNumberMessageStruct;

typedef OneNumberMessageStruct *OneNumberMessage;

JAUS_EXPORT
OneNumberMessage oneNumberMessageCreate(void);

JAUS_EXPORT
void oneNumberMessageDestroy(OneNumberMessage);

JAUS_EXPORT
JausBoolean oneNumberMessageFromBuffer(OneNumberMessage message, unsigned char *buffer, unsigned int bufferBytes);

JAUS_EXPORT
JausBoolean oneNumberMessageToBuffer(OneNumberMessage message, unsigned char *buffer, unsigned int bufferBytes);

JAUS_EXPORT
OneNumberMessage oneNumberMessageFromJausMessage(JausMessage jausMessage);

JAUS_EXPORT
JausMessage oneNumberMessageToJausMessage(OneNumberMessage message);

JAUS_EXPORT
unsigned int oneNumberMessageSize(OneNumberMessage message);

JAUS_EXPORT
char *oneNumberMessageToString(OneNumberMessage message);

#ifdef __cplusplus
}
#endif

#endif // ONENUMBER_MESSAGE_H
