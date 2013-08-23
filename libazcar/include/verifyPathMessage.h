/*
 *  waypointRequestMessage.h
 *  OpenJaus
 *
 *  Created by Sean Whitsitt on 11/14/10.
 *  Copyright 2010 University of Arizona. All rights reserved.
 *
 */

#ifndef VERIFY_PATH_MESSAGE_H
#define VERIFY_PATH_MESSAGE_H
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

} VerifyPathMessageStruct;

typedef VerifyPathMessageStruct *VerifyPathMessage;

JAUS_EXPORT
VerifyPathMessage verifyPathMessageCreate(void);

JAUS_EXPORT
void verifyPathMessageDestroy(VerifyPathMessage);

JAUS_EXPORT
JausBoolean verifyPathMessageFromBuffer(VerifyPathMessage message, unsigned char *buffer, unsigned int bufferBytes);

JAUS_EXPORT
JausBoolean verifyPathMessageToBuffer(VerifyPathMessage message, unsigned char *buffer, unsigned int bufferBytes);

JAUS_EXPORT
VerifyPathMessage verifyPathMessageFromJausMessage(JausMessage jausMessage);

JAUS_EXPORT
JausMessage verifyPathMessageToJausMessage(VerifyPathMessage message);

JAUS_EXPORT
unsigned int verifyPathMessageSize(VerifyPathMessage message);

JAUS_EXPORT
char *verifyPathMessageToString(VerifyPathMessage message);

#ifdef __cplusplus
}
#endif

#endif // VERIFY_PATH_MESSAGE_H


