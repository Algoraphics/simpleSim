/*
 *  predictionMessage.h
 *  OpenJaus
 *
 *  Created by Sean Whitsitt on 11/15/10.
 *  Copyright 2010 University of Arizona. All rights reserved.
 *
 */

#ifndef PREDICTION_MESSAGE_H
#define PREDICTION_MESSAGE_H
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
	JausByte numStates;
	JausArray prediction;
	JausUnsignedInteger timestamp;

} PredictionMessageStruct;

typedef PredictionMessageStruct *PredictionMessage;

JAUS_EXPORT
PredictionMessage predictionMessageCreate(void);

JAUS_EXPORT
void predictionMessageDestroy(PredictionMessage);

JAUS_EXPORT
JausBoolean predictionMessageFromBuffer(PredictionMessage message, unsigned char *buffer, unsigned int bufferBytes);

JAUS_EXPORT
JausBoolean predictionMessageToBuffer(PredictionMessage message, unsigned char *buffer, unsigned int bufferBytes);

JAUS_EXPORT
PredictionMessage predictionMessageFromJausMessage(JausMessage jausMessage);

JAUS_EXPORT
JausMessage predictionMessageToJausMessage(PredictionMessage message);

JAUS_EXPORT
unsigned int predictionMessageSize(PredictionMessage message);

JAUS_EXPORT
char *predictionMessageToString(PredictionMessage message);

#ifdef __cplusplus
}
#endif

#endif // PREDICTION_MESSAGE_H
