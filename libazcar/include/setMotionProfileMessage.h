/*
 *  setMotionProfileMessage.h
 *  OpenJaus
 *
 *  Created by JausMessageML_Interpreter on 07/23/11.
 *
 */

#ifndef _SetMotionProfileMessage_H
#define _SetMotionProfileMessage_H
#include <jaus.h>
#include "motionCommand.h"

#include "ThesisMessages_common.h"

#define JAUS_EXPERIMENTAL_SETMOTIONPROFILEMESSAGE 0xE328

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

	// generated code
	JausByte msgVersion;
	JausByte nummotionCommands;
	JausArray motionCommands;

	// end generated code

} SetMotionProfileMessageStruct;

typedef SetMotionProfileMessageStruct *SetMotionProfileMessage;

// Creates the message of this type.
JAUS_EXPORT
SetMotionProfileMessage setMotionProfileMessageCreate(void);

// Destroys the message of this type.
JAUS_EXPORT
void setMotionProfileMessageDestroy(SetMotionProfileMessage);

// Gets the data for this message from a char* buffer.
JAUS_EXPORT
JausBoolean setMotionProfileMessageFromBuffer(SetMotionProfileMessage message, unsigned char *buffer, unsigned int bufferBytes);

// Puts the data for this message into a char* buffer.
JAUS_EXPORT
JausBoolean setMotionProfileMessageToBuffer(SetMotionProfileMessage message, unsigned char *buffer, unsigned int bufferBytes);

// Transforms a JausMessage generic type into a message of this type.
JAUS_EXPORT
SetMotionProfileMessage setMotionProfileMessageFromJausMessage(JausMessage jausMessage);

// Transforms a message of this type into a generic JausMessage.
JAUS_EXPORT
JausMessage setMotionProfileMessageToJausMessage(SetMotionProfileMessage message);

// Reports the current size of the message data that will be stored in the char* buffer.
JAUS_EXPORT
unsigned int setMotionProfileMessageSize(SetMotionProfileMessage message);

// Returns a string with the data of this message.
JAUS_EXPORT
char *setMotionProfileMessageToString(SetMotionProfileMessage message);

#ifdef __cplusplus
}
#endif

#endif // _SetMotionProfileMessage_H



