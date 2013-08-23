/*
 *  reportCurvatureMessage.h
 *  OpenJaus
 *
 *  Created by JausMessageML_Interpreter on 07/23/11.
 *
 */

#ifndef _ReportCurvatureMessage_H
#define _ReportCurvatureMessage_H
#include <jaus.h>

#include "ThesisMessages_common.h"

#define JAUS_EXPERIMENTAL_REPORTCURVATUREMESSAGE 0xE455

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
	JausFloat curvature;
	JausTime timestamp;

	// end generated code

} ReportCurvatureMessageStruct;

typedef ReportCurvatureMessageStruct *ReportCurvatureMessage;

// Creates the message of this type.
JAUS_EXPORT
ReportCurvatureMessage reportCurvatureMessageCreate(void);

// Destroys the message of this type.
JAUS_EXPORT
void reportCurvatureMessageDestroy(ReportCurvatureMessage);

// Gets the data for this message from a char* buffer.
JAUS_EXPORT
JausBoolean reportCurvatureMessageFromBuffer(ReportCurvatureMessage message, unsigned char *buffer, unsigned int bufferBytes);

// Puts the data for this message into a char* buffer.
JAUS_EXPORT
JausBoolean reportCurvatureMessageToBuffer(ReportCurvatureMessage message, unsigned char *buffer, unsigned int bufferBytes);

// Transforms a JausMessage generic type into a message of this type.
JAUS_EXPORT
ReportCurvatureMessage reportCurvatureMessageFromJausMessage(JausMessage jausMessage);

// Transforms a message of this type into a generic JausMessage.
JAUS_EXPORT
JausMessage reportCurvatureMessageToJausMessage(ReportCurvatureMessage message);

// Reports the current size of the message data that will be stored in the char* buffer.
JAUS_EXPORT
unsigned int reportCurvatureMessageSize(ReportCurvatureMessage message);

// Returns a string with the data of this message.
JAUS_EXPORT
char *reportCurvatureMessageToString(ReportCurvatureMessage message);

#ifdef __cplusplus
}
#endif

#endif // _ReportCurvatureMessage_H



