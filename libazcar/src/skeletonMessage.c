/*****************************************************************************
 *  Copyright (c) 2009, OpenJAUS.com
 *  All rights reserved.
 *
 *  This file is part of OpenJAUS.  OpenJAUS is distributed under the BSD
 *  license.  See the LICENSE file for details.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of the University of Florida nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/
// File Name: xXXXMessage.c
//
// Written By: Danny Kent (jaus AT dannykent DOT com), Tom Galluzzo (galluzzo AT gmail DOT com)
//
// Version: 3.3.0b
//
// Date: 09/08/09
//
// Description: This file defines the functionality of a XxXxMessage

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "jaus.h"

static const int commandCode = JAUS_XXXX;
static const int maxDataSizeBytes = 0;

static JausBoolean headerFromBuffer(XxXxMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static JausBoolean headerToBuffer(XxXxMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static int headerToString(XxXxMessage message, char **buf);

static JausBoolean dataFromBuffer(XxXxMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static int dataToBuffer(XxXxMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static void dataInitialize(XxXxMessage message);
static void dataDestroy(XxXxMessage message);
static unsigned int dataSize(XxXxMessage message);

// ************************************************************************************************************** //
//                                    USER CONFIGURED FUNCTIONS
// ************************************************************************************************************** //

// Initializes the message-specific fields
static void dataInitialize(XxXxMessage message) {
	// Set initial values of message fields
	// Example from ReportGlobalPoseMessage.c
	//
	//	message->presenceVector = newJausUnsignedShort(JAUS_SHORT_PRESENCE_VECTOR_ALL_ON);
	//	message->latitudeDegrees = newJausDouble(0);			// Scaled Int (-90, 90)
	//	message->longitudeDegrees = newJausDouble(0);			// Scaled Int (-180, 180)
	//	message->elevationMeters = newJausDouble(0);			// Scaled Int (-10000, 35000)
	//	message->positionRmsMeters = newJausDouble(0);			// Scaled UInt (0, 100)
	//	message->rollRadians = newJausDouble(0);				// Scaled Short (-JAUS_PI, JAUS_PI)
	//	message->pitchRadians = newJausDouble(0);				// Scaled Short (-JAUS_PI, JAUS_PI)
	//	message->yawRadians = newJausDouble(0);					// Scaled Short (-JAUS_PI, JAUS_PI)
	//	message->attitudeRmsRadians = newJausDouble(0);			// Scaled Short (0, JAUS_PI)
	//	message->timeStamp = newJausUnsignedInteger(0);

}

// Destructs the message-specific fields
static void dataDestroy(XxXxMessage message) {
	// Free message fields
}

// Return boolean of success
static JausBoolean dataFromBuffer(XxXxMessage message, unsigned char *buffer, unsigned int bufferSizeBytes) {
	int index = 0;

	if (bufferSizeBytes == message->dataSize) {
		// Unpack Message Fields from Buffer

		// Couple Examples from ReportGlobalPoseMessage
		//
		//	if(!jausUnsignedShortFromBuffer(&message->presenceVector, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
		//	index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
		//
		//	if(jausUnsignedShortIsBitSet(message->presenceVector, JAUS_POSE_PV_LATITUDE_BIT))
		//	{
		//		//unpack
		//		if(!jausIntegerFromBuffer(&tempInt, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
		//		index += JAUS_INTEGER_SIZE_BYTES;
		//		// Scaled Int (-90, 90)
		//		message->latitudeDegrees = jausIntegerToDouble(tempInt, -90, 90);
		//	}


		return JAUS_TRUE;
	} else {
		return JAUS_FALSE;
	}
}

// Returns number of bytes put into the buffer
static int dataToBuffer(XxXxMessage message, unsigned char *buffer, unsigned int bufferSizeBytes) {
	int index = 0;

	if (bufferSizeBytes >= dataSize(message)) {
		// Pack Message Fields to Buffer

		// Couple Examples from ReportGlobalPoseMessage
		//
		//	if(!jausUnsignedShortToBuffer(message->presenceVector, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
		//	index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
		//
		//	if(jausUnsignedShortIsBitSet(message->presenceVector, JAUS_POSE_PV_LATITUDE_BIT))
		//	{
		//		// Scaled Int (-90, 90)
		//		tempInt = jausIntegerFromDouble(message->latitudeDegrees, -90, 90);
		//
		//		//pack
		//		if(!jausIntegerToBuffer(tempInt, buffer+index, bufferSizeBytes-index)) return JAUS_FALSE;
		//		index += JAUS_INTEGER_SIZE_BYTES;
		//	}

	}

	return index;
}

static int dataToString(XxXxMessage message, char **buf) {
	//message already verified

	//Setup temporary string buffer

	//Fill in maximum size of output string
	unsigned int bufSize = ;
	(*buf) = (char *)malloc(sizeof(char) * bufSize);

	//Code example from ActivateServiceConnectionMessage
	//char* tmpStr;
	//tmpStr = (char*)malloc(sizeof(char)*20);

	//strcpy((*buf), "\nService Connection Command Code: " );

	//strcat((*buf), jausCommandCodeString(message->commandCode));

	//strcpy((*buf), "\nInstance Id: " );

	//jausByteToString(message->instanceId, tmpStr);
	//strcat((*buf), tmpStr);

	//free(tmpStr);

	return strlen((*buf));
}

// Returns number of bytes put into the buffer
static unsigned int dataSize(XxXxMessage message) {
	int index = 0;

	// Couple Examples from ReportGlobalPoseMessage
	//
	//	// PresenceVector
	//	index += JAUS_UNSIGNED_SHORT_SIZE_BYTES;
	//
	//	if(jausUnsignedShortIsBitSet(message->presenceVector, JAUS_POSE_PV_LATITUDE_BIT))
	//	{
	//		// Latitude
	//		index += JAUS_INTEGER_SIZE_BYTES;
	//	}

	return index;
}

// ************************************************************************************************************** //
//                                    NON-USER CONFIGURED FUNCTIONS
// ************************************************************************************************************** //

XxXxMessage xXXXMessageCreate(void) {
	XxXxMessage message;

	message = (XxXxMessage)malloc(sizeof(XxXxMessageStruct));
	if (message == NULL) {
		return NULL;
	}

	// Initialize Values
	message->properties.priority = JAUS_DEFAULT_PRIORITY;
	message->properties.ackNak = JAUS_ACK_NAK_NOT_REQUIRED;
	message->properties.scFlag = JAUS_NOT_SERVICE_CONNECTION_MESSAGE;
	message->properties.expFlag = JAUS_NOT_EXPERIMENTAL_MESSAGE;
	message->properties.version = JAUS_VERSION_3_3;
	message->properties.reserved = 0;
	message->commandCode = commandCode;
	message->destination = jausAddressCreate();
	message->source = jausAddressCreate();
	message->dataFlag = JAUS_SINGLE_DATA_PACKET;
	message->dataSize = maxDataSizeBytes;
	message->sequenceNumber = 0;

	dataInitialize(message);
	message->dataSize = dataSize(message);

	return message;
}

void xXXXMessageDestroy(XxXxMessage message) {
	dataDestroy(message);
	jausAddressDestroy(message->source);
	jausAddressDestroy(message->destination);
	free(message);
}

JausBoolean xXXXMessageFromBuffer(XxXxMessage message, unsigned char *buffer, unsigned int bufferSizeBytes) {
	int index = 0;

	if (headerFromBuffer(message, buffer + index, bufferSizeBytes - index)) {
		index += JAUS_HEADER_SIZE_BYTES;
		if (dataFromBuffer(message, buffer + index, bufferSizeBytes - index)) {
			return JAUS_TRUE;
		} else {
			return JAUS_FALSE;
		}
	} else {
		return JAUS_FALSE;
	}
}

JausBoolean xXXXMessageToBuffer(XxXxMessage message, unsigned char *buffer, unsigned int bufferSizeBytes) {
	if (bufferSizeBytes < xXXXMessageSize(message)) {
		return JAUS_FALSE; //improper size
	} else {
		message->dataSize = dataToBuffer(message, buffer + JAUS_HEADER_SIZE_BYTES, bufferSizeBytes - JAUS_HEADER_SIZE_BYTES);
		if (headerToBuffer(message, buffer, bufferSizeBytes)) {
			return JAUS_TRUE;
		} else {
			return JAUS_FALSE; // headerToXxXxBuffer failed
		}
	}
}

XxXxMessage xXXXMessageFromJausMessage(JausMessage jausMessage) {
	XxXxMessage message;

	if (jausMessage->commandCode != commandCode) {
		return NULL; // Wrong message type
	} else {
		message = (XxXxMessage)malloc(sizeof(XxXxMessageStruct));
		if (message == NULL) {
			return NULL;
		}

		message->properties.priority = jausMessage->properties.priority;
		message->properties.ackNak = jausMessage->properties.ackNak;
		message->properties.scFlag = jausMessage->properties.scFlag;
		message->properties.expFlag = jausMessage->properties.expFlag;
		message->properties.version = jausMessage->properties.version;
		message->properties.reserved = jausMessage->properties.reserved;
		message->commandCode = jausMessage->commandCode;
		message->destination = jausAddressCreate();
		*message->destination = *jausMessage->destination;
		message->source = jausAddressCreate();
		*message->source = *jausMessage->source;
		message->dataSize = jausMessage->dataSize;
		message->dataFlag = jausMessage->dataFlag;
		message->sequenceNumber = jausMessage->sequenceNumber;

		// Unpack jausMessage->data
		if (dataFromBuffer(message, jausMessage->data, jausMessage->dataSize)) {
			return message;
		} else {
			return NULL;
		}
	}
}

JausMessage xXXXMessageToJausMessage(XxXxMessage message) {
	JausMessage jausMessage;
	int size;

	jausMessage = (JausMessage)malloc(sizeof(struct JausMessageStruct));
	if (jausMessage == NULL) {
		return NULL;
	}

	jausMessage->properties.priority = message->properties.priority;
	jausMessage->properties.ackNak = message->properties.ackNak;
	jausMessage->properties.scFlag = message->properties.scFlag;
	jausMessage->properties.expFlag = message->properties.expFlag;
	jausMessage->properties.version = message->properties.version;
	jausMessage->properties.reserved = message->properties.reserved;
	jausMessage->commandCode = message->commandCode;
	jausMessage->destination = jausAddressCreate();
	*jausMessage->destination = *message->destination;
	jausMessage->source = jausAddressCreate();
	*jausMessage->source = *message->source;
	jausMessage->dataSize = dataSize(message);
	jausMessage->dataFlag = message->dataFlag;
	jausMessage->sequenceNumber = message->sequenceNumber;

	jausMessage->data = (unsigned char *)malloc(jausMessage->dataSize);
	jausMessage->dataSize = dataToBuffer(message, jausMessage->data, jausMessage->dataSize);

	return jausMessage;
}

unsigned int xXXXMessageSize(XxXxMessage message) {
	return (unsigned int)(dataSize(message) + JAUS_HEADER_SIZE_BYTES);
}

char *xXXXMessageToString(XxXxMessage message) {
	if (message) {
		char *buf1 = NULL;
		char *buf2 = NULL;

		int returnVal;

		//Print the message header to the string buffer
		returnVal = headerToString(message, &buf1);

		//Print the message data fields to the string buffer
		returnVal += dataToString(message, &buf2);

		char *buf;
		buf = (char *)malloc(strlen(buf1) + strlen(buf2));
		strcpy(buf, buf1);
		strcat(buf, buf2);

		free(buf1);
		free(buf2);

		return buf;
	} else {
		char *buf = "Invalid XxXx Message";
		char *msg = (char *)malloc(strlen(buf) + 1);
		strcpy(msg, buf);
		return msg;
	}
}
//********************* PRIVATE HEADER FUNCTIONS **********************//

static JausBoolean headerFromBuffer(XxXxMessage message, unsigned char *buffer, unsigned int bufferSizeBytes) {
	if (bufferSizeBytes < JAUS_HEADER_SIZE_BYTES) {
		return JAUS_FALSE;
	} else {
		// unpack header
		message->properties.priority = (buffer[0] & 0x0F);
		message->properties.ackNak	 = ((buffer[0] >> 4) & 0x03);
		message->properties.scFlag	 = ((buffer[0] >> 6) & 0x01);
		message->properties.expFlag	 = ((buffer[0] >> 7) & 0x01);
		message->properties.version	 = (buffer[1] & 0x3F);
		message->properties.reserved = ((buffer[1] >> 6) & 0x03);

		message->commandCode = buffer[2] + (buffer[3] << 8);

		message->destination->instance = buffer[4];
		message->destination->component = buffer[5];
		message->destination->node = buffer[6];
		message->destination->subsystem = buffer[7];

		message->source->instance = buffer[8];
		message->source->component = buffer[9];
		message->source->node = buffer[10];
		message->source->subsystem = buffer[11];

		message->dataSize = buffer[12] + ((buffer[13] & 0x0F) << 8);

		message->dataFlag = ((buffer[13] >> 4) & 0x0F);

		message->sequenceNumber = buffer[14] + (buffer[15] << 8);

		return JAUS_TRUE;
	}
}

static JausBoolean headerToBuffer(XxXxMessage message, unsigned char *buffer, unsigned int bufferSizeBytes) {
	JausUnsignedShort *propertiesPtr = (JausUnsignedShort *)&message->properties;

	if (bufferSizeBytes < JAUS_HEADER_SIZE_BYTES) {
		return JAUS_FALSE;
	} else {
		buffer[0] = (unsigned char)(*propertiesPtr & 0xFF);
		buffer[1] = (unsigned char)((*propertiesPtr & 0xFF00) >> 8);

		buffer[2] = (unsigned char)(message->commandCode & 0xFF);
		buffer[3] = (unsigned char)((message->commandCode & 0xFF00) >> 8);

		buffer[4] = (unsigned char)(message->destination->instance & 0xFF);
		buffer[5] = (unsigned char)(message->destination->component & 0xFF);
		buffer[6] = (unsigned char)(message->destination->node & 0xFF);
		buffer[7] = (unsigned char)(message->destination->subsystem & 0xFF);

		buffer[8] = (unsigned char)(message->source->instance & 0xFF);
		buffer[9] = (unsigned char)(message->source->component & 0xFF);
		buffer[10] = (unsigned char)(message->source->node & 0xFF);
		buffer[11] = (unsigned char)(message->source->subsystem & 0xFF);

		buffer[12] = (unsigned char)(message->dataSize & 0xFF);
		buffer[13] = (unsigned char)((message->dataFlag & 0xFF) << 4) | (unsigned char)((message->dataSize & 0x0F00) >> 8);

		buffer[14] = (unsigned char)(message->sequenceNumber & 0xFF);
		buffer[15] = (unsigned char)((message->sequenceNumber & 0xFF00) >> 8);

		return JAUS_TRUE;
	}
}

static int headerToString(XxXxMessage message, char **buf) {
	//message existance already verified

	//Setup temporary string buffer

	unsigned int bufSize = 500;
	(*buf) = (char *)malloc(sizeof(char) * bufSize);

	strcpy((*buf), jausCommandCodeString(message->commandCode));
	strcat((*buf), " (0x");
	sprintf((*buf) + strlen(*buf), "%04X", message->commandCode);

	strcat((*buf), ")\nReserved: ");
	jausUnsignedShortToString(message->properties.reserved, (*buf) + strlen(*buf));

	strcat((*buf), "\nVersion: ");
	switch (message->properties.version) {
		case 0:
			strcat((*buf), "2.0 and 2.1 compatible");
			break;
		case 1:
			strcat((*buf), "3.0 through 3.1 compatible");
			break;
		case 2:
			strcat((*buf), "3.2 and 3.3 compatible");
			break;
		default:
			strcat((*buf), "Reserved for Future: ");
			jausUnsignedShortToString(message->properties.version, (*buf) + strlen(*buf));
			break;
	}

	strcat((*buf), "\nExp. Flag: ");
	if (message->properties.expFlag == 0) {
		strcat((*buf), "Not Experimental");
	} else {
		strcat((*buf), "Experimental");
	}

	strcat((*buf), "\nSC Flag: ");
	if (message->properties.scFlag == 1) {
		strcat((*buf), "Service Connection");
	} else {
		strcat((*buf), "Not Service Connection");
	}

	strcat((*buf), "\nACK/NAK: ");
	switch (message->properties.ackNak) {
		case 0:
			strcat((*buf), "None");
			break;
		case 1:
			strcat((*buf), "Request ack/nak");
			break;
		case 2:
			strcat((*buf), "nak response");
			break;
		case 3:
			strcat((*buf), "ack response");
			break;
		default:
			break;
	}

	strcat((*buf), "\nPriority: ");
	if (message->properties.priority < 12) {
		strcat((*buf), "Normal Priority ");
		jausUnsignedShortToString(message->properties.priority, (*buf) + strlen(*buf));
	} else {
		strcat((*buf), "Safety Critical Priority ");
		jausUnsignedShortToString(message->properties.priority, (*buf) + strlen(*buf));
	}

	strcat((*buf), "\nSource: ");
	jausAddressToString(message->source, (*buf) + strlen(*buf));

	strcat((*buf), "\nDestination: ");
	jausAddressToString(message->destination, (*buf) + strlen(*buf));

	strcat((*buf), "\nData Size: ");
	jausUnsignedIntegerToString(message->dataSize, (*buf) + strlen(*buf));

	strcat((*buf), "\nData Flag: ");
	jausUnsignedIntegerToString(message->dataFlag, (*buf) + strlen(*buf));
	switch (message->dataFlag) {
		case 0:
			strcat((*buf), " Only data packet in single-packet stream");
			break;
		case 1:
			strcat((*buf), " First data packet in muti-packet stream");
			break;
		case 2:
			strcat((*buf), " Normal data packet");
			break;
		case 4:
			strcat((*buf), " Retransmitted data packet");
			break;
		case 8:
			strcat((*buf), " Last data packet in stream");
			break;
		default:
			strcat((*buf), " Unrecognized data flag code");
			break;
	}

	strcat((*buf), "\nSequence Number: ");
	jausUnsignedShortToString(message->sequenceNumber, (*buf) + strlen(*buf));

	return strlen((*buf));


}
