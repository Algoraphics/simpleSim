/*
 *  predictionMessage.c
 *  OpenJaus
 *
 *  Created by Sean Whitsitt on 11/15/10.
 *  Copyright 2010 University of Arizona. All rights reserved.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <jaus.h>
#include <openJaus.h>
#include <assert.h>
#include <predictionMessage.h>
#include <state.h>

static const int commandCode = JAUS_EXPERIMENTAL_PREDICTION_DATA;
static const int maxDataSizeBytes = 0;

static JausBoolean headerFromBuffer(PredictionMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static JausBoolean headerToBuffer(PredictionMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static int headerToString(PredictionMessage message, char **buf);

static JausBoolean dataFromBuffer(PredictionMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static int dataToBuffer(PredictionMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static void dataInitialize(PredictionMessage message);
static void dataDestroy(PredictionMessage message);
static unsigned int dataSize(PredictionMessage message);

// ************************************************************************************************************** //
//                                    USER CONFIGURED FUNCTIONS
// ************************************************************************************************************** //

// Initializes the message-specific fields
static void dataInitialize(PredictionMessage message) {
	//Initialize array
	message->numStates = 0;
	message->prediction = jausArrayCreate();
	message->timestamp = newJausUnsignedInteger(0);
}

// Destructs the message-specific fields
static void dataDestroy(PredictionMessage message) {
	// trash the prediction array
	jausArrayDestroy(message->prediction, (void *)stateDestroy);
}

// Return boolean of success
static JausBoolean dataFromBuffer(PredictionMessage message, unsigned char *buffer, unsigned int bufferSizeBytes) {
	int ii = 0;
	int index = 0;
	State tempObject;

	if (bufferSizeBytes >= dataSize(message)) {
		// get the timestamp from the buffer
		if (!jausUnsignedIntegerFromBuffer(&message->timestamp, buffer + index, bufferSizeBytes - index)) {
			return JAUS_FALSE;
		}
		index += JAUS_UNSIGNED_INTEGER_SIZE_BYTES;

		// get the number of states from the buffer
		if (!jausByteFromBuffer(&message->numStates, buffer + index, bufferSizeBytes - index)) {
			printf("Error retrieving number of states from buffer!!\n");
			return JAUS_FALSE;
		}
		index += JAUS_BYTE_SIZE_BYTES;

		// get the states from the buffer
		for (ii = 0; ii < message->numStates; ii++) {
			if (!stateFromBuffer(&tempObject, buffer + index, bufferSizeBytes - index)) {
				printf("Error retrieving state %d", ii);
				return JAUS_FALSE;
			}
			index += stateSize(tempObject);
			jausArrayAdd(message->prediction, tempObject);
		}

		return JAUS_TRUE;

	} else {
		printf("Buffer size not appropriate in dataFromBuffer(PredictionMessage...)\n");
		printf("Leaving dataFromBuffer(PredictionMessage...) with JAUS_FALSE\n");
		return JAUS_FALSE;
	}
}

// Returns number of bytes put into the buffer
static int dataToBuffer(PredictionMessage message, unsigned char *buffer, unsigned int bufferSizeBytes) {
	int ii = 0;
	int index = 0;
	State tempObject;

	if (bufferSizeBytes >= dataSize(message)) {
		if (!jausUnsignedIntegerToBuffer(message->timestamp, buffer + index, bufferSizeBytes - index)) {
			return JAUS_FALSE;
		}
		index += JAUS_UNSIGNED_INTEGER_SIZE_BYTES;

		if (!jausByteToBuffer(message->numStates, buffer + index, bufferSizeBytes - index)) {
			printf("Error placing number of states into buffer!!\n");
			return JAUS_FALSE;
		}
		index += JAUS_BYTE_SIZE_BYTES;

		// make sure we've got a lovely bunch of coconuts all in a row.
		assert(message->prediction->elementCount < JAUS_BYTE_MAX_VALUE);
		message->numStates = message->prediction->elementCount;

		for (ii = 0; ii < message->numStates; ii++) {
			tempObject = (State)(message->prediction->elementData[ii]);
			if (!stateToBuffer(tempObject, buffer + index, bufferSizeBytes - index)) {
				printf("Error placing state %d into the buffer\n", ii);
				return JAUS_FALSE;
			}
			index += stateSize(tempObject);
		}
	} else {
		// bad things...
		printf("Bad things in dataToBuffer(PredictionMessage...)\n");
	}

	return index;
}

// returns the length of the string placed into buf
static int dataToString(PredictionMessage message, char **buf) {
	//Assume: message already verified

	int bufSize = 1024;
	int ii = 0;
	State tempObject;
	bufSize += 256 * message->numStates;
	char *tempStr = (char *)malloc(sizeof(char) * bufSize);
	sprintf(tempStr, "\nPrediction with %d states at time ", message->numStates);
	char *tmp = (char *)malloc(sizeof(char) * 512);
	JausTime tempTime = jausTimeCreate();
	jausTimeSetTimeStamp(tempTime, message->timestamp);
	jausTimeTimeToString(tempTime, tmp);
	strcat(tempStr, tmp);
	jausTimeDestroy(tempTime);
	strcat(tempStr, ":\n");

	// add in strings for all the states
	for (ii = 0; ii < message->numStates; ii++) {
		tempObject = (State)(message->prediction->elementData[ii]);
		strcat(tempStr, stateToString(tempObject));
		strcat(tempStr, "\n");
	}

	// allocate the buffer
	(*buf) = (char *)malloc(sizeof(char) * bufSize);
	strcpy((*buf), tempStr);

	free(tempStr);

	return strlen((*buf));
}

// Returns number of bytes put into the buffer
static unsigned int dataSize(PredictionMessage message) {
	State tempObject;
	int ii = 0;
	int size = JAUS_BYTE_SIZE_BYTES + JAUS_UNSIGNED_INTEGER_SIZE_BYTES;
	for (ii = 0; ii < message->numStates; ii++) {
		tempObject = (State)(message->prediction->elementData[ii]);
		size += stateSize(tempObject);
	}
	return size;
}

// ************************************************************************************************************** //
//                                    NON-USER CONFIGURED FUNCTIONS
// ************************************************************************************************************** //

PredictionMessage predictionMessageCreate(void) {
	PredictionMessage message;

	message = (PredictionMessage)malloc(sizeof(PredictionMessageStruct));
	if (message == NULL) {
		return NULL;
	}

	// Initialize Values
	message->properties.priority = JAUS_DEFAULT_PRIORITY;
	message->properties.ackNak = JAUS_ACK_NAK_NOT_REQUIRED;
	message->properties.scFlag = JAUS_NOT_SERVICE_CONNECTION_MESSAGE;
	message->properties.expFlag = JAUS_EXPERIMENTAL_MESSAGE;
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

void predictionMessageDestroy(PredictionMessage message) {
	dataDestroy(message);
	jausAddressDestroy(message->source);
	jausAddressDestroy(message->destination);
	free(message);
}

JausBoolean predictionMessageFromBuffer(PredictionMessage message, unsigned char *buffer, unsigned int bufferSizeBytes) {
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

JausBoolean predictionMessageToBuffer(PredictionMessage message, unsigned char *buffer, unsigned int bufferSizeBytes) {
	if (bufferSizeBytes < predictionMessageSize(message)) {
		return JAUS_FALSE; //improper size
	} else {
		message->dataSize = dataToBuffer(message, buffer + JAUS_HEADER_SIZE_BYTES, bufferSizeBytes - JAUS_HEADER_SIZE_BYTES);
		if (headerToBuffer(message, buffer, bufferSizeBytes)) {
			return JAUS_TRUE;
		} else {
			return JAUS_FALSE; // headerToPredictionBuffer failed
		}
	}
}

PredictionMessage predictionMessageFromJausMessage(JausMessage jausMessage) {
	PredictionMessage message;

	if (jausMessage->commandCode != commandCode) {
		return NULL; // Wrong message type
	} else {
		message = (PredictionMessage)malloc(sizeof(PredictionMessageStruct));
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
		// first, create the jaus array
		message->prediction = jausArrayCreate();

		// Unpack jausMessage->data
		if (dataFromBuffer(message, jausMessage->data, jausMessage->dataSize)) {
			return message;
		} else {
			return NULL;
		}
	}
}

JausMessage wayPointRequestMessageToJausMessage(PredictionMessage message) {
	JausMessage jausMessage;

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

unsigned int predictionMessageSize(PredictionMessage message) {
	unsigned int size = JAUS_HEADER_SIZE_BYTES;
	size += JAUS_BYTE_SIZE_BYTES;
	size += JAUS_BYTE_SIZE_BYTES;
	size += dataSize(message);
	return (size);
}

char *predictionMessageToString(PredictionMessage message) {
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
		char *buf = "Invalid Prediction Message";
		char *msg = (char *)malloc(strlen(buf) + 1);
		strcpy(msg, buf);
		return msg;
	}
}
//********************* PRIVATE HEADER FUNCTIONS **********************//

static JausBoolean headerFromBuffer(PredictionMessage message, unsigned char *buffer, unsigned int bufferSizeBytes) {
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

static JausBoolean headerToBuffer(PredictionMessage message, unsigned char *buffer, unsigned int bufferSizeBytes) {
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

static int headerToString(PredictionMessage message, char **buf) {
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
