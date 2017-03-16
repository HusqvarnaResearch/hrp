/*
*------------------------------------------------------------------------------
* (c) Husqvarna AB
*------------------------------------------------------------------------------
*/
/**
* @file
* <Brief description of this file>
*------------------------------------------------------------------------------
* \par DESCRIPTION:
*      <Detailed description of this file.>\n
*      <Backslash n forces a new line.>
*
* \par IDENTIFICATION:
*           $Module: HCP-Runtime $
*           $Target: any $
*      $Environment: <Development tool> $
*          $Project: HCP
*         $Revision: 1$
*             $Date: 2005-05-19 14:20:45$
*           $Author: Olof Andreassen$
*
*------------------------------------------------------------------------------
* \par HISTORY SUMMARY (for the 10 last revisions)
* $Log[10]$
*
*------------------------------------------------------------------------------
*/

/*
*==============================================================================
*  1.2  References
*==============================================================================
*  [Ref 1]  <Doc no. and Document name>
*  [Ref 2]  ...
*==============================================================================
*/


/*
*==============================================================================
*  2.   INCLUDE FILES
*==============================================================================
*/

#include "amg3.h"

/*
*==============================================================================
*  3.   DECLARATIONS
*  3.1  Internal constants
*==============================================================================
*/

/*
*==============================================================================
*  3.2  Internal macros
*==============================================================================
*/

/*
*==============================================================================
*  3.3  Internal type definitions
*==============================================================================
*/

#pragma pack(push, 8)
typedef struct {
	hcp_Uint8 protocolId;
	hcp_Uint16 messageLength;
	hcp_Uint8 transactionId;
} amg3_tHeader;
#pragma pack(pop)

typedef struct {
	hcp_Uint8 buff[AMG3_MESSAGESIZE];
	hcp_tBlob received;			/* wrappers [buffer] into a blob for easier access */
	hcp_Uint8 subCmd;	/* track the last sub-command sent */
	hcp_Uint16 msgType;	/* track last message type */
	hcp_Uint8 cmdResult; /* returned command result from previous response */
	hcp_Size_t lastSTX;	/* stored postion of the last encountered STX character, used to
						 * calculate CRC-values */
	hcp_tCommand* command;	/* current command that we expect response from */
	amg3_tHeader header;	/* header for messages with header support*/
} amg3_tSession;

/*
*==============================================================================
*  3.4  Global variables (declared as 'extern' in some header file)
*==============================================================================
*/

/*
*==============================================================================
*  3.5  Global constant data
*==============================================================================
*/

/*
*==============================================================================
*  3.6  Local function prototypes (defined in Section 5)
*==============================================================================
*/
static hcp_Int HCP_CALL amg3_Setup(hcp_tRuntime* pRuntime, hcp_tBuffer* pContext);
static hcp_Int HCP_CALL amg3_Encode(hcp_tRuntime* pRuntime, hcp_tProtocol* pHeaderProtocol, const hcp_tCommand* pCommand, hcp_tBlob* pDestination, hcp_tBuffer* pContext);
static hcp_Int HCP_CALL amg3_Decode(hcp_tRuntime* pRuntime, hcp_tProtocol* pProtocol, const hcp_tBlob* pSource, hcp_tCommandSet* pCommands, hcp_tCommand** ppCommand, hcp_tBuffer* pContext);
static hcp_Int amg3_AppendFooter(hcp_tRuntime* R, hcp_tBlob* pDestination, const hcp_Size_t HeaderStartOffset);
static hcp_Uint8 amg3_CalculateCrc8(const hcp_tBlob* pSource, const hcp_Size_t Start, const hcp_Size_t End);
static hcp_Uint8 amg3_GetHeaderSize(const hcp_Uint8 MessageType);
static hcp_Int amg3_InterpretHeader(hcp_tRuntime* R, amg3_tSession* pSession, const hcp_Size_t Position, hcp_Boolean* pCompleteHeader);
static hcp_Int amg3_GetDeviceError(hcp_tBuffer* pContext, hcp_szStr* pMessage);
static hcp_Int amg3_InterpretByte(hcp_tRuntime* R, const hcp_Uint8 Byte, amg3_tSession* pSession, hcp_Boolean* pCompleteMessage);

/*
*==============================================================================
*  3.7  Local variables
*==============================================================================
*/

/*
*==============================================================================
*  3.8  Local constant data
*==============================================================================
*/

/*
*==============================================================================
*  4.   GLOBAL FUNCTIONS (declared as 'extern' in some header file)
*==============================================================================
*/

static hcp_tCodecLibrary library;

hcp_tCodecLibrary* hcp_GetLibrary(void) {

	library.requiredVersion = 1;
	library.name = AMG3_NAME;
	library.decode = amg3_Decode;
	library.encode = amg3_Encode;
	library.lastError = HCP_NULL;
	library.messages = amg3_Errors;
	library.setup = amg3_Setup;

	library.context.length = 0;
	return (hcp_tCodecLibrary*)&library;
}

/*
*==============================================================================
*  5.   LOCAL FUNCTIONS (declared in Section 3.5)
*==============================================================================
*/

hcp_Int amg3_GetDeviceError(hcp_tBuffer* pContext , hcp_szStr* pMessage) {
	amg3_tSession* session = (amg3_tSession*)pContext->value;
	*pMessage = HCP_NULL;
	const hcp_Int result = session->cmdResult;

	if (pMessage != HCP_NOERROR) {
		hcp_tErrorMessage* messages = (hcp_tErrorMessage*)amg3_Errors;

		while (messages != HCP_NULL && messages->message != HCP_NULL) {
			if (result == (messages->code - (AMG3_CMDRESOFFSET))){
				*pMessage = messages->message;
				break;
			}

			messages++;
		}
	}

	return result;
}

hcp_Uint8 amg3_CalculateCrc8(const hcp_tBlob* pSource, const hcp_Size_t Start, const hcp_Size_t Length) {
	hcp_Uint8 crc = InitCrc;
	hcp_Uint8* source = (hcp_Uint8*)((hcp_Size_t)pSource->value + Start);
	hcp_Size_t length = Length;

	while (length--) {
		crc = CRC8_TABLE[(crc ^ *source)];
		source++;
	}

	return crc;
}

hcp_Int amg3_Setup(hcp_tRuntime* R, hcp_tBuffer* pContext) {
	hcp_Int error = R->Resize(pContext, sizeof(amg3_tSession));

	if (error == HCP_NOERROR) {
		amg3_tSession* session = (amg3_tSession*)pContext->value;

		hcp_tBlob* receiveBuffer = &session->received;

		receiveBuffer->length = 0;
		receiveBuffer->maxLength = sizeof(session->buff);
		receiveBuffer->value = session->buff;

		session->msgType = 0;
		session->subCmd = 0;

		R->Memset(R, receiveBuffer->value, 0, receiveBuffer->maxLength);
	}

	return error;
}

hcp_Int amg3_AppendFooter(hcp_tRuntime* R, hcp_tBlob* pDestination, const hcp_Size_t HeaderStartOffset) {
	// calculate crc
	hcp_Size_t position = 0;
	const hcp_Uint8 crc = amg3_CalculateCrc8(pDestination, HeaderStartOffset, pDestination->length - HeaderStartOffset);

	hcp_Int error = HCP_NOERROR;

	error = R->AppendByte(crc, pDestination);

	if (error == HCP_NOERROR) {
		error = R->AppendByte(AMG3_ETX, pDestination);
	}

	return error;
}

hcp_Int amg3_AppendHeader(hcp_tRuntime* R, const hcp_Uint16 MessageType, const hcp_Size_t DataLength, hcp_tBlob* pDestination) {
	hcp_Uint8 headerByteSize = amg3_GetHeaderSize((hcp_Uint8)MessageType);
	hcp_Uint8* destination = pDestination->value;
	hcp_Int error = HCP_NOERROR;
	hcp_Uint16 dataLength = (hcp_Uint16)DataLength;

	if (MessageType >= 0x7F) {
		// Add default protocol id
		error = R->AppendByte(AMG3_PROTOCOL_EXTENDED, pDestination);

		if (error != HCP_NOERROR) {
			return error;
		}

		hcp_Uint16 remaningBytes = (hcp_Uint16)(
				sizeof(hcp_Uint8) + // transaction id
				sizeof(hcp_Uint8) * 2 + // message type
				sizeof(hcp_Uint8) + // data length
				sizeof(hcp_Uint8) * DataLength + // payload size
				sizeof(hcp_Uint8) * 2); // CRC + ETX


		error = R->Uint16ToBytes(remaningBytes, pDestination, HCP_LITTLEENDIAN);

		if (error != HCP_NOERROR) {
			return error;
		}

		// for now, we ignore the transaction id
		error = R->AppendByte(0, pDestination);

		if (error != HCP_NOERROR) {
			return error;
		}

		// remove first bit since its just an indicator of the header size
		// and not the actual message type
		error = R->Uint16ToBytes(MessageType | 0x8000, pDestination, HCP_BIGENDIAN);
	}
	else {
		error = R->Uint8ToBytes((hcp_Uint8)MessageType, pDestination, AMG3_ENDIANESS);
	}

	if (error == HCP_NOERROR) {
		error = R->AppendByte((hcp_Uint8)dataLength, pDestination);
	}

	return error;
}

hcp_Int amg3_Encode(hcp_tRuntime* R, hcp_tProtocol* pHeaderProtocol, const hcp_tCommand* pCommand, hcp_tBlob* pDestination, hcp_tBuffer* pContext) {
	hcp_tProtocol* protocol = &pCommand->template_->protocol;
	amg3_tSession* session = (amg3_tSession*)pContext->value;

	// calculate the number of bytes required to represents the payload/parameters
	hcp_Size_t dataLength = R->GetParameterSetSize(&pCommand->inParams);

	// Determine if the message contains a heaer
	hcp_Int error = HCP_NOERROR;
	hcp_Uint8 subCommand = 0;
	hcp_Uint16 messageType = 0;
	hcp_Boolean hasSubCommand = HCP_FALSE;

	// the subCommand is optional (not used by all commands, but 90% of them)
	error = R->GetUint8(protocol, AMG3_PROTOCOL_SUBCMD, &subCommand);

	if (error == HCP_NOERROR) {
		hasSubCommand = HCP_TRUE;
	}
	// get the message type
	error = R->GetUint16(protocol, AMG3_PROTOCOL_MSGTYPE, &messageType);

	if (error != HCP_NOERROR) {
		return AMG3_MISSINGMSGTYPE;
	}

	// Add STX
	error = R->AppendByte(AMG3_STX, pDestination);

	if (error != HCP_NOERROR) {
		return error;
	}

	// append a sub-command byte if specified
	if (hasSubCommand) {
		dataLength++;
	}

	// add message header
	error = amg3_AppendHeader(R,messageType, dataLength, pDestination);

	if (error != HCP_NOERROR) {
		return error;
	}

	if (hasSubCommand) {
		error = R->AppendByte(subCommand, pDestination);

		if (error != HCP_NOERROR) {
			return error;
		}
	}

	// write parameters (payload)
	error = R->AppendParameters(R, pDestination, (hcp_tParameterSet*)&pCommand->inParams, AMG3_ENDIANESS, HCP_NULL, HCP_NULL);

	if (error != HCP_NOERROR) {
		return error;
	}

	// write crc and ETX
	error = amg3_AppendFooter(R,pDestination, 1);

	if (error == HCP_NOERROR) {
		// save the current command in order to know which
		// command to resolve when the request comes
		session->command = (hcp_tCommand*)pCommand;
		// clear receive buffer
		session->received.length = 0;
	}

	return error;
}

hcp_Uint8 amg3_GetHeaderSize(const hcp_Uint8 MessageType) {
	if (MessageType > 0x7F) {
		return sizeof(hcp_Uint8) * 2;
	}

	return sizeof(hcp_Uint8);
}

hcp_Int amg3_InterpretPayload(hcp_tRuntime* R, amg3_tSession* pSession, const hcp_Uint16 MessageType, const hcp_Uint8 CommandResult, const hcp_tBlob* pPayload, const hcp_Size_t Position, hcp_Boolean* pCompleteMessage) {
	*pCompleteMessage = HCP_FALSE;
	hcp_Size_t bytesRead = Position;
	hcp_tBlob* buffer = &pSession->received;
	hcp_Int error = AMG3_PARSE_MISSINGDATA;

	// before parsing parameters, lets check CRC and ETX
	hcp_Uint8 CRC = 0;
	hcp_Uint8 ETX = 0;

	// read CRC
	if (R->BytesToUint8(buffer, bytesRead, &CRC, AMG3_ENDIANESS) == HCP_NOERROR) {
		bytesRead++;

		static const hcp_Size_t footerSize =  sizeof(hcp_Uint8);

		hcp_Uint8 expectedCRC = amg3_CalculateCrc8(buffer, pSession->lastSTX,
			bytesRead - (pSession->lastSTX + footerSize));
		// if the two CRC's dont match, ignore everything we have read so far
		if (CRC != expectedCRC) {
			return AMG3_PARSE_INVALIDCRC;
		}
	}
	else {
		// no more bytes..
		return AMG3_PARSE_MISSINGDATA;
	}

	// read ETX
	if (R->BytesToUint8(buffer, bytesRead, &ETX, AMG3_ENDIANESS) == HCP_NOERROR) {
		bytesRead++;
		// last byte was not ETX, everything we have read so far is irrelevant
		if (ETX != AMG3_ETX) {
			return AMG3_PARSE_NOETX;
		}
	}
	else {
		// no more bytes..
		return AMG3_PARSE_MISSINGDATA;
	}
	// we have read all byte(s) and verified CRC and got ETX
	// we now have everthing we need to continue processing
	// the message content
	*pCompleteMessage = HCP_TRUE;
	pSession->cmdResult = CommandResult;

	switch (CommandResult) {
		case AMG3_CMD_OK: {
			// populate the command with parameters
			error = R->BytesToParameters(R, pPayload, 0, &pSession->command->outParams, AMG3_ENDIANESS, HCP_NULL, HCP_NULL);
		} break;
		case AMG3_CMD_ERR_UNKNOWN:
		case AMG3_CMD_ERR_VALUE:
		case AMG3_CMD_ERR_RANGE:
		case AMG3_CMD_ERR_NOTAVAIL:
		case AMG3_CMD_ERR_ACCESS:
		case AMG3_CMD_ERR_GROUP:
		case AMG3_CMD_ERR_ID:
		case AMG3_CMD_ERR_BUSY:
		case AMG3_CMD_ERR_INVALID_PIN:
		case AMG3_CMD_ERR_MOWER_BLOCKED: {
			error = CommandResult + AMG3_CMDRESOFFSET;
		} break;
		default: {
			pSession->cmdResult = AMG3_CMD_ERR_UNKNOWN;
			error = AMg3_PARSE_INVALIDCMDRES;
		} break;
	}

	return error;
}

hcp_Int amg3_InterpretCommandResult(hcp_tRuntime* R, amg3_tSession* pSession, const hcp_Uint16 MessageType, const hcp_Uint8 PayloadLength, const hcp_Size_t Position, hcp_Boolean* pCompleteMessage) {
	*pCompleteMessage = HCP_FALSE;
	hcp_Size_t bytesRead = Position;
	hcp_tBlob* buffer = &pSession->received;
	hcp_Int error = AMG3_PARSE_MISSINGDATA;

	hcp_Uint8 commandResult = AMG3_CMD_OK;

	// get command result
	if (R->BytesToUint8(buffer, bytesRead, &commandResult, AMG3_ENDIANESS) == HCP_NOERROR) {
		// move to the next byte
		bytesRead++;

		hcp_tBlob payload;
		// create a blob that refers only to the payload
		payload.value = (hcp_Uint8*)((hcp_Size_t)buffer->value + bytesRead);

		// remove the command-result from the payload length
		payload.length = PayloadLength - sizeof(hcp_Uint8);
		payload.maxLength = payload.length;

		// skip ahead to the end of the payload
		bytesRead += payload.length;
		error = amg3_InterpretPayload(R,pSession, MessageType, commandResult, &payload, bytesRead, pCompleteMessage);
	}

	return error;
}

hcp_Int amg3_InterpretLength(hcp_tRuntime* R, amg3_tSession* pSession,const hcp_Uint16 MessageType, const hcp_Size_t Position, hcp_Boolean* pCompleteMessage) {
	*pCompleteMessage = HCP_FALSE;
	hcp_Size_t bytesRead = Position;
	hcp_tBlob* buffer = &pSession->received;
	hcp_Int error = AMG3_PARSE_MISSINGDATA;

	hcp_Uint8 payloadLength = 0;

	// get payload length
	if (R->BytesToUint8(buffer, bytesRead, &payloadLength, AMG3_ENDIANESS) == HCP_NOERROR) {
		bytesRead++;

		// make sure we have enough byte(s) to interpret the remaning message
		if (bytesRead + payloadLength > buffer->length) {
			// return OK but with message complete equals to false
			// we need more bytes before we can continue.
			return AMG3_PARSE_MISSINGDATA;
		}

		error = amg3_InterpretCommandResult(R,pSession, MessageType, payloadLength, bytesRead, pCompleteMessage);
	}

	return error;
}

hcp_Int amg3_InterpretHeader(hcp_tRuntime* R,amg3_tSession* pSession, const hcp_Size_t Position, hcp_Boolean* pCompleteHeader) {
	*pCompleteHeader = HCP_FALSE;
	hcp_Size_t bytesRead = Position;
	hcp_tBlob* buffer = &pSession->received;
	amg3_tHeader* header = &pSession->header;
	hcp_Int error = AMG3_PARSE_MISSINGDATA;

	// Get Protocol ID (1 byte)
	if (R->BytesToUint8(buffer, bytesRead, &header->protocolId, AMG3_ENDIANESS) == HCP_NOERROR) {
		bytesRead++;
		// Get message length (2 bytes)
		if (R->BytesToUint16(buffer, bytesRead, &header->messageLength, AMG3_ENDIANESS) == HCP_NOERROR) {
			bytesRead += sizeof(hcp_Uint16);
			// Get transation id (1 byte)
			if (R->BytesToUint8(buffer, bytesRead, &header->transactionId, AMG3_ENDIANESS) == HCP_NOERROR) {
				*pCompleteHeader = HCP_TRUE;

				return sizeof(header->messageLength) + sizeof(header->transactionId) + sizeof(header->protocolId);
			}
		}
	}

	return error;
}

hcp_Int amg3_InterpretMessageType(hcp_tRuntime* R, amg3_tSession* pSession, const hcp_Size_t Position, hcp_Boolean* pCompleteMessage) {
	*pCompleteMessage = HCP_FALSE;
	hcp_Uint8 nextByte = 0;
	hcp_Size_t bytesRead = Position;
	hcp_tBlob* buffer = &pSession->received;
	hcp_Int error = AMG3_PARSE_MISSINGDATA;

	// first byte(s) should be message type
	hcp_Uint8 msgTypeHigh = 0;
	hcp_Uint8 msgTypeLow = 0;
	hcp_Uint16 msgType = 0;
	// get the message type (first byte)
	if (R->BytesToUint8(buffer, bytesRead, &msgTypeHigh, AMG3_ENDIANESS) == HCP_NOERROR) {
		// check if we should read a header
		if (msgTypeHigh > 0x7F) {
			if (msgTypeHigh != AMG3_PROTOCOL_EXTENDED) {
				return AMG3_PARSE_INVALIDPROTOCOL;
			}

			hcp_Boolean completeHeader = HCP_FALSE;
			error = amg3_InterpretHeader(R, pSession, bytesRead, &completeHeader);

			if (error < 0) {
				return error;
			}

			bytesRead += error;
			error = HCP_NOERROR;

			if (completeHeader == HCP_FALSE) {
				// we expected a header but didnt receive enough byte(s)
				return AMG3_PARSE_MISSINGDATA;
			}

			// if there was in fact a header, we need to re-read the message type
			if (R->BytesToUint8(buffer, bytesRead, &msgTypeHigh, AMG3_ENDIANESS) != HCP_NOERROR) {
				// we didnt get the message type byte
				return AMG3_PARSE_MISSINGDATA;
			}
		}

		bytesRead++;

		// get the second byte (if two byte-header)
		if (amg3_GetHeaderSize(msgTypeHigh) == 2) {
			if (R->BytesToUint8(buffer, bytesRead, &msgTypeLow, AMG3_ENDIANESS) != HCP_NOERROR) {
				// return 0 bytes consumed
				return AMG3_PARSE_MISSINGDATA;
			}

			msgType = (msgTypeHigh << 8) | (msgTypeLow & 0xFF);

			bytesRead++;
		}
		else {
			msgType = msgTypeHigh;
		}

		error = amg3_InterpretLength(R,pSession, msgType, bytesRead, pCompleteMessage);
	}

	return error;
}

hcp_Int amg3_InterpretByte(hcp_tRuntime* R, const hcp_Uint8 Byte, amg3_tSession* pSession, hcp_Boolean* pCompleteMessage) {
	hcp_tBlob* buffer = &pSession->received;
	hcp_Size_t bytesRead = 0;
	hcp_Int error = AMG3_PARSE_MISSINGDATA;
	*pCompleteMessage = HCP_FALSE;
	pSession->lastSTX = 0;

	// make sure we have enough space for our new byte
	if (sizeof(Byte) + buffer->length > buffer->maxLength) {
		R->Leftshift(R, buffer, sizeof(hcp_Uint8));
		// left shift
	}

	// append the byte
	error = R->AppendByte(Byte, buffer);
	// this should not occur...
	if (error != HCP_NOERROR) {
		return error;
	}

	hcp_Uint8 nextByte = 0;
	// while we got data in the buffer...
	while (R->BytesToUint8(buffer, bytesRead, &nextByte, AMG3_ENDIANESS) == HCP_NOERROR) {
		bytesRead++;

		if (nextByte == AMG3_STX) {

			hcp_Size_t stxPos = bytesRead;
			pSession->lastSTX = stxPos;

			error = amg3_InterpretMessageType(R,pSession, bytesRead, pCompleteMessage);

			if (error == AMG3_PARSE_MISSINGDATA || error == HCP_BLOBOUTOFRANGE || *pCompleteMessage == HCP_TRUE) {
				return HCP_NOERROR;
			}

			// this occurs when a found STX-byte does not actually
			// represent a start of a message so we should try
			// to find a new start
			bytesRead = stxPos;
		}
		else {
			error = AMG3_PARSE_NOSTX;
		}
	}

	return error;
}

hcp_Int amg3_Decode(hcp_tRuntime* pRuntime, hcp_tProtocol* pProtocol, const hcp_tBlob* pSource, hcp_tCommandSet* pCommands, hcp_tCommand** ppCommand, hcp_tBuffer* pContext) {
	// Copy the new data to the receive buffer in order to handle partially
	// received message(s). If the total amount of bytes (already received
	// + newly received) exceeds the buffer size, we will move everything
	// to the left (left shift), ignoring old data
	hcp_Boolean completeMessage = HCP_FALSE;

	hcp_Uint8* source = pSource->value;
	hcp_Size_t length = pSource->length;
	hcp_Int error = 0;
	amg3_tSession* session = (amg3_tSession*)pContext->value;

	while (completeMessage == HCP_FALSE && length > 0) {
		error = amg3_InterpretByte(pRuntime,*source, session, &completeMessage);
		source++; length--;
	}

	if (error == HCP_NOERROR && completeMessage == HCP_TRUE) {
		// clear the receive buffer
		session->received.length = 0;
		*ppCommand = session->command;
	}
	else {
		*ppCommand = HCP_NULL;
	}

	return (hcp_Int)(pSource->length - length);
}

/*
*==============================================================================
* END OF FILE
*==============================================================================
*/
