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
#include "hcp_library.h"
#include "hcp_string.h"
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

static hcp_Int hcp_CompareParameter(void* pLhs, void* pRhs, void* pState);
static hcp_Boolean hcp_IsParameter(void* pValue, void* pState);
static hcp_Int hcp_CompareCommand(void* pLhs, void* pRhs, void* pContext);
static hcp_Boolean hcp_IsCommand(void* pValue, void* pContext);
static hcp_Int hcp_CompareProtocolNode(void* pProtocolNode, void* pName, void* pState);
static hcp_Boolean hcp_IsProtocolNode(void* pProtocolNode, void* pState);
static hcp_Int hcp_CompareCommandTemplate(void* pLhs, void* pRhs, void* pContext);
static hcp_Boolean hcp_IsCommandTemplate(void* pValue, void* pContext);
static hcp_Int hcp_CompareCommandTemplate(void* pLhs, void* pRhs, void* pContext);
static hcp_Boolean hcp_IsCommandTemplate(void* pValue, void* pContext);
static hcp_Int hcp_CompareParameterTemplate(void* pParameterTemplate, void* pName, void* pState);
static hcp_Boolean hcp_IsParameterTemplate(void* pParameterTemplate, void* pState);
static hcp_Int hcp_ToBytes(void* pNumber, const hcp_Size_t NumberOfBytes, hcp_tBlob* pDestination, const hcp_Uint8 Endianess);
static hcp_Int hcp_InitializeParametersFromTemplate(hcp_tState* pState, hcp_tParameterSet* pParameters, hcp_tParameterTemplateSet* pTemplateSet);

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


hcp_Int hcp_AppendZeroes(const hcp_Size_t Length, hcp_tBlob* pDestination) {
	if (pDestination->length + Length > pDestination->maxLength) {
		return HCP_BLOBOUTOFRANGE;
	}

	hcp_Uint8* destination = (hcp_Uint8*)((hcp_Size_t)pDestination->value + pDestination->length);
	hcp_Memset(HCP_NULL, (void*)destination, 0, (hcp_Size_t)Length);

	return HCP_NOERROR;
}

hcp_Int hcp_AppendString(const hcp_tString* pString, hcp_tBlob* pDestination, hcp_Uint8 Encoding) {
	// we might add encodings later...
	if (Encoding != HCP_ENCODING_ASCII) {
		return HCP_INVALIDENCODING;
	}

	// hack until we get proper encoding
	const hcp_Uint8* source = (const hcp_Uint8*)pString->value;
	return hcp_AppendBytes(source, pString->length, pDestination);
}

hcp_Int hcp_AppendByte(const hcp_Uint8 Value, hcp_tBlob* pDestination) {
	if (pDestination->length + 1 > pDestination->maxLength) {
		return HCP_BLOBOUTOFRANGE;
	}

	hcp_Uint8* dest = (hcp_Uint8*)((hcp_Size_t)pDestination->value + pDestination->length * sizeof(hcp_Uint8));
	*dest = Value;
	pDestination->length = pDestination->length + 1;

	return HCP_NOERROR;
}

hcp_Int hcp_AppendBlob(const hcp_tBlob* pSource, hcp_tBlob* pDestination) {
	return hcp_AppendBytes(pSource->value, pSource->length, pDestination);
}

hcp_Int hcp_AppendBytes(const hcp_Uint8* pSource, const hcp_Size_t Length, hcp_tBlob* pDestination) {
	if (pSource == HCP_NULL || Length == 0) {
		return HCP_NOERROR;
	}

	if (Length + pDestination->length >= pDestination->maxLength) {
		return HCP_BLOBOUTOFRANGE;
	}

	hcp_Int error = HCP_NOERROR;
	hcp_Uint8* dest = (hcp_Uint8*)((hcp_Size_t)pDestination->value + pDestination->length);

	hcp_Memcpy(HCP_NULL, dest, pSource, Length);

	pDestination->length += Length;
	return HCP_NOERROR;
}

hcp_Int hcp_CharacterToInt(const hcp_Char Character) {
	if (Character >= 48 && Character <= 57) {
		return Character - 48;
	}

	if (Character >= 65 && Character <= 70) {
		return (Character - 65) + 10;
	}

	return -1;
}

hcp_Int hcp_SetString(hcp_tParameterSet* pParameters, const hcp_tString* Name, const hcp_tString* Value) {
	hcp_Boolean found = HCP_FALSE;
	hcp_Size_t index = hcp_FindFirst(&pParameters->header, 0, (void*)Name, &found);
	hcp_tParameter* parameter = HCP_NULL;
	hcp_Int error = HCP_NOERROR;

	if (found == HCP_FALSE) {
		if ((error = hcp_PushEmpty(&pParameters->header, &index)) != HCP_NULL) {
			return error;
		}

		parameter = (hcp_tParameter*)hcp_ValueAt(&pParameters->header, index);
	}
	else {
		parameter = (hcp_tParameter*)hcp_ValueAt(&pParameters->header, index);
	}

	parameter->value.str = *Value;
	parameter->hasValue = HCP_TRUE;

	return HCP_NOERROR;
}

hcp_Int hcp_GetUint8(const hcp_tProtocol* pProtocol, const hcp_szStr Key, hcp_Uint8* pDest) {
	*pDest = 0;
	hcp_tString key;

	key.value = Key;
	key.length = hcp_szStrLen(Key);
	key.zeroTerm = HCP_TRUE;

	hcp_Boolean found = HCP_FALSE;
	hcp_Size_t index = hcp_FindFirst(&pProtocol->header, 0, &key, &found);

	if (found == HCP_FALSE) {
		return HCP_MISSINGCODECNODE;
	}

	hcp_tProtocolNode* node = (hcp_tProtocolNode*)hcp_ValueAt(&pProtocol->header, index);

	*pDest = (hcp_Uint8)hcp_Atio(&node->value);
	return HCP_NOERROR;
}

hcp_Int hcp_GetUint16(const hcp_tProtocol* pProtocol, const hcp_szStr Key, hcp_Uint16* pDest) {
	*pDest = 0;
	hcp_tString key;

	key.value = Key;
	key.length = hcp_szStrLen(Key);
	key.zeroTerm = HCP_TRUE;

	hcp_Boolean found = HCP_FALSE;

	hcp_Size_t index = hcp_FindFirst(&pProtocol->header, 0, &key, &found);

	if (found == HCP_FALSE) {
		return HCP_MISSINGCODECNODE;
	}

	hcp_tProtocolNode* node = (hcp_tProtocolNode*)hcp_ValueAt(&pProtocol->header, index);

	*pDest = (hcp_Uint16)hcp_Atio(&node->value);
	return HCP_NOERROR;
}

hcp_Int hcp_Resize(hcp_tBuffer* pBuffer, const hcp_Size_t Length) {
	if (Length < sizeof(pBuffer->value)) {
		pBuffer->length = Length;
	}
	else {
		return HCP_BLOBOUTOFRANGE;
	}

	return HCP_NOERROR;
}

void hcp_InitializeBuffer(hcp_tBuffer* pBuffer) {
	pBuffer->length = sizeof(pBuffer->value);
}


hcp_Int hcp_Int16ToBytes(const hcp_Int16 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess) {
	return hcp_ToBytes((void*)&Number, sizeof(hcp_Int16), pDestination, Endianess);
}

hcp_Int hcp_ToBytes(void* pNumber, const hcp_Size_t NumberOfBytes, hcp_tBlob* pDestination, const hcp_Uint8 Endianess) {
	if (pDestination->length + NumberOfBytes >= pDestination->maxLength) {
		return HCP_BLOBOUTOFRANGE;
	}

	hcp_Uint8* destination = pDestination->value + pDestination->length;
	hcp_Size_t position = 0;

	if (Endianess == HCP_BIGENDIAN) {
		for (hcp_Size_t i = 0, j = NumberOfBytes - 1; i < NumberOfBytes; i++, j--) {
			hcp_Uint8 byte = ((hcp_Uint8*)pNumber)[j];
			destination[i] = byte;
		}

		pDestination->length += NumberOfBytes;
	}
	else if (Endianess == HCP_LITTLEENDIAN) {
		for (hcp_Size_t i = 0; i < NumberOfBytes; i++) {
			hcp_Uint8 byte = ((hcp_Uint8*)pNumber)[i];
			destination[i] = byte;
		}

		pDestination->length += NumberOfBytes;
	}
	else {
		return HCP_INVALIDENDIANESS;
	}

	return HCP_NOERROR;
}

hcp_Int hcp_Uint16ToBytes(const hcp_Uint16 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess) {
	return hcp_ToBytes((void*)&Number, sizeof(hcp_Uint16), pDestination, Endianess);
}

hcp_Int hcp_Uint32ToBytes(const hcp_Uint32 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess) {
	return hcp_ToBytes((void*)&Number, sizeof(hcp_Uint32), pDestination, Endianess);
}

hcp_Int hcp_Int32ToBytes(const hcp_Int32 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess) {
	return hcp_ToBytes((void*)&Number, sizeof(hcp_Int32), pDestination, Endianess);
}

hcp_Int hcp_Uint64ToBytes(const hcp_Uint64 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess) {
	return hcp_ToBytes((void*)&Number, sizeof(hcp_Uint64), pDestination, Endianess);
}

hcp_Int hcp_Int64ToBytes(const hcp_Int64 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess) {
	return hcp_ToBytes((void*)&Number, sizeof(hcp_Int64), pDestination, Endianess);
}

hcp_Int hcp_Int8ToBytes(const hcp_Int8 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess) {
	if (pDestination->length + sizeof(hcp_Int8) >= pDestination->maxLength) {
		return HCP_BLOBOUTOFRANGE;
	}

	*(hcp_Uint8*)((hcp_Size_t)pDestination->value + pDestination->length) = (hcp_Uint8)Number;
	pDestination->length++;

	return HCP_NOERROR;
}

hcp_Int hcp_Uint8ToBytes(const hcp_Uint8 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess) {
	return hcp_Int8ToBytes(Number, pDestination, Endianess);
}


extern hcp_Int hcp_BytesToUint8(const hcp_tBlob* pSource, const hcp_Size_t Offset, hcp_Uint8* Number, const hcp_Uint8 Endianess) {
	if ((pSource->length) < (sizeof(hcp_Uint8) + Offset)) {
		return HCP_BLOBOUTOFRANGE;
	}

	*Number = *(hcp_Uint8*)((hcp_Size_t)pSource->value + Offset);
	return HCP_NOERROR;
}
extern hcp_Int hcp_BytesToInt8(const hcp_tBlob* pSource, const hcp_Size_t Offset, hcp_Int8* Number, const hcp_Uint8 Endianess) {
	if ((pSource->length) < (sizeof(hcp_Int8) + +Offset)) {
		return HCP_BLOBOUTOFRANGE;
	}

	*Number = *(hcp_Int8*)((hcp_Size_t)pSource->value + Offset);
	return HCP_NOERROR;
}

extern hcp_Int hcp_BytesToUint16(const hcp_tBlob* pSource, const hcp_Size_t Offset, hcp_Uint16* Number, const hcp_Uint8 Endianess) {
	static const hcp_Size_t byteSize = sizeof(hcp_Uint16);
	const hcp_Uint8* source = (hcp_Uint8*)((hcp_Size_t)pSource->value + Offset);
	hcp_Size_t position = 0;

	*Number = 0;

	if ((pSource->length) < (byteSize + Offset)) {
		return HCP_BLOBOUTOFRANGE;
	}

	if (Endianess == HCP_BIGENDIAN) {
		for (hcp_Size_t i = 0; i < byteSize; i++) {
			*Number |= source[i] << position;
			position += 8;
		}
	}
	else if (Endianess == HCP_LITTLEENDIAN) {
		hcp_Size_t i = byteSize - 1;
		position = 8 * i;


		do {
			*Number |= source[i] << position;
			position -= 8;
		} while (i-- != 0);
	}
	else {
		return HCP_INVALIDENDIANESS;
	}

	return HCP_NOERROR;
}

extern hcp_Int hcp_BytesToInt16(const hcp_tBlob* pSource, const hcp_Size_t Offset, hcp_Int16* Number, const hcp_Uint8 Endianess) {
	static const hcp_Size_t byteSize = sizeof(hcp_Int16);
	const hcp_Uint8* source = (hcp_Uint8*)((hcp_Size_t)pSource->value + Offset);
	hcp_Size_t position = 0;

	*Number = 0;

	if ((pSource->length) < (byteSize + Offset)) {
		return HCP_BLOBOUTOFRANGE;
	}

	if (Endianess == HCP_BIGENDIAN) {
		for (hcp_Size_t i = 0; i < byteSize; i++) {
			*Number |= source[i] << position;
			position += 8;
		}
	}
	else if (Endianess == HCP_LITTLEENDIAN) {
		hcp_Size_t i = byteSize - 1;
		position = 8 * i;


		do {
			*Number |= source[i] << position;
			position -= 8;
		} while (i-- != 0);
	}
	else {
		return HCP_INVALIDENDIANESS;
	}

	return HCP_NOERROR;
}

extern hcp_Int hcp_BytesToUint32(const hcp_tBlob* pSource, const hcp_Size_t Offset, hcp_Uint32* Number, const hcp_Uint8 Endianess) {
	static const hcp_Size_t byteSize = sizeof(hcp_Uint32);
	const hcp_Uint8* source = (hcp_Uint8*)((hcp_Size_t)pSource->value + Offset);
	hcp_Size_t position = 0;

	*Number = 0;

	if ((pSource->length) < (byteSize + Offset)) {
		return HCP_BLOBOUTOFRANGE;
	}

	if (Endianess == HCP_BIGENDIAN) {
		for (hcp_Size_t i = 0; i < byteSize; i++) {
			*Number |= source[i] << position;
			position += 8;
		}
	}
	else if (Endianess == HCP_LITTLEENDIAN) {
		hcp_Size_t i = byteSize - 1;
		position = 8 * i;


		do {
			*Number |= source[i] << position;
			position -= 8;
		} while (i-- != 0);
	}
	else {
		return HCP_INVALIDENDIANESS;
	}

	return HCP_NOERROR;
}

extern hcp_Int hcp_BytesToInt32(const hcp_tBlob* pSource, const hcp_Size_t Offset, hcp_Int32* Number, const hcp_Uint8 Endianess) {
	static const hcp_Size_t byteSize = sizeof(hcp_Int32);
	const hcp_Uint8* source = (hcp_Uint8*)((hcp_Size_t)pSource->value + Offset);
	hcp_Size_t position = 0;

	*Number = 0;

	if ((pSource->length) < (byteSize + Offset)) {
		return HCP_BLOBOUTOFRANGE;
	}

	if (Endianess == HCP_BIGENDIAN) {
		for (hcp_Size_t i = 0; i < byteSize; i++) {
			*Number |= source[i] << position;
			position += 8;
		}
	}
	else if (Endianess == HCP_LITTLEENDIAN) {
		hcp_Size_t i = byteSize - 1;
		position = 8 * i;


		do {
			*Number |= source[i] << position;
			position -= 8;
		} while (i-- != 0);
	}
	else {
		return HCP_INVALIDENDIANESS;
	}

	return HCP_NOERROR;
}

extern hcp_Int hcp_BytesToUint64(const hcp_tBlob* pSource, const hcp_Size_t Offset, hcp_Uint64* Number, const hcp_Uint8 Endianess) {
	static const hcp_Size_t byteSize = sizeof(hcp_Uint64);
	const hcp_Uint8* source = (hcp_Uint8*)((hcp_Size_t)pSource->value + Offset);
	hcp_Size_t position = 0;

	*Number = 0;

	if ((pSource->length) < (byteSize + Offset)) {
		return HCP_BLOBOUTOFRANGE;
	}

	if (Endianess == HCP_BIGENDIAN) {
		for (hcp_Size_t i = 0; i < byteSize; i++) {
			*Number |= source[i] << position;
			position += 8;
		}
	}
	else if (Endianess == HCP_LITTLEENDIAN) {
		hcp_Size_t i = byteSize - 1;
		position = 8 * i;

		do {
			*Number |= source[i] << position;
			position -= 8;
		} while (i-- != 0);
	}
	else {
		return HCP_INVALIDENDIANESS;
	}

	return HCP_NOERROR;
}

extern hcp_Int hcp_BytesToInt64(const hcp_tBlob* pSource, const hcp_Size_t Offset, hcp_Int64* Number, const hcp_Uint8 Endianess) {
	static const hcp_Size_t byteSize = sizeof(hcp_Int64);
	const hcp_Uint8* source = (hcp_Uint8*)((hcp_Size_t)pSource->value + Offset);
	hcp_Size_t position = 0;

	*Number = 0;

	if ((pSource->length) < (byteSize + Offset)) {
		return HCP_BLOBOUTOFRANGE;
	}

	if (Endianess == HCP_BIGENDIAN) {
		for (hcp_Size_t i = 0; i < byteSize; i++) {
			*Number |= source[i] << position;
			position += 8;
		}
	}
	else if (Endianess == HCP_LITTLEENDIAN) {
		hcp_Size_t i = byteSize - 1;
		position = 8 * i;

		do {
			*Number |= source[i] << position;
			position -= 8;
		} while (i-- != 0);
	}
	else {
		return HCP_INVALIDENDIANESS;
	}

	return HCP_NOERROR;
}


hcp_Int hcp_InitializeParameterTemplates(hcp_tState* pState, hcp_tParameterTemplateSet* pParameterTemplates) {
	hcp_Int error = HCP_INITIALIZEVECTOR(pState, &pParameterTemplates->header, pParameterTemplates->fixed,
		hcp_tParameterTemplate, HCP_NULL, hcp_CompareParameterTemplate, hcp_IsParameterTemplate);

	return error;
}

hcp_Int hcp_InitializeCommandTemplate(hcp_tState* pState, hcp_tCommandTemplate* pTemplate) {
	hcp_Int error = HCP_NOERROR;

	error = hcp_InitializeParameterTemplates(pState, &pTemplate->inParameters);

	if (error != HCP_NOERROR) {
		return error;
	}

	error = hcp_InitializeParameterTemplates(pState, &pTemplate->outParameters);

	if (error != HCP_NOERROR) {
		return error;
	}

	error = hcp_InitializeProtocol(pState, &pTemplate->protocol);

	if (error != HCP_NOERROR) {
		return error;
	}

	hcp_Memset(pState, &pTemplate->header, 0, sizeof(hcp_tCommandHeader));

	return error;
}

hcp_Int hcp_InitializeCommandTemplates(hcp_tState* pState, hcp_tCommandTemplateSet* pTemplates) {
	hcp_Int error = HCP_INITIALIZEVECTOR(pState, &pTemplates->header, pTemplates->fixed,
		hcp_tCommandTemplate, HCP_NULL, hcp_CompareCommandTemplate, hcp_IsCommandTemplate);

	return error;
}

hcp_Int hcp_InitializeProtocol(hcp_tState* pState, hcp_tProtocol* pProtocol) {
	hcp_Int error = HCP_INITIALIZEVECTOR(pState, &pProtocol->header, pProtocol->fixed,
		hcp_tProtocolNode, HCP_NULL, hcp_CompareProtocolNode, hcp_IsProtocolNode);

	return error;
}

hcp_Int hcp_InitializeCommands(hcp_tState* pState, hcp_tCommandSet* pCommands, hcp_tModel* pTemplate) {
	hcp_Int error = HCP_INITIALIZEVECTOR(pState, &pCommands->header, pCommands->fixed,
		hcp_tCommand, HCP_NULL, hcp_CompareCommand, hcp_IsCommand);

	const hcp_Size_t length = pTemplate->commands.header.length;
	hcp_Size_t i = 0;
	for (i = 0; i < length; i++) {
		hcp_Size_t commandIndex = 0;

		error = hcp_PushEmpty(&pCommands->header, &commandIndex);

		if (error != HCP_NOERROR) {
			return error;
		}

		hcp_tCommand* command = (hcp_tCommand*)hcp_ValueAt(&pCommands->header, commandIndex);
		hcp_tCommandTemplate* t = (hcp_tCommandTemplate*)hcp_ValueAt(&pTemplate->commands.header, i);

		command->template_ = t;
		// create in-parameters
		error = hcp_InitializeParametersFromTemplate(pState, &command->inParams, &t->inParameters);

		if (error != HCP_NOERROR) {
			hcp_Pop(&pCommands->header, commandIndex);
			return error;
		}

		//create out-parameters
		error = hcp_InitializeParametersFromTemplate(pState, &command->outParams, &t->outParameters);

		if (error != HCP_NOERROR) {
			hcp_Pop(&pCommands->header, commandIndex);
			return error;
		}
	}

	return error;
}

hcp_Int hcp_InitializeParameters(hcp_tState* pState, hcp_tParameterSet* pParameters) {
	hcp_Size_t elementSize = sizeof(hcp_tParameter);
	hcp_Size_t arraySize = sizeof(pParameters->fixed);
	hcp_Size_t fixedCount = arraySize / elementSize;

	hcp_Size_t maxCount = hcp_IsDynamic(pState) == HCP_TRUE ? HCP_MAXSIZE_DYNAMIC : fixedCount;

	hcp_InitializeVector(pState, &pParameters->header, pParameters->fixed, elementSize, maxCount, HCP_NULL,
		hcp_CompareParameter, hcp_IsParameter, pState);

	return HCP_NOERROR;
}

hcp_Int hcp_InitializeParametersFromTemplate(hcp_tState* pState, hcp_tParameterSet* pParameters, hcp_tParameterTemplateSet* pTemplateSet) {
	hcp_Int error = HCP_NOERROR;

	error = hcp_InitializeParameters(pState, pParameters);

	if (error != HCP_NOERROR) {
		return error;
	}

	const hcp_Size_t templateCount = pTemplateSet->header.length;
	// for each template, create a new parameter
	hcp_Size_t templateIndex = 0;

	for (templateIndex = 0; templateIndex < templateCount; templateIndex++) {
		hcp_Size_t parameterIndex = 0;

		error = hcp_PushEmpty(&pParameters->header, &parameterIndex);

		if (error != HCP_NOERROR) {
			break;
		}

		hcp_tParameter* parameter = (hcp_tParameter*)hcp_ValueAt(&pParameters->header, parameterIndex);
		hcp_tParameterTemplate* t = (hcp_tParameterTemplate*)hcp_ValueAt(&pTemplateSet->header, templateIndex);

		parameter->template_ = t;
		parameter->hasValue = HCP_FALSE;
		parameter->value.i = 0;
	}

	return error;
}

hcp_Int hcp_CompareParameterTemplate(void* pParameterTemplate, void* pName, void* pState) {
	hcp_tParameterTemplate* t = (hcp_tParameterTemplate*)pParameterTemplate;
	hcp_tString* name = (hcp_tString*)pName;

	return hcp_tStrCmp(&t->name, name);
}

hcp_Boolean hcp_IsParameterTemplate(void* pParameterTemplate, void* pState) {
	hcp_tParameterTemplate* t = (hcp_tParameterTemplate*)pParameterTemplate;
	return t->name.length == 0 ? HCP_FALSE : HCP_TRUE;
}

hcp_Int hcp_CompareProtocolNode(void* pProtocolNode, void* pName, void* pState) {
	hcp_tProtocolNode* node = (hcp_tProtocolNode*)pProtocolNode;
	hcp_tString* name = (hcp_tString*)pName;

	return hcp_tStrCmp(&node->key, name);
}

hcp_Boolean hcp_IsProtocolNode(void* pProtocolNode, void* pState) {
	hcp_tProtocolNode* node = (hcp_tProtocolNode*)pProtocolNode;
	return node->key.length == 0 ? HCP_FALSE : HCP_TRUE;
}

hcp_Int hcp_CompareParameter(void* pLhs, void* pRhs, void* pState) {
	hcp_tParameter* parameter = (hcp_tParameter*)pLhs;
	hcp_tString* name = (hcp_tString*)pRhs;

	hcp_Int equals = 0;
	hcp_tParameterTemplate* t = parameter->template_;

	if (t->name.length < name->length) {
		equals = -1;
	}
	else if (t->name.length > name->length) {
		equals = 1;
	}
	else {
		hcp_Size_t position = 0;

		while (position < name->length) {
			if (t->name.value[position] != name->value[position]) {
				equals = 1;
				break;
			}

			position++;
		}
	}

	return equals;
}

hcp_Boolean hcp_IsParameter(void* pValue, void* pState) {
	hcp_tParameter* parameter = (hcp_tParameter*)pValue;
	return parameter->template_ == 0 ? HCP_FALSE : HCP_TRUE;
}

hcp_Int hcp_CompareCommand(void* pLhs, void* pRhs, void* pContext) {
	hcp_tCommand* lhs = (hcp_tCommand*)pLhs;
	hcp_tCommandHeader* rhs = (hcp_tCommandHeader*)pRhs;

	hcp_Int equals = hcp_tStrCmp(&lhs->template_->header.command, &rhs->command);

	if (equals != 0) {
		return equals;
	}

	return hcp_tStrCmp(&lhs->template_->header.family, &rhs->family);
}

hcp_Boolean hcp_IsCommand(void* pValue, void* pContext) {
	hcp_tCommand* command = (hcp_tCommand*)pValue;
	return command->template_ == 0 ? HCP_FALSE : HCP_TRUE;
}

hcp_Int hcp_CompareCommandTemplate(void* pLhs, void* pRhs, void* pContext) {
	hcp_tCommandTemplate* lhs = (hcp_tCommandTemplate*)pLhs;
	hcp_tCommandHeader* rhs = (hcp_tCommandHeader*)pRhs;

	hcp_Int equals = hcp_tStrCmp(&lhs->header.command, &rhs->command);

	if (equals != 0) {
		return equals;
	}

	return hcp_tStrCmp(&lhs->header.family, &rhs->family);
}

hcp_Boolean hcp_IsCommandTemplate(void* pValue, void* pContext) {
	hcp_tCommandTemplate* command = (hcp_tCommandTemplate*)pValue;
	return command->header.command.length == 0 ? HCP_FALSE : HCP_TRUE;
}

hcp_Int HCP_CALL hcp_AppendParameters(hcp_tRuntime* R, hcp_tBlob* pDestination, hcp_tParameterSet* pParameters, const hcp_Uint8 Endianess, hcp_EncodeString StringEncoder, void* pContext) {
	const hcp_Size_t length = pParameters->header.length;
	hcp_Int error = HCP_NOERROR;


	for (hcp_Size_t i = 0; i < length; i++) {
		hcp_tParameter* parameter = (hcp_tParameter*)hcp_ValueAt(&pParameters->header, i);

		switch (parameter->template_->type) {
		case HCP_BOOLEAN_ID:
		case HCP_INT8_ID:
		case HCP_UINT8_ID: {
			error = hcp_AppendByte(parameter->value.u8, pDestination);
		}break;
		case HCP_UINT16_ID: {
			error = hcp_Uint16ToBytes(parameter->value.u16, pDestination, Endianess);
		}break;
		case HCP_INT16_ID: {
			error = hcp_Int16ToBytes(parameter->value.i16, pDestination, Endianess);
		}break;
		case HCP_UNIXTIME_ID:
		case HCP_UINT32_ID: {
			error = hcp_Uint32ToBytes(parameter->value.u32, pDestination, Endianess);
		}break;
		case HCP_INT32_ID: {
			error = hcp_Int32ToBytes(parameter->value.i32, pDestination, Endianess);
		}break;
		case HCP_UINT64_ID: {
			error = hcp_Uint64ToBytes(parameter->value.u64, pDestination, Endianess);
		}break;
		case HCP_INT64_ID: {
			error = hcp_Int64ToBytes(parameter->value.i64, pDestination, Endianess);
		}break;
		case HCP_STRING_ID: {
			if (StringEncoder != HCP_NULL) {
				error = StringEncoder(R,parameter->template_, &parameter->value.str, pDestination, pContext);
			}
			else {
				const hcp_Size_t fixedLength = parameter->template_->length;
				const hcp_Size_t valueLength = parameter->value.str.length;

				if (fixedLength != 0 && fixedLength < valueLength) {
					error = HCP_INVALID_STRINGSIZE;
				}
				else {
					error = hcp_AppendString(&parameter->value.str, pDestination, HCP_ENCODING_ASCII);

					if (error == HCP_NOERROR && fixedLength > valueLength) {
						// add padding so we write the number of byte(s) specified in parameter-length
						error = hcp_AppendZeroes(fixedLength - valueLength, pDestination);
					}
				}
			}
		} break;
		case HCP_BLOB_ID: {
			error = hcp_AppendBlob(&parameter->value.blb, pDestination);
		} break;
		case HCP_SIMPLEVERSION_ID: {
			error = hcp_Uint16ToBytes(parameter->value.i16, pDestination, Endianess);
		} break;
		default: {
			error = HCP_INVALIDTYPEID;
		} break;
		}

		if (error != HCP_NOERROR) {
			break;
		}
	}

	return error;
}

void HCP_CALL hcp_Leftshift(hcp_tRuntime* R, hcp_tBlob* pSource, const hcp_Size_t Steps) {
	if (pSource->length < Steps) {
		return;
	}

	const hcp_Uint8* source = (hcp_Uint8*)((hcp_Size_t)pSource->value + Steps);

	hcp_Memcpy((hcp_tState*)R->context, pSource->value, source, pSource->length - Steps);

	pSource->length -= Steps;
}

hcp_Int HCP_CALL hcp_BytesToParameters(hcp_tRuntime* R, const hcp_tBlob* pSource, const hcp_Size_t Offset, hcp_tParameterSet* pParameters, const hcp_Uint8 Endianess, hcp_DecodeString StringDecoder, void* pContext) {
	hcp_Size_t i = 0;
	const hcp_Size_t parameterCount = pParameters->header.length;
	hcp_Size_t bytesRead = 0;

	for (i = 0; i < parameterCount; i++) {
		hcp_tParameter* parameter = (hcp_tParameter*)hcp_ValueAt(&pParameters->header, i);

		const hcp_Int typeId = parameter->template_->type;
		hcp_Size_t typeSize = hcp_GetTypeSize(typeId);

		hcp_Int error = HCP_NOERROR;
		 
		switch (typeId)
		{
			case HCP_BOOLEAN_ID: {
				hcp_Uint8 value = 0;

				error = hcp_BytesToUint8(pSource, Offset + bytesRead, &value, Endianess);

				if (error == HCP_NOERROR) {
					parameter->value.b = (value != HCP_FALSE) ? HCP_TRUE : HCP_FALSE;
				}
			} break;
			case HCP_UINT8_ID: {
				error = hcp_BytesToUint8(pSource, Offset + bytesRead, &parameter->value.u8, Endianess);
			} break;
			case HCP_INT8_ID: {
				error = hcp_BytesToInt8(pSource, Offset + bytesRead, &parameter->value.s8, Endianess);
			} break;
			case HCP_SIMPLEVERSION_ID:
			case HCP_UINT16_ID: {
				error = hcp_BytesToUint16(pSource, Offset + bytesRead, &parameter->value.u16, Endianess);
			} break;
			case HCP_INT16_ID: {
				error = hcp_BytesToInt16(pSource, Offset + bytesRead, &parameter->value.i16, Endianess);
			} break;
			case HCP_UNIXTIME_ID:
			case HCP_UINT32_ID: {
				error = hcp_BytesToUint32(pSource, Offset + bytesRead, &parameter->value.u32, Endianess);
			} break;
			case HCP_INT32_ID: {
				error = hcp_BytesToInt32(pSource, Offset + bytesRead, &parameter->value.i32, Endianess);
			} break;
			case HCP_UINT64_ID: {
				error = hcp_BytesToUint64(pSource, Offset + bytesRead, &parameter->value.u64, Endianess);
			} break;
			case HCP_INT64_ID: {
				error = hcp_BytesToInt64(pSource, Offset + bytesRead, &parameter->value.i64, Endianess);
			} break;
			case HCP_BLOB_ID: {
				if (parameter->template_->length < 1) {
					// no length specified, the byte-array must be the last
					// parameter in the list since we will consume the remaning byte(s)
					if (i + 1 < parameterCount) {
						return HCP_MISSINGBYTEARRAY_SIZE;
					}

					typeSize = pSource->length - (Offset + bytesRead);
				}
				else {
					typeSize = parameter->template_->length;
				}

				parameter->value.blb.maxLength = typeSize;
				parameter->value.blb.length = typeSize;
				parameter->value.blb.value = (hcp_Uint8*)((hcp_Size_t)pSource->value + Offset + bytesRead);
			} break;
			case HCP_STRING_ID: {
				if (StringDecoder != HCP_NULL) {
					typeSize = StringDecoder(R, pSource, parameter->template_, Offset + bytesRead, &parameter->value.str, pContext);

					if (typeSize < 1) {
						return typeSize;
					}
				}
				else {
					hcp_Int32 templateLength = parameter->template_->length;
					hcp_Char* str = (hcp_Char*)((hcp_Size_t)pSource->value + Offset + bytesRead);
					hcp_tString* pString = &parameter->value.str;

					// fixed size
					if (templateLength > 0) {
						// the string may contain \0 in which case 
						// the specified length is not valid...
						pString->zeroTerm = HCP_FALSE;
						pString->length = templateLength;

						hcp_Int32 end = 0;

						for (hcp_Int32 end = 0; end < (hcp_Int32)templateLength; end++) {
							if (str[end] == 0) {
								pString->length = end;
								pString->zeroTerm = HCP_TRUE;
								break;
							}
						}

						pString->value = str;
					}
					// zero-terminated
					else if (templateLength == 0) {
						pString->length = R->szStrLen((hcp_szStr)str);
						pString->value = str;
						pString->zeroTerm = HCP_TRUE;
						typeSize = pString->length;
					}
					// consume the remaning byte(s)
					else {
						pString->length = pSource->length - Offset;
						pString->value = str;
						pString->zeroTerm = HCP_FALSE;

						for (hcp_Int32 end = 0; end < pString->length; end++) {
							if (str[end] == 0) {
								pString->length = end;
								pString->zeroTerm = HCP_TRUE;
								break;
							}
						}
					}
					// move past the string
					typeSize = pString->length;
					// move past the trailing \0 if avalible
					if (pString->zeroTerm) {
						typeSize++;
					}
				}
			} break;

			default: {
				return HCP_INVALIDTYPEID;
			}
		}

		bytesRead += typeSize;;
	}

	return bytesRead;
}

hcp_Size_t HCP_CALL hcp_GetParameterSetSize(const hcp_tParameterSet* pParameters) {
	const hcp_Size_t length = pParameters->header.length;
	hcp_Size_t dataSize = 0;

	for (hcp_Size_t i = 0; i < length; i++) {
		hcp_tParameter* parameter = (hcp_tParameter*)hcp_ValueAt(&pParameters->header, i);
		const hcp_Uint8 typeId = parameter->template_->type;

		if (typeId == HCP_STRING_ID) {
			if (parameter->template_->length > 0) {
				dataSize += parameter->template_->length;
			}
			else {
				dataSize += parameter->value.str.length;
			}
		}
		else if (typeId == HCP_BLOB_ID) {
			dataSize += parameter->value.blb.length;
		}
		else {
			dataSize += hcp_GetTypeSize(typeId);
		}
	}

	return dataSize;
}

/*
*==============================================================================
*  5.   LOCAL FUNCTIONS (declared in Section 3.5)
*==============================================================================
*/

/*
*==============================================================================
* END OF FILE
*==============================================================================
*/
