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
#include "hcp_error.h"
#include "hcp_tif.h"
#include "hcp_string.h"
#include "hcp_codec.h"
#include "hcp_library.h"
#include "cJSON.h"

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

typedef cJSON hcp_tJSON;	/* tried to typedef-away library dependency */

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

static hcp_Int hcp_LoadCommandTemplate(hcp_tState* pState, cJSON* pMethod, cJSON* pTypeArray, hcp_tCommandTemplate* pTemplate);
static hcp_Int hcp_LoadMethods(hcp_tState* pState, cJSON* pObject, hcp_tCommandTemplateSet* pTemplates);
static hcp_Int hcp_LoadHeader(hcp_tState* pState, hcp_tJSON* pRoot, hcp_tModel* pTemplate);
static hcp_Int hcp_LoadParameterTemplates(hcp_tState* pState, cJSON* pArray, cJSON* pTypeArray, hcp_tParameterTemplateSet* pParameters);
static hcp_Int hcp_GetString(cJSON* N, hcp_cszStr Name, hcp_tString* pDestination);
static hcp_Boolean hcp_LoadCommandHeader(cJSON* pObject, hcp_tCommandHeader* pHeader);
static hcp_tString hcp_NextString(hcp_tString* pSource, char Terminator, hcp_Boolean EndOnEndOfString);
static hcp_Int hcp_ParseTIFHeader(hcp_tString* pCommand, hcp_tCommandHeader* pHeader);
static hcp_Int hcp_ParseArgumentName(hcp_tString* pArguments, hcp_tParameterSet* pParameters, hcp_tParameter** ppResult);
static hcp_Int hcp_LoadProtocol(hcp_tState* pState, cJSON* pObject, hcp_tProtocol* pProtocol);
static hcp_Int hcp_ParseArgumentValue(hcp_tString* pValue, hcp_tParameter* pParameter);
static hcp_Int hcp_ParseStringValue(hcp_tString* pValue, hcp_tParameter* pParameter, hcp_Int32 ExpectedLength);
static hcp_Int hcp_ParseByteArray(hcp_tString* pValue, hcp_tParameter* pParameter);
static void hcp_IgnoreSpace(hcp_tString* pString);

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

hcp_Int hcp_ParseTIFArguments(hcp_tString* pArguments, hcp_tCommand* pCommand) {
	const hcp_Size_t maxLength = pArguments->length;
	hcp_Int error = HCP_NOERROR;

	hcp_IgnoreSpace(pArguments);

	while (pArguments->length > 0) {
		// no args?
		if (*pArguments->value == ')') {
			break;
		}

		hcp_tParameter* parameter = HCP_NULL;

		// get the argument (parameter)
		error = hcp_ParseArgumentName(pArguments, &pCommand->inParams, &parameter);

		if (error != HCP_NOERROR) {
			return error;
		}

		// populate the argument value
		error = hcp_ParseArgumentValue(pArguments, parameter);

		if (error != HCP_NOERROR) {
			return error;
		}

		hcp_IgnoreSpace(pArguments);

		if (*pArguments->value == ')') {
			break;
		}
		else if (*pArguments->value == ',') {
			pArguments->value++;
			pArguments->length--;
		}
		else {
			return HCP_MISSINGENDOFCOMMAND;
		}
	}

	return HCP_NOERROR;
}

hcp_Int hcp_ParseTifCommand(const hcp_tString* TIFCommand, hcp_tCommandSet* pCommands, hcp_tCommand** ppCommand) {
	// lets skip empty strings
	if (TIFCommand->length == 0) {
		*ppCommand = HCP_NULL;
		return HCP_NOERROR;
	}

	hcp_Size_t position = 0;
	hcp_tCommandHeader header;
	hcp_tString string = *TIFCommand;
	hcp_Int error = HCP_NOERROR;

	// get family.command
	if ((error = hcp_ParseTIFHeader(&string, &header)) != HCP_NOERROR) {
		return error;
	}

	hcp_Boolean found = HCP_FALSE;
	// find the command
	hcp_Size_t index = hcp_FindFirst(&pCommands->header, 0, &header, &found);

	if (found == HCP_FALSE) {
		return HCP_COMMANDNOTLOADED;
	}

	hcp_tCommand* output = (hcp_tCommand*)hcp_ValueAt(&pCommands->header, index);
	// parse argument(s)
	error = hcp_ParseTIFArguments(&string, output);

	if (error != HCP_NOERROR) {
		output = HCP_NULL;
	}

	*ppCommand = output;
	return HCP_NOERROR;
}

hcp_Int hcp_LoadTIFTemplate(hcp_tState* pState, const hcp_tString* pText, hcp_tModel* pTemplate) {
	if (pText->zeroTerm == HCP_FALSE) {
		return HCP_NONSZSTRING;
	}

	hcp_Int error = HCP_NOERROR;
	cJSON* root = cJSON_Parse(pText->value);

	if (root == HCP_NULL) {
		const char* result = cJSON_GetErrorPtr();
		return HCP_PARSEJSONFAILED;
	}

	error = hcp_LoadHeader(pState, root, pTemplate);

	if (error != HCP_NOERROR) {
		return error;
	}

	error = hcp_LoadMethods(pState, root, &pTemplate->commands);

	if (error != HCP_NOERROR) {
		return error;
	}

	if (error != HCP_NOERROR) {
		cJSON_Delete(root);
		root = HCP_NULL;
	}

	pTemplate->cache = root;
	return error;
}

/*
*==============================================================================
*  5.   LOCAL FUNCTIONS (declared in Section 3.5)
*==============================================================================
*/

void hcp_IgnoreSpace(hcp_tString* pString) {
	while (*pString->value == ' ' || *pString->value == '\t') {
		pString->value++;
		pString->length--;
	}
}

hcp_Int hcp_ParseArgumentName(hcp_tString* pArguments, hcp_tParameterSet* pParameters, hcp_tParameter** ppResult) {
	hcp_IgnoreSpace(pArguments);
	hcp_tString argumentName = hcp_NextString(pArguments, ':', HCP_FALSE);

	if (argumentName.length == 0 || argumentName.value == HCP_NULL) {
		return HCP_NOARGUMENTNAME;
	}

	hcp_Boolean found = HCP_FALSE;
	// try to tind the parameter in the provided list
	hcp_Size_t index = hcp_FindFirst(&pParameters->header, 0, &argumentName, &found);

	if (found == HCP_FALSE) {
		return HCP_NOMATCHINGPARAMETER;
	}

	*ppResult = (hcp_tParameter*)hcp_ValueAt(&pParameters->header, index);
	return HCP_NOERROR;
}

hcp_Int hcp_ParseStringValue(hcp_tString* pValue, hcp_tParameter* pParameter, hcp_Int32 ExpectedLength) {
	hcp_Boolean quoteStart = HCP_FALSE;
	// if the first sign is a " lets skip it
	if (pValue->length > 0 && *pValue->value == '\"') {
		pValue->value++;
		pValue->length--;
		quoteStart = HCP_TRUE;
	}

	hcp_Char const* end = pValue->value;
	hcp_Char const* start = pValue->value;
	hcp_Boolean terminated = HCP_FALSE;

	if (quoteStart == HCP_TRUE) {
		while (pValue->length > 0) {
			if (*pValue->value == '\"') {
				pValue->value++;
				pValue->length--;
				terminated = HCP_TRUE;
				break;
			}

			end = pValue->value;

			pValue->value++;
			pValue->length--;
		}
	}
	else {
		while (pValue->length > 0) {
			if (*pValue->value == ',' || *pValue->value == ')') {
				terminated = HCP_TRUE;
				break;
			}

			end = pValue->value;

			pValue->value++;
			pValue->length--;
		}
	}

	if (terminated == HCP_FALSE) {
		return HCP_MISSINGTERMINATOR;
	}

	pParameter->value.str.length = (hcp_Size_t)((hcp_Size_t)end - (hcp_Size_t)start) + sizeof(hcp_Char);
	pParameter->value.str.zeroTerm = HCP_FALSE;
	pParameter->value.str.value = start;

	return HCP_NOERROR;
}

hcp_Int hcp_ParseByteArray(hcp_tString* pValue, hcp_tParameter* pParameter) {
	// if the first sign is a " lets skip it
	if (pValue->length > 0 && *pValue->value == '\"') {
		pValue->value++;
		pValue->length--;
	}

	hcp_Char const* end = pValue->value;
	hcp_Char const* start = pValue->value;
	hcp_Size_t i = 1;

	hcp_Uint8* dest = (hcp_Uint8*)pValue->value;

	while (pValue->length > 0) {
		if (*pValue->value == ',' || *pValue->value == ' ' || *pValue->value == ')') {
			break;
		}

		end = pValue->value;

		if (i % 2 == 0) {
			hcp_Uint8 value = 0;
			hcp_Int index = i - 2;
			value = hcp_CharacterToInt(pValue->value[-1]) * 16;
			value += hcp_CharacterToInt(pValue->value[0]);

			dest[i / 2] = value;
		}

		pValue->value++;
		pValue->length--;
		i++;
	}

	pParameter->value.blb.value = dest;
	pParameter->value.blb.length = i / 2;
	pParameter->value.blb.maxLength = i / 2;

	return HCP_NOERROR;
}

hcp_Int hcp_ParseDigitValue(hcp_tString* pValue, hcp_tParameter* pParameter) {
	hcp_Size_t length = 0;
	hcp_Size_t valueLength = 0;
	hcp_Int error = HCP_NOERROR;

	hcp_IgnoreSpace(pValue);
	hcp_Char const* start = pValue->value;
	hcp_Char const* end = pValue->value;

	while (pValue->length > 0) {
		if (*pValue->value == ',' || *pValue->value == ')') {
			break;
		}

		if (hcp_IsDigit(*pValue->value) == HCP_FALSE) {
			return HCP_MISSINGDIGIT;
		}

		hcp_IgnoreSpace(pValue);

		pValue->value++;
		pValue->length--;
		end = pValue->value;
	}

	if (end - start == 0) {
		// special case, value is set to zero
		pParameter->value.i = 0;
	}
	else {
		hcp_tString value;

		value.value = start;
		value.length = ((hcp_Size_t)end - (hcp_Size_t)start);
		value.zeroTerm = HCP_FALSE;

		hcp_Int result  = hcp_Atio(&value);
		pParameter->value.i = result;
	}

	return error;
}

hcp_Int hcp_ParseArgumentValue(hcp_tString* pValue, hcp_tParameter* pParameter) {
	hcp_Int error = HCP_NOERROR;

	if (pParameter->template_->type == HCP_BLOB_ID) {
		hcp_IgnoreSpace(pValue);
		error = hcp_ParseByteArray(pValue, pParameter);
	} else if (pParameter->template_->type == HCP_STRING_ID) {
		// parse string element
		error = hcp_ParseStringValue(pValue, pParameter, pParameter->template_->length);
	}
	else if (hcp_IsDigit(*pValue->value) == HCP_TRUE) {
		hcp_IgnoreSpace(pValue);
		error = hcp_ParseDigitValue(pValue, pParameter);
	}
	else {
		error = HCP_INVALIDARGUMENTVALUE;
	}

	return error;
}

hcp_Int hcp_LoadProtocol(hcp_tState* pState, cJSON* pRoot, hcp_tProtocol* pProtocol) {
	hcp_Int error = hcp_InitializeProtocol(pState, pProtocol);

	if (error != HCP_NULL) {
		return error;
	}

	if (pRoot == HCP_NULL) {
		return HCP_NOERROR;
	}

	cJSON* array = cJSON_GetObjectItem(pRoot, HCP_TIF_PROTOCOL);

	if (array == HCP_NULL || array->type != cJSON_Array) {
		return HCP_NOERROR;
	}
	const hcp_Int length = cJSON_GetArraySize(array);

	for (hcp_Int i = 0; i < length; i++) {
		cJSON* item = cJSON_GetArrayItem(array, i);

		hcp_Size_t index = 0;

		if ((error = hcp_PushEmpty(&pProtocol->header, &index)) != HCP_NOERROR) {
			return error;
		}

		hcp_tProtocolNode* node = (hcp_tProtocolNode*)hcp_ValueAt(&pProtocol->header, index);

		hcp_GetString(item, HCP_TIF_PROTOCOLKEY,&node->key);
		hcp_GetString(item, HCP_TIF_PROTOCOLVALUE, &node->value);

		if (node->key.length == 0) {
			return HCP_MISSINGKEY;
		}
	}

	return HCP_NOERROR;
}

hcp_Int hcp_ParseTIFHeader(hcp_tString* pCommand, hcp_tCommandHeader* pHeader) {
	hcp_Size_t position = 0;

	pHeader->family = hcp_NextString(pCommand, HCP_TIF_SEPARATOR, HCP_FALSE);

	if (pHeader->family.length == 0) {
		return HCP_MISSINGFAMILY;
	}

	pHeader->command = hcp_NextString(pCommand, '(', HCP_FALSE);

	if (pHeader->command.length == 0) {
		return HCP_MISSINGCOMMAND;
	}

	return HCP_NOERROR;
}

hcp_tString hcp_NextString(hcp_tString* pString,char Terminator, hcp_Boolean EndOnEndOfString) {
	hcp_IgnoreSpace(pString);

	hcp_Boolean complete = EndOnEndOfString;
	hcp_Char const* start = pString->value;
	hcp_Char const* end = pString->value;

	while (pString->length > 0) {
		hcp_Char character = *pString->value;

		if ((hcp_IsAlphaNumerical(character) == 0 && hcp_IsDigit(character) == 0) || character == Terminator) {
			end = pString->value;

			hcp_IgnoreSpace(pString);
			character = *pString->value;

			if (character == Terminator) {
				pString->value++;
				pString->length--;
				break;
			}
			else {
				hcp_tString empty;

				empty.value = HCP_NULL;
				empty.length = 0;
				empty.zeroTerm = HCP_TRUE;

				return empty;
			}
		}

		pString->value++;
		pString->length--;
	}

	hcp_tString output;

	output.value = start;
	output.length = ((hcp_Size_t)end - (hcp_Size_t)start);
	output.zeroTerm = HCP_FALSE;

	return output;
}

hcp_Boolean hcp_LoadCommandHeader(cJSON* pObject, hcp_tCommandHeader* pHeader) {
	if (pObject == HCP_NULL) {
		return HCP_FALSE;
	}


	return HCP_TRUE;
}

hcp_Int hcp_LoadCommandTemplate(hcp_tState* pState, cJSON* pMethod, cJSON* pTypeArray, hcp_tCommandTemplate* pTemplate) {
	hcp_Int error = HCP_NOERROR;

	error = hcp_InitializeCommandTemplate(pState, pTemplate);

	if (error != HCP_NOERROR) {
		return error;
	}

	if (pMethod == HCP_NULL || pMethod->type != cJSON_Object) {
		return HCP_NOERROR;
	}

	hcp_GetString(pMethod, HCP_TIF_COMMAND, &pTemplate->header.command);
	hcp_GetString(pMethod, HCP_TIF_FAMILY, &pTemplate->header.family);


	error = hcp_LoadParameterTemplates(pState, cJSON_GetObjectItem(pMethod, HCP_TIF_INPARAMS), pTypeArray, &pTemplate->inParameters);

	if (error != HCP_NOERROR) {
		return error;
	}

	error = hcp_LoadParameterTemplates(pState, cJSON_GetObjectItem(pMethod, HCP_TIF_OUTPARAMS), pTypeArray, &pTemplate->outParameters);

	if (error != HCP_NOERROR) {
		return error;
	}

	error = hcp_LoadProtocol(pState, pMethod, &pTemplate->protocol);

	return error;
}

hcp_Int hcp_LoadMethods(hcp_tState* pState, cJSON* pObject,hcp_tCommandTemplateSet* pTemplates) {
	hcp_Int error = HCP_NOERROR;

	error = hcp_InitializeCommandTemplates(pState, pTemplates);

	if (error != HCP_NOERROR) {
		return error;
	}

	cJSON* methods = cJSON_GetObjectItem(pObject, HCP_TIF_METHOD);

	if (methods == HCP_NULL) {
		return HCP_NOERROR;
	}

	// type-array, used to resolve from complex to simple built-in types
	cJSON* types = cJSON_GetObjectItem(pObject, HCP_TIF_TYPES);

	hcp_Int length = cJSON_GetArraySize(methods);

	for (hcp_Int i = 0; i < length; i++) {
		cJSON* item = cJSON_GetArrayItem(methods, i);

		if (item == HCP_NULL || item->type != cJSON_Object) {
			continue;
		}

		hcp_Size_t index = -1;

		error = hcp_PushEmpty(&pTemplates->header, &index);

		if (error != HCP_NOERROR) {
			return error;
		}

		hcp_tCommandTemplate* t = (hcp_tCommandTemplate*)hcp_ValueAt(&pTemplates->header, index);
		error = hcp_LoadCommandTemplate(pState, item, types, t);

		if (error != HCP_NOERROR) {
			hcp_Pop(&pTemplates->header, index);
			return error;
		}
	}

	return HCP_NOERROR;
}

hcp_Int hcp_GetString(cJSON* N, hcp_cszStr Name, hcp_tString* pDestination) {
	cJSON* node = cJSON_GetObjectItem(N, (const char*)Name);

	if (node == HCP_NULL || node->type != cJSON_String) {
		pDestination->value = HCP_NULL;
		pDestination->length = 0;
		pDestination->zeroTerm = HCP_TRUE;

		return HCP_MISSINGSTRING;
	}

	pDestination->value = node->valuestring;
	pDestination->length = hcp_szStrLen(node->valuestring);
	pDestination->zeroTerm = HCP_TRUE;

	return HCP_NOERROR;
}

hcp_Int hcp_GetBasicType(const hcp_tString* pTypeName, cJSON* pArray) {
	hcp_Int typeId = hcp_GetTypeId(pTypeName);

	if (typeId != HCP_TYPE_INVALID || pArray == HCP_NULL || pArray->type != cJSON_Array) {
		return typeId;
	}

	const hcp_Int arraySize = cJSON_GetArraySize(pArray);

	for (hcp_Int i = 0; i < arraySize ; i++) {
		cJSON* item = cJSON_GetArrayItem(pArray, i);

		hcp_tString name;
		hcp_tString type;

		if (hcp_GetString(item, HCP_TIF_TYPENAME, &name) != HCP_NOERROR) {
			continue;
		}

		if (hcp_tStrCmp(&name, pTypeName) != 0) {
			continue;
		}

		if (hcp_GetString(item, HCP_TIF_TYPETYPE, &type) != HCP_NOERROR) {
			return HCP_TYPE_INVALID;
		}

		// lets make sure we actually got a value...
		if (type.length == 0) {
			return HCP_TYPE_INVALID;
		}

		// if the located type is not a built in we do a recursive search
		typeId = hcp_GetTypeId(&type);
		// the type was not a built in type...
		if (typeId != HCP_TYPE_INVALID) {
			return typeId;
		}
		// make sure we dont reference to ourselfs
		if (hcp_tStrCmp(&name, &type) == 0) {
			return HCP_TYPE_INVALID;
		}
		// try to find the built in type
		return hcp_GetBasicType(&type, pArray);
	}

	return HCP_TYPE_INVALID;
}

hcp_Int hcp_LoadParameterTemplates(hcp_tState* pState, cJSON* pArray,cJSON* pTypeArray , hcp_tParameterTemplateSet* pParameters) {
	if (pArray == HCP_NULL || pArray->type != cJSON_Array) {
		return HCP_NOERROR;
	}

	hcp_Int length = cJSON_GetArraySize(pArray);
	hcp_Int error = HCP_NOERROR;

	for (hcp_Int i = 0; i < length; i++) {
		cJSON* pair = cJSON_GetArrayItem(pArray, i);

		if (pair == HCP_NULL || pair->type != cJSON_Object) {
			continue;
		}

		hcp_Size_t index = 0;

		error = hcp_PushEmpty(&pParameters->header, &index);

		if (error != HCP_NOERROR) {
			return error;
		}

		hcp_tParameterTemplate* t = (hcp_tParameterTemplate*)hcp_ValueAt(&pParameters->header, index);
		hcp_tString typeName;
		hcp_tString valueLength;
		hcp_Uint8 typeId = HCP_TYPE_INVALID;

		typeName.length = 0;
		typeName.value = 0;

		hcp_GetString(pair, HCP_TIF_PARAMNAME, &t->name);
		hcp_GetString(pair, HCP_TIF_PARAMTYPE, &typeName);
		hcp_GetString(pair, HCP_TIF_PARAMLENGTH, &valueLength);

		if (t->name.length == 0) {
			error = HCP_MISSING_PARAMETERNAME;
		} else  if (typeName.length == 0) {
			error = HCP_MISSING_PARAMETERTYPE;
		}

		typeId = hcp_GetBasicType(&typeName, pTypeArray);

		if (typeId == HCP_TYPE_INVALID) {
			error = HCP_INVALID_PARAMETERTYPE;
		}

		if (error != HCP_NOERROR) {
			hcp_Pop(&pParameters->header, index);
			return error;
		}

		if (valueLength.length > 0) {
			t->length = hcp_Atio(&valueLength);
		}
		else{
			t->length = 0;
		}

		if (t->length == -1 && i + 1 < length) {
			return HCP_INVALIDPARAMETER_UNKNOWNSTRINGNOTATEND;
		}

		t->type = typeId;
	}

	return error;
}

hcp_Int hcp_LoadHeader(hcp_tState* pState, hcp_tJSON* pRoot, hcp_tModel* pTemplate) {
	cJSON* header = cJSON_GetObjectItem(pRoot, HCP_TIF_HEADER);
	hcp_Int error = error = hcp_LoadProtocol(pState, header, &pTemplate->protocol);

	if (error != HCP_NOERROR) {
		return error;
	}

	if (header == NULL || header->type != cJSON_Object) {
		return HCP_NOERROR;
	}

	hcp_tModelHeader* templateHeader = &pTemplate->header;

	hcp_GetString(header, HCP_TIF_HEADER_SCHEMA, &templateHeader->schema);
	hcp_GetString(header, HCP_TIF_HEADER_VERSION, &templateHeader->version);
	hcp_GetString(header, HCP_TIF_HEADER_CREATED, &templateHeader->created);
	hcp_GetString(header, HCP_TIF_HEADER_PROTOCOL, &templateHeader->protocol);

	return HCP_NOERROR;
}




/*
*==============================================================================
* END OF FILE
*==============================================================================
*/
