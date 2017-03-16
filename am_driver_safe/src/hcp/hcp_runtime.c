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
#include "hcp_runtime.h"
#include "hcp_types.h"
#include "hcp_vector.h"
#include "hcp_tif.h"
#include "hcp_string.h"
#include "hcp_codec.h"
#include "hcp_library.h"
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

/** Vector with codecs */
HCP_VECTOR(hcp_tCodec, hcp_tCodecSet, HCP_MAXSIZE_CODECS);
/**	Vector with product libraries */
HCP_VECTOR(hcp_tCodecLibrary, hcp_tLibrarySet, HCP_MAXSIZE_LIBRARIES);

typedef struct {
	hcp_tVector header;
	hcp_tCommandTemplate fixed[HCP_MAXSIZE_COMMANDSETS];
	hcp_Size_t nextId;
} hcp_tCommmandTemplateSet;

/**	Object which connects host OS mapping functions to connections.
*/
struct hcp_tState {
	hcp_tHost host;
	hcp_tCodecSet codecs;
	hcp_tModelSet templates;
	hcp_tLibrarySet libraries;
	hcp_Uint32 nextId;
	hcp_Boolean readLock;		/* HCP_TRUE if the state is locked in read only */
	hcp_Boolean writeLock;		/* HCP_TRUE if the state is currently being written to 
								 * which means that no other lock can (should) be aquired */
	hcp_tRuntime runtime;
};


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

	/** Initializes a runtime-structure.
	 *--------------------------------------------------------
	 * \par	Description:
	 *		Maps a runtime-object to correct functions.
	 *
	 * \param	pState	[IN]	State to use for memory management
	 * \param	pDest	[IN]	Object to map.
	 */
	static void hcp_InitializeRuntime(hcp_tState* pState, hcp_tRuntime* pDest);
	/** Compares two codecs objects for equality.
	 *--------------------------------------------------------
	 * \par	Description:
	 *		Checks the id property of two codecs objects and returns a\n
	 *		a value which indicates if the two match.
	 *
	 * \param	pLhs	[IN]	Vector header.
	 * \param	pRhs	[IN]	Value to store.
	 *
	 * \return	Operation result.
	 * \retval	0	=	Success
	 * \retval	-1 or 1	=	The two object did not match.
	 *--------------------------------------------------------
	 */
	static hcp_Int hcp_CompareCodec(void* pLhs, void* pRhs, void* pContext);
	/** Checks if a value handle refers to a valid hcp_tCodec
	*--------------------------------------------------------
	* \par	Description:
	*		Casts [pValue] into a hcp_tCodec and checks if the\n
	*		id property is larger than zero.
	*
	* \param	pValue	[IN]	Vector value..
	* \param	pContext	[IN]	Ignored.
	*
	* \return	Operation result.
	* \retval	HCP_TRUE	=	The element is empty (and can be overwritten)
	* \retval	HCP_FALSE	=	The element is in use.
	*--------------------------------------------------------
	*/
	static hcp_Boolean hcp_IsCodec(void* pValue, void* pContext);

	static hcp_Int hcp_InitializeLibraries(hcp_tState* pState, hcp_tLibrarySet* pLibraries);
	static hcp_Int hcp_InitializeTIFTemplates(hcp_tState* pState, hcp_tModelSet* pTemplates);
	static hcp_Int hcp_CompareTIFTemplate(void* pTemplate, void* Id, void* pState);
	static hcp_Boolean hcp_IsTIFTemplate(void* pTemplate, void* pState);
	static hcp_Int hcp_CompareLibrary(void* pLibrary, void* pName, void* pState);
	static hcp_Boolean hcp_IsLibrary(void* pLibrary, void* pState);
	static void hcp_RTMemcpy(hcp_tRuntime* pRuntime, void* pDestination, const void* pSource, const hcp_Size_t NumberOfBytes);
	static void hcp_RTMemset(hcp_tRuntime* pRuntime, void* pDest, const unsigned char Value, const hcp_Size_t DestSize);

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

void* hcp_Malloc(hcp_tState* pState, const hcp_Size_t Size) {
	if (pState == HCP_NULL || pState->host.malloc_ == HCP_NULL || Size == 0) {
		return HCP_NULL;
	}

	return pState->host.malloc_(Size, pState->host.context);
}

void hcp_Free(hcp_tState* pState, void* pDest) {
	if (pState == HCP_NULL || pDest == HCP_NULL || pState->host.free_ == HCP_NULL) {
		return;
	}

	pState->host.free_(pDest, pState->host.context);
}

void hcp_Memset(hcp_tState* pState, void* pDest, const unsigned char Value, const hcp_Size_t DestSize) {
	if (DestSize == 0 || pDest == HCP_NULL) {
		return;
	}

	if (pState == HCP_NULL || pState->host.memset_ == HCP_NULL) {
		unsigned char* dest = (unsigned char*)pDest;
		hcp_Size_t length = DestSize;

		while(length > 0) {
			*dest = Value;
			dest++;
			length--;
		}

	}
	else {
		void* context = pState->host.context;
		pState->host.memset_(pDest, Value, DestSize, context);
	}
}

void hcp_Memcpy(hcp_tState* pState, void* pDestination, const void* pSource, const hcp_Size_t NumBytes) {
	if (pDestination == HCP_NULL || pSource == HCP_NULL || NumBytes == 0) {
		return;
	}

	if (pState == HCP_NULL || pState->host.memcpy_ == HCP_NULL) {
		unsigned char* dest = (unsigned char*)pDestination;
		unsigned char* src = (unsigned char*)pSource;
		hcp_Size_t length = NumBytes;

		while (length > 0) {
			*dest = *src;
			dest++; src++;
			length--;
		}
	}
	else {
		pState->host.memcpy_(pDestination, pSource, NumBytes, pState->host.context);
	}
}

hcp_Boolean hcp_IsDynamic(hcp_tState* pState) {
	if (pState == HCP_NULL || pState->host.malloc_ == HCP_NULL) {
		return HCP_FALSE;
	}

	return HCP_TRUE;
}

void hcp_Unlock(hcp_tState* pState, const void* pHandle) {
	if (pState->host.unlock != HCP_NULL) {
		pState->host.unlock(pHandle, pState->host.context);
	}
}

hcp_Boolean hcp_TryLock(hcp_tState* pState, const void* pHandle, const hcp_Int Duration) {
	if (pState->host.lock == HCP_NULL) {
		return HCP_TRUE;
	}

	return pState->host.lock(pHandle, Duration, pState->host.context);
}

hcp_Int hcp_CloseState(hcp_tState* pState) {
	if (pState == HCP_NULL) {
		return HCP_INVALIDSTATE;
	}
	// locked region
	{
		// pop all codecs
		if (pState->codecs.header.length > 0) {
			hcp_Size_t index = pState->codecs.header.length - 1;

			do {
				hcp_Pop(&pState->codecs.header, index);
			} while (index-- != 0);
		}

		// pop all templates
		if (pState->templates.header.length > 0) {
			hcp_Size_t index = pState->templates.header.length - 1;

			do {
				hcp_Pop(&pState->templates.header, index);
			} while (index-- != 0);
		}

		// pop all libraries
		if (pState->libraries.header.length > 0) {
			hcp_Size_t index = pState->libraries.header.length - 1;

			do {
				hcp_Pop(&pState->libraries.header, index);
			} while (index-- != 0);
		}

		hcp_Memset(HCP_NULL, (void*)pState, 0, (const hcp_Size_t)sizeof(hcp_tState));
	}
	hcp_Unlock(pState, &pState->writeLock);

	return HCP_NOERROR;
}

hcp_Int hcp_CloseCodec(hcp_tState* pState, const hcp_Size_t Id) {
	if (pState == HCP_NULL) {
		return HCP_INVALIDSTATE;
	}

	// locked region
	{
		hcp_Boolean found = HCP_FALSE;
		hcp_Size_t index = hcp_FindFirst(&pState->codecs.header, 0, (void*)Id, &found);

		if (found == HCP_FALSE) {
			return HCP_INVALIDID;
		}

		hcp_Pop(&pState->codecs.header, index);
	}

	return HCP_NOERROR;
}



hcp_Int hcp_NewState(hcp_tState* pState, hcp_tHost* pHost) {
	if (pState == HCP_NULL) {
		return HCP_INVALIDSTATE;
	}

	pState->host.memset_ = HCP_NULL;
	pState->host.context = HCP_NULL;
	pState->host.free_ = HCP_NULL;
	pState->host.lock = HCP_NULL;
	pState->host.malloc_ = HCP_NULL;
	pState->host.memcpy_ = HCP_NULL;
	pState->host.realloc_ = HCP_NULL;
	pState->host.unlock = HCP_NULL;

	hcp_Memset(HCP_NULL, (void*)pState, 0, sizeof(hcp_tState));
	
	if (pHost != HCP_NULL) {
		if (pHost->malloc_ != HCP_NULL && pHost->free_ == HCP_NULL) {
			return HCP_NOFREE;
		}

		if (pHost->lock != HCP_NULL && pHost->unlock == HCP_NULL) {
			return HCP_NOUNLOCK;
		}

		(pState)->host = *pHost;
	}

	// init serialiser set
	HCP_INITIALIZEVECTOR(pState, &pState->codecs.header, pState->codecs.fixed,
		hcp_tCodec, HCP_NULL, hcp_CompareCodec, hcp_IsCodec);
	// init tif-templates
	hcp_InitializeTIFTemplates(pState, &(pState)->templates);
	// init product libraries
	hcp_InitializeLibraries(pState, &(pState)->libraries);
	// map runtime-functions
	hcp_InitializeRuntime(pState, &pState->runtime);
	// the id needs to be greater than zero since zero indicates failure
	// in hcp_NewCodec
	(pState)->nextId = 1;
	(pState)->templates.nextId = 1;


	return HCP_NOERROR;
}

hcp_Int hcp_Encode(hcp_tState* pState, hcp_Size_t CodecId, hcp_cszStr Command, hcp_Uint8* pDestination, hcp_Uint32 MaxLength) {
	if (pState == HCP_NULL) {
		return HCP_INVALIDSTATE;
	}

	hcp_Size_t index = -1;
	hcp_tCodec* codec = HCP_NULL;
	hcp_Int error = HCP_NOERROR;

	// locked region 
	{
		hcp_Boolean found = HCP_FALSE;
		index = hcp_FindFirst(&pState->codecs.header, 0, (void*)CodecId, &found);

		if (found == HCP_TRUE) {
			codec = (hcp_tCodec*)hcp_ValueAt(&pState->codecs.header, index);
		}
	}

	if (codec == HCP_NULL) {
		return HCP_INVALIDID;
	}

	// locked region
	{
		hcp_tCommand* output = HCP_NULL;
		hcp_tString input;

		input.zeroTerm = HCP_TRUE;
		input.value = Command;
		input.length = hcp_szStrLen(Command);

		error = hcp_ParseTifCommand(&input, &codec->commands, &output);

		if (error == HCP_NOERROR && output != HCP_NULL) {
			// call library
			if (codec->library->encode == HCP_NULL) {
				error = HCP_SERIALIZENOTSUPPORTED;
			}
			else {
				hcp_tBlob destination;

				destination.value = pDestination;
				destination.maxLength = MaxLength;
				destination.length = 0;

				error = codec->library->encode(&pState->runtime,&codec->template_->protocol,
					output, &destination, &codec->context);

				if (error == HCP_NOERROR) {
					// on success, the return value is the number of bytes written
					error = (hcp_Int)destination.length;
				}
			}
		}
	}

	// a return value greater than -1 means the number of bytes written to the stream
	return error;
}

hcp_Int hcp_Decode(hcp_tState* pState, hcp_Size_t CodecId, const hcp_Uint8* pSource, const hcp_Size_t Length, hcp_tResult* pResult) {
	hcp_Int error = HCP_NOERROR;
	hcp_Boolean found = HCP_FALSE;
	hcp_Size_t codecIndex = hcp_FindFirst(&pState->codecs.header, 0, (void*)CodecId, &found);

	if (found == HCP_FALSE) {
		return HCP_INVALIDID;
	}

	hcp_tCodec* codec = (hcp_tCodec*)hcp_ValueAt(&pState->codecs.header, codecIndex);
	hcp_Int bytesRead = 0;

	// locked area
	{
		hcp_tProtocol* protocol = &codec->template_->protocol;
		hcp_tCodecLibrary* library = codec->library;

		if (library->decode == HCP_NULL) {
			bytesRead = HCP_DESERIALIZENOTSUPPORTED;
		}
		else {
			hcp_tCommand* command = HCP_NULL;
			hcp_tBlob source;

			source.value = (hcp_Uint8*)pSource;
			source.length = Length;
			source.maxLength = Length;

			hcp_tCommandSet* commandSet = &codec->commands;
			hcp_tBuffer* context = &codec->context;

			bytesRead = library->decode(&pState->runtime, protocol, &source, commandSet, &command, context);

			if (pResult != HCP_NULL) {
				if (bytesRead < 0) {
					pResult->error = bytesRead;

					if (library->lastError != HCP_NULL) {
						pResult->deviceError = library->lastError(&pState->runtime, &codec->context, &pResult->message);
					} else {
						pResult->deviceError = bytesRead;
					}
					
					pResult->parameterCount = 0;
					pResult->parameters = HCP_NULL;
				}
				else {
					pResult->error = HCP_NOERROR;
					// libraries should really return a command, but if they dont we still
					// dont want everything to crash and burn...
					if (command != HCP_NULL) {
						pResult->deviceError = HCP_NOERROR;
						pResult->parameterCount = command->outParams.header.length;
						pResult->parameters = (hcp_tParameter*)command->outParams.header.values;
						pResult->command = command->template_->header.command;
						pResult->family = command->template_->header.family;
					}
					else {
						hcp_Memset(pState, pResult, 0, sizeof(hcp_tResult));
					}
				}
			}	
		}
	} 

	return bytesRead;
}

hcp_Int hcp_NewCodec(hcp_tState* pState,hcp_cszStr Codec,const hcp_Size_t TemplateId, hcp_Size_t* pId) {
	*pId = 0;
	hcp_Int error = HCP_NOERROR;

	// locked region
	{
		hcp_Boolean found = HCP_FALSE;
		// locate the specified tif-t using which the codec will
		// get it's commands populated
		hcp_Size_t templateIndex = hcp_FindFirst(&pState->templates.header, 0, (void*)TemplateId, &found);

		if (found == HCP_FALSE) {
			error = HCP_INVALIDTEMPLATEID;
		}
		else {
			hcp_tModel* t = (hcp_tModel*)hcp_ValueAt(&pState->templates.header, templateIndex);

			// locate the specified codec library to use with the codec
			hcp_Boolean found = HCP_FALSE;

			hcp_Size_t libraryIndex = hcp_FindFirst(&pState->libraries.header, 0, (void*)Codec, &found);

			if (found == HCP_FALSE) {
				error = HCP_INVALIDLIB;
			} else {
				hcp_tCodecLibrary* library = (hcp_tCodecLibrary*)hcp_ValueAt(&pState->libraries.header, libraryIndex);

				hcp_Size_t codecIndex = 0;
				error = hcp_PushEmpty(&pState->codecs.header, &codecIndex);

				if (error == HCP_NOERROR) {
					hcp_tCodec* obj = (hcp_tCodec*)hcp_ValueAt(&pState->codecs.header, codecIndex);

					obj->id = pState->nextId++;
					obj->parent = pState;
					obj->template_ = t;
					obj->library = library;
					obj->context.length = sizeof(obj->context.value);

					//hcp_InitializeBuffer(&obj->context);

					*pId = (hcp_Size_t)obj->id;
					// populate the codec's commands using the TIF-file
					error = hcp_InitializeCommands(pState, &obj->commands, t);
					
					if (error == HCP_NOERROR) {
						// let the codec setup it's internal state
						if (obj->library->setup != HCP_NULL) {
							error = obj->library->setup(&pState->runtime, &obj->context);
						}
					}

					if (error != HCP_NOERROR) {
						// if parsing fails, remove the new instance from the list
						hcp_Pop(&pState->codecs.header, codecIndex);
					}
				}
			}
		}
	}

	return error;
}

hcp_Size_t hcp_SizeOfState(void) {
	return sizeof(hcp_tState);
}

void hcp_GetMessage(const hcp_Int ErrorCode, hcp_szStr pOutput, const hcp_Size_t MaxLength) {
	const hcp_tErrorMessage* msg = hcp_Errors;
	hcp_cszStr src = HCP_NULL;

	while (msg != HCP_NULL && !((*msg).code == 0 && (*msg).message == HCP_NULL)) {
		if ((*msg).code == ErrorCode) {
			src = (*msg).message;
			break;
		}

		msg++;
	}

	if (src == HCP_NULL) {
		src = "Invalid error code.";
	}

	hcp_Size_t len = 0;
	hcp_szStr output = (hcp_szStr)pOutput;
	// copy the string char by char since we have no state
	while (*src != 0 && len < MaxLength) {
		*output = *src;
		output++;
		src++;
		len++;
	}

	if (len < MaxLength) {
		pOutput[len] = 0;
	}
}

hcp_Int hcp_GetPrimitiveType(hcp_tState* pState, const hcp_Int CommandSetId, const hcp_szStr ComplexType) {
	hcp_Int error = HCP_NOERROR;

	{
		hcp_Boolean found = HCP_FALSE;

		hcp_Size_t index = hcp_FindFirst(&pState->templates.header, 0, (void*)(HCP_SIZEMASK & CommandSetId), &found);

		if (found == HCP_TRUE) {
			hcp_tModel* t = (hcp_tModel*)hcp_ValueAt(&pState->templates.header, index);
		}
		else {
			error = HCP_INVALIDTEMPLATEID;
		}
	}

	return error;
}

hcp_Int hcp_LoadModel(hcp_tState* pState, hcp_cszStr Model, const hcp_Size_t Length, hcp_Int* pId) {
	*pId = -1;

	hcp_Int error = HCP_NOERROR;
	// locked session
	{
		// cast into a tString in which we track the length
		hcp_tString text;

		text.value = (hcp_Char*)Model;
		text.length = Length;
		text.zeroTerm = HCP_TRUE;

		hcp_Size_t index = 0;

		error = hcp_PushEmpty(&pState->templates.header, &index);

		if (error != HCP_NOERROR) {
			return error;
		}

		hcp_tModel* t = (hcp_tModel*)hcp_ValueAt(&pState->templates.header, index);

		error = hcp_LoadTIFTemplate(pState, &text, t);

		if (error != HCP_NOERROR) {
			hcp_Pop(&pState->templates.header, index);
		}
		else {
			t->id = pState->templates.nextId++;
			*pId = t->id;
		}
	}

	return error;
}

hcp_cszStr hcp_GetTypeName(const hcp_Uint8 Id) {
	const hcp_tType* type = hcp_Types;

	while (type != HCP_NULL && !(type->id == 0 && type->name == HCP_NULL)) {
		if (Id == type->id) {
			return type->name;
		}

		type++;
	}

	return HCP_INVALID_NAME;
}

const hcp_Uint8 hcp_GetTypeId(const hcp_tString* pName) {
	const hcp_tType* type = hcp_Types;

	while (type != HCP_NULL && !(type->id == 0 && type->name == HCP_NULL)) {
		
		if (hcp_tStrSzCmp(pName, type->name) == 0) {
			return type->id;
		}

		type++;
	}

	return HCP_TYPE_INVALID;
}

hcp_Size_t hcp_GetTypeSize(const hcp_Int TypeId) {
	switch (TypeId) {
		case HCP_INVALID: return 0;
		case HCP_FLOAT_ID: return sizeof(hcp_Float);
		case HCP_BOOLEAN_ID: return sizeof(hcp_Boolean);
		case HCP_VOID_ID: return sizeof(hcp_Void*);
		case HCP_SIZET_ID: return sizeof(hcp_Size_t);
		case HCP_UINT8_ID: return sizeof(hcp_Uint8);
		case HCP_INT8_ID: return sizeof(hcp_Int8);
		case HCP_UINT16_ID: return sizeof(hcp_Uint16);
		case HCP_INT16_ID: return sizeof(hcp_Int16);
		case HCP_UINT32_ID: return sizeof(hcp_Uint32);
		case HCP_INT32_ID: return sizeof(hcp_Int32);
		case HCP_UINT64_ID: return sizeof(hcp_Uint64);
		case HCP_INT64_ID: return sizeof(hcp_Int64);
		case HCP_STRING_ID: return sizeof(hcp_tString);
		case HCP_DOUBLE_ID: return sizeof(hcp_Double);
		case HCP_UNIXTIME_ID: return sizeof(hcp_UnixTime);
		case HCP_SIMPLEVERSION_ID: return sizeof(hcp_SimpleVersion);
	}

	return 0;
}

hcp_Int hcp_LoadCodec(hcp_tState* pState, hcp_tCodecLibrary* pLibrary, hcp_szStr CodecName, const hcp_Size_t MaxLength) {
	if (pState == HCP_NULL) {
		return HCP_INVALIDSTATE;
	}

	if (pLibrary == HCP_NULL) {
		return HCP_CODECLOADFAILED;
	}

	if (pLibrary->name == HCP_NULL) {
		return HCP_INVALIDCODECNAME;
	}
	// try to found one with the same name
	hcp_Boolean success = HCP_FALSE;
	hcp_Size_t codecIndex = hcp_FindFirst(&pState->libraries.header, 0, pLibrary->name, &success);

	if (success == HCP_TRUE) {
		// the codec has already been loaded
		return HCP_INVALIDCODECNAME;
	}
	// push a copy onto the vector
	hcp_Int error = hcp_PushEmpty(&pState->libraries.header, &codecIndex);

	if (error != HCP_NOERROR) {
		return error;
	}

	hcp_tCodecLibrary* dest = (hcp_tCodecLibrary*)hcp_ValueAt(&pState->libraries.header, codecIndex);
	hcp_Memcpy(pState, dest, pLibrary, sizeof(hcp_tCodecLibrary));

	// output the name of the loaded library
	if (CodecName != HCP_NULL && MaxLength > 1) {
		hcp_Size_t size = hcp_szStrLen(dest->name);

		size = size + 1 > MaxLength ? MaxLength - 1 : size;

		hcp_Memcpy(pState, CodecName, dest->name, size);
		CodecName[size] = 0;
	}

	return error;
}
/*
*==============================================================================
*  5.   LOCAL FUNCTIONS (declared in Section 3.5)
*==============================================================================
*/

hcp_Int hcp_CompareCodec(void* pLhs, void* pRhs, void* pContext) {
	hcp_tCodec* codec = (hcp_tCodec*)pLhs;
	return (hcp_Int)((HCP_SIZEMASK & codec->id) == (hcp_Size_t)pRhs) ? 0 : 1;
}

hcp_Boolean hcp_IsCodec(void* pValue, void* pContext) {
	hcp_tCodec* codec = (hcp_tCodec*)pValue;
	return codec->id == 0 ? HCP_FALSE : HCP_TRUE;
}

hcp_Int hcp_InitializeTIFTemplates(hcp_tState* pState, hcp_tModelSet* pTemplates) {
	hcp_Int error = HCP_INITIALIZEVECTOR(pState, &pTemplates->header, pTemplates->fixed,
		hcp_tModel, HCP_NULL, hcp_CompareTIFTemplate, hcp_IsTIFTemplate);

	return error;
}

hcp_Int hcp_InitializeLibraries(hcp_tState* pState, hcp_tLibrarySet* pLibraries) {
	hcp_Int error = HCP_INITIALIZEVECTOR(pState, &pLibraries->header, pLibraries->fixed,
		hcp_tCodecLibrary, HCP_NULL, hcp_CompareLibrary, hcp_IsLibrary);

	return error;
}

hcp_Int hcp_CompareTIFTemplate(void* pTemplate, void* Id, void* pState) {
	hcp_tModel* t = (hcp_tModel*)pTemplate;
	return (hcp_Int)((HCP_SIZEMASK & t->id) == (hcp_Size_t)Id) ? 0 : -1;
}

hcp_Boolean hcp_IsTIFTemplate(void* pTemplate, void* pState) {
	hcp_tModel* t = (hcp_tModel*)pTemplate;
	return t->id > 0 ? HCP_TRUE : HCP_FALSE;
}

hcp_Int hcp_CompareLibrary(void* pLibrary, void* pName, void* pState) {
	hcp_tCodecLibrary* library = (hcp_tCodecLibrary*)pLibrary;
	hcp_szStr name = (hcp_szStr)pName;

	return hcp_szStrCmp(library->name, name);
}

hcp_Boolean hcp_IsLibrary(void* pLibrary, void* pState) {
	hcp_tCodecLibrary* library = (hcp_tCodecLibrary*)pLibrary;
	return library->name == HCP_NULL ? HCP_FALSE : HCP_TRUE;
}

void hcp_RTMemset(hcp_tRuntime* pRuntime, void* pDest, const unsigned char Value, const hcp_Size_t DestSize) {
	if (pRuntime == HCP_NULL) {
		return;
	}

	hcp_Memset((hcp_tState*)pRuntime->context, pDest, Value, DestSize);
}
void hcp_RTMemcpy(hcp_tRuntime* pRuntime, void* pDestination, const void* pSource, const hcp_Size_t NumberOfBytes) {
	if (pRuntime == HCP_NULL) {
		return;
	}

	hcp_Memcpy((hcp_tState*)pRuntime->context, pDestination, pSource, NumberOfBytes);
}

void hcp_InitializeRuntime(hcp_tState* pState, hcp_tRuntime* R) {
	hcp_Memset(pState, R, 0, sizeof(hcp_tRuntime));

	R->context = (void*)pState;
	// vector
	R->FindFirst = hcp_FindFirst;
	R->ValueAt = hcp_ValueAt;
	// register
	R->Load = HCP_NULL;
	R->Memset = hcp_RTMemset;
	R->Memcpy = hcp_RTMemcpy;
	// codec functions
	R->GetUint8 = hcp_GetUint8;
	R->GetUint16 = hcp_GetUint16;
	// todo
	R->InitializeBuffer = hcp_InitializeBuffer;
	R->Resize = hcp_Resize;
	// common functions
	R->GetTypeSize = hcp_GetTypeSize;
	R->GetTypeName = hcp_GetTypeName;
	R->GetTypeId = hcp_GetTypeId;
	// string functions
	R->szStrLen = hcp_szStrLen;
	R->szStrCmp = hcp_szStrCmp;

	R->tStrCmp = hcp_tStrCmp;
	R->tStrSzCmp = hcp_tStrSzCmp;
	R->IsDecimal = hcp_IsDecimal;
	R->IsAlphaNumerical = hcp_IsAlphaNumerical;
	R->IsDigit = hcp_IsDigit;
	R->IsHexadecimal = hcp_IsHexadecimal;
	R->Atio = hcp_Atio;
	R->CharacterToInt = hcp_CharacterToInt;

	R->Uint8ToBytes = hcp_Uint8ToBytes;
	R->Int8ToBytes = hcp_Int8ToBytes;
	R->Uint16ToBytes = hcp_Uint16ToBytes;
	R->Int16ToBytes = hcp_Int16ToBytes;
	R->Uint32ToBytes = hcp_Uint32ToBytes;
	R->Int32ToBytes = hcp_Int32ToBytes;
	R->Uint64ToBytes = hcp_Uint64ToBytes;
	R->Int64ToBytes = hcp_Int64ToBytes;

	R->BytesToUint8 = hcp_BytesToUint8;
	R->BytesToInt8 = hcp_BytesToInt8;
	R->BytesToUint16 = hcp_BytesToUint16;
	R->BytesToInt16 = hcp_BytesToInt16;
	R->BytesToUint32 = hcp_BytesToUint32;
	R->BytesToInt32 = hcp_BytesToInt32;
	R->BytesToUint64 = hcp_BytesToUint64;
	R->BytesToInt64 = hcp_BytesToInt64;

	R->AppendBytes = hcp_AppendBytes;
	R->AppendByte = hcp_AppendByte;
	R->AppendBlob = hcp_AppendBlob;
	R->AppendZeroes = hcp_AppendZeroes;
	R->AppendString = hcp_AppendString;

	R->AppendParameters = hcp_AppendParameters;
	R->Leftshift = hcp_Leftshift;
	R->BytesToParameters = hcp_BytesToParameters;
	R->GetParameterSetSize = hcp_GetParameterSetSize;
}

/*
*==============================================================================
* END OF FILE
*==============================================================================
*/
