/*
*------------------------------------------------------------------------------
* (c) Husqvarna AB
*------------------------------------------------------------------------------
*/
/**
* @file
* <Brief description of this file>.
*------------------------------------------------------------------------------
* \par DESCRIPTION:
*      <Detailed description of this file.>\n
*      <Backslash n forces a new line.>
*
* \par IDENTIFICATION:
*           $Module: runtime $
*           $Target: any $
*      $Environment: Visual Studio 2015 $
*          $Project: HCP
*         $Revision: 1$
*             $Date: 2016-03-4 14:20:45$
*           $Author: Olof Andreassen$
*
*/


/*
*==============================================================================
*  1.3     Re-definition guard
*==============================================================================
*/
#ifndef _HCP_CODEC_H_
#define _HCP_CODEC_H_
/*
*==============================================================================
*  2.      INCLUDE FILES
*==============================================================================
*/

#include "hcp_types.h"
#include "hcp_vector.h"
/*
*==============================================================================
*  3.      DECLARATIONS
*  3.1     Global constants
*==============================================================================
*/

/**	Allows CODEC libraries to specifiy custom error messages. When returning
 *	a error message libraries should "&" with HCP_LIBERR to indicte that their
 *	custom error-message array should be used instead of the HCP-builtin.
 */
#define HCP_LIBERR -0x200
#define HCP_LIBNAMEMAXSIZE 32

/*
*==============================================================================
*  3.2     Global macros
*==============================================================================
*/

/*
*==============================================================================
*  3.3     Global type definitions
*==============================================================================
*/

/** Represents a key/value pair in a codec collection
 */
typedef struct {
	hcp_tString key;
	hcp_tString value;
} hcp_tProtocolNode;

HCP_VECTOR(hcp_tProtocolNode, hcp_tProtocol, HCP_MAXSIZE_PROTOCOLS);

/**	Description of a parameter
 */
#pragma pack(push, 8)
typedef struct {
	hcp_tString name;
	hcp_Uint8 type;
	hcp_Int32 length;	/* custom length of a value */
} hcp_tParameterTemplate;
#pragma pack(pop)

/** Named parameter.
*/
#pragma pack(push, 8)
typedef struct {
	hcp_tParameterTemplate* template_;
	hcp_tValue value;	/* value of the parameter */
	hcp_Boolean hasValue;	/* indicates if the parameter actually has a value  */
} hcp_tParameter;
#pragma pack(pop)

/** Collection (set) of parameters
*/
HCP_VECTOR(hcp_tParameter, hcp_tParameterSet, HCP_MAXSIZE_PARAMS);
HCP_VECTOR(hcp_tParameterTemplate, hcp_tParameterTemplateSet, HCP_MAXSIZE_PARAMS);

/**	Header which identifies a command within a collection
 */
typedef struct {
	hcp_tString command;
	hcp_tString family;
} hcp_tCommandHeader;
/**	Represens a header which uniquely identifies a command element
 */
typedef struct {
	hcp_tCommandHeader header;
	hcp_tParameterTemplateSet inParameters;
	hcp_tParameterTemplateSet outParameters;
	hcp_tProtocol protocol;
} hcp_tCommandTemplate;

/**
*	A executable command which will be passed to a codec library for exeuction.
*/
typedef struct hcp_tCommand {
	hcp_tCommandTemplate* template_;		/* template on which the command is built and populated */
	hcp_tParameterSet inParams;		/* input parameters that will be populated before passed to the command */
	hcp_tParameterSet outParams;	/* output parameters that the codec library is expected to populate on run*/
} hcp_tCommand;


/**	Vector with commands that will be populated during runtime
*/
HCP_VECTOR(hcp_tCommand, hcp_tCommandSet, HCP_MAXSIZE_COMMANDS);

typedef struct hcp_tCodecLibrary hcp_tCodecLibrary;
typedef struct hcp_tRuntime hcp_tRuntime;

typedef int(*hcp_EncodeString)(hcp_tRuntime* R, const hcp_tParameterTemplate* pTemplate, const hcp_tString* pString, hcp_tBlob* pDestination, void* pContext);
typedef int(*hcp_DecodeString)(hcp_tRuntime* R, const hcp_tBlob* pSource, const hcp_tParameterTemplate* pTemplate, const hcp_Size_t Offset, hcp_tString* pString, void* pContext);


/**
*	HCP Runtime Interface
*/
struct hcp_tRuntime {
	void* context;

	// vector
	hcp_Size_t(HCP_CALL *FindFirst)(const hcp_tVector* pHeader, const hcp_Size_t StartIndex, void* pValue, hcp_Boolean* pSuccess);
	void* (HCP_CALL *ValueAt)(const hcp_tVector* pVector, const hcp_Size_t Index);
	// register
	hcp_Int(HCP_CALL *Load)(const hcp_tCodecLibrary* pLibrary);
	void (HCP_CALL *Memset)(hcp_tRuntime* pRuntime, void* pDest, const unsigned char Value, const hcp_Size_t DestSize);
	void (HCP_CALL *Memcpy)(hcp_tRuntime* pRuntime, void* pDestination, const void* pSource, const hcp_Size_t NumberOfBytes);
	void (HCP_CALL *Leftshift)(hcp_tRuntime* R, hcp_tBlob* pSource, const hcp_Size_t Steps);

	// codec functions
	hcp_Int(HCP_CALL *GetUint8)(const hcp_tProtocol* pProtocol, const hcp_szStr Key, hcp_Uint8* pDest);
	hcp_Int(HCP_CALL *GetUint16)(const hcp_tProtocol* pProtocol, const hcp_szStr Key, hcp_Uint16* pDest);
	// todo: add more functions
	void (HCP_CALL *InitializeBuffer)(hcp_tBuffer* pBuffer);
	hcp_Int(HCP_CALL *Resize)(hcp_tBuffer* pBuffer, const hcp_Size_t Length);

	// Common functions
	hcp_Size_t(HCP_CALL *GetParameterSetSize)(const hcp_tParameterSet* pParameters);
	hcp_Size_t(HCP_CALL *GetTypeSize)(const hcp_Int TypeId);
	hcp_cszStr(HCP_CALL* GetTypeName)(const hcp_Uint8 Id);
	const hcp_Uint8(HCP_CALL* GetTypeId)(const hcp_tString* pName);

	// String functions
	hcp_Int(HCP_CALL *szStrLen)(hcp_cszStr String);
	hcp_Int(HCP_CALL *szStrCmp)(hcp_cszStr Lhs, hcp_cszStr Rhs);

	hcp_Int(HCP_CALL *tStrCmp)(const hcp_tString* pLhs, const hcp_tString* pRhs);
	hcp_Int(HCP_CALL *tStrSzCmp)(const hcp_tString* pLhs, hcp_cszStr pRhs);
	hcp_Boolean(HCP_CALL *IsDecimal)(const hcp_tString* Number);
	hcp_Boolean(HCP_CALL *IsAlphaNumerical)(const hcp_Char Character);
	hcp_Boolean(HCP_CALL *IsDigit)(const hcp_Char Character);
	hcp_Boolean(HCP_CALL *IsHexadecimal)(const hcp_tString* Number);
	hcp_Int32(HCP_CALL *Atio)(const hcp_tString* pNumber);
	hcp_Int(HCP_CALL *CharacterToInt)(const hcp_Char Character);

	hcp_Int(HCP_CALL *Uint8ToBytes)(const hcp_Uint8 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess);
	hcp_Int(HCP_CALL *Int8ToBytes)(const hcp_Int8 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess);
	hcp_Int(HCP_CALL *Uint16ToBytes)(const hcp_Uint16 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess);
	hcp_Int(HCP_CALL *Int16ToBytes)(const hcp_Int16 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess);
	hcp_Int(HCP_CALL *Uint32ToBytes)(const hcp_Uint32 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess);
	hcp_Int(HCP_CALL *Int32ToBytes)(const hcp_Int32 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess);
	hcp_Int(HCP_CALL *Uint64ToBytes)(const hcp_Uint64 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess);
	hcp_Int(HCP_CALL *Int64ToBytes)(const hcp_Int64 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess);

	hcp_Int(HCP_CALL *BytesToUint8)(const hcp_tBlob* pSource, const hcp_Size_t Position, hcp_Uint8* Number, const hcp_Uint8 Endianess);
	hcp_Int(HCP_CALL *BytesToInt8)(const hcp_tBlob* pSource, const hcp_Size_t Position, hcp_Int8* Number, const hcp_Uint8 Endianess);
	hcp_Int(HCP_CALL *BytesToUint16)(const hcp_tBlob* pSource, const hcp_Size_t Position, hcp_Uint16* Number, const hcp_Uint8 Endianess);
	hcp_Int(HCP_CALL *BytesToInt16)(const hcp_tBlob* pSource, const hcp_Size_t Position, hcp_Int16* Number, const hcp_Uint8 Endianess);
	hcp_Int(HCP_CALL *BytesToUint32)(const hcp_tBlob* pSource, const hcp_Size_t Position, hcp_Uint32* Number, const hcp_Uint8 Endianess);
	hcp_Int(HCP_CALL *BytesToInt32)(const hcp_tBlob* pSource, const hcp_Size_t Position, hcp_Int32* Number, const hcp_Uint8 Endianess);
	hcp_Int(HCP_CALL *BytesToUint64)(const hcp_tBlob* pSource, const hcp_Size_t Position, hcp_Uint64* Number, const hcp_Uint8 Endianess);
	hcp_Int(HCP_CALL *BytesToInt64)(const hcp_tBlob* pSource, const hcp_Size_t Position, hcp_Int64* Number, const hcp_Uint8 Endianess);

	hcp_Int(HCP_CALL *BytesToParameters)(hcp_tRuntime* R, const hcp_tBlob* pSource, const hcp_Size_t Offset, hcp_tParameterSet* pParameters, const hcp_Uint8 Endianess, hcp_DecodeString StringDecoder, void* pContext);

	hcp_Int(HCP_CALL *AppendBytes)(const hcp_Uint8* pSource, const hcp_Size_t Length, hcp_tBlob* pDestination);
	hcp_Int(HCP_CALL *AppendByte)(const hcp_Uint8 Value, hcp_tBlob* pDestination);
	hcp_Int(HCP_CALL *AppendBlob)(const hcp_tBlob* pSource, hcp_tBlob* pDestination);
	hcp_Int(HCP_CALL *AppendZeroes)(const hcp_Size_t Length, hcp_tBlob* pDestination);
	hcp_Int(HCP_CALL *AppendString)(const hcp_tString* pString, hcp_tBlob* pDestination, hcp_Uint8 Encoding);
	hcp_Int(HCP_CALL *AppendParameters)(hcp_tRuntime* R, hcp_tBlob* pDestination, hcp_tParameterSet* pParameters, const hcp_Uint8 Endianess, hcp_EncodeString StringEncoder, void* pContext);

};

typedef hcp_tCodecLibrary*(HCP_CALL *hcp_CodecLoad)(void);
typedef hcp_Int(HCP_CALL *hcp_CodecSetup)(hcp_tRuntime* pRuntime, hcp_tBuffer* pContext);
typedef hcp_Int(HCP_CALL *hcp_CodecEncode)(hcp_tRuntime* pRuntime, hcp_tProtocol* pProtocol, const hcp_tCommand* pCommand, hcp_tBlob* pDestination, hcp_tBuffer* pContext);
typedef hcp_Int(HCP_CALL *hcp_CodecDecode)(hcp_tRuntime* pRuntime, hcp_tProtocol* pProtocol, const hcp_tBlob* pSource, hcp_tCommandSet* pCommands, hcp_tCommand** ppCommand, hcp_tBuffer* pContext);
typedef hcp_Int(HCP_CALL *hcp_CodecLastError)(hcp_tRuntime* pRuntime, hcp_tBuffer* pContext, hcp_cszStr* pMessage);

/**
*	Structure which exposes functionallity for encoding/decoding
*	data into a specific format.
*/
struct hcp_tCodecLibrary {
	hcp_Int requiredVersion;	/* Required version of HCP to run. */
	hcp_szStr name;	/* name of the codec */
	hcp_CodecSetup setup;	/* gives the codec a chance to initialize a state */
	hcp_CodecEncode encode;
	hcp_CodecDecode decode;
	hcp_CodecLastError lastError;
	const hcp_tErrorMessage* messages;	/* handle to array with custom error messages. the array must
										* end with a {HCP_NULL, HCP_NULL} pair.*/
	hcp_tBuffer context;	/* codec specific context buffer if a codec requires more \n
							* one should increae the size of tBuffer's value */
};

/*
*==============================================================================
*  3.4     Global variables (defined in some implementation file)
*==============================================================================
*/


/*
*==============================================================================
*  3.5     Global constant data
*==============================================================================
*/

/*
*==============================================================================
*  4.      GLOBAL FUNCTIONS (defined in some implementation file)
*==============================================================================
*/

#endif
/*
*==============================================================================
* END OF FILE
*==============================================================================
*/
