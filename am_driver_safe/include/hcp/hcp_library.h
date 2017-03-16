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
#ifndef _HCP_LIBRARY_H_
#define _HCP_LIBRARY_H_
/*
*==============================================================================
*  2.      INCLUDE FILES
*==============================================================================
*/

#include "hcp_codec.h"
#include "hcp_model.h"
/*
*==============================================================================
*  3.      DECLARATIONS
*  3.1     Global constants
*==============================================================================
*/

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



/**	Allocates a block of memory.
*-----------------------------------------------------------------------------
* \par	Description:
*		Allocates a block of size bytes of memory, returning a pointer to the beginning of the block.\n
*		The content of the newly allocated block of memory is not initialized, remaining with indeterminate values.\n
*		If no malloc function was provided this function will always return HCP_NULL,
*
* \param	pState	[IN]	State contaning OS malloc mapping.
* \param	Size [IN]	Number of bytes to allocate.
*
*
* \return	On success, a pointer to the memory block allocated by the function. The type of this pointer is\n
*			always void*, which can be cast to the desired type of data pointer in order to be dereferenceable.\n
*			If the function failed to allocate the requested block of memory, a null pointer is returned.
*/
extern void* HCP_CALL hcp_Malloc(hcp_tState* pState, const hcp_Size_t Size);
/**	Frees a block of dynamc memory.
*-----------------------------------------------------------------------------
* \par	Description:
*		If specified, calls the host's (OS) free function to release a block of memory.\n
*		If a null pointer is passed or no free method is specified, then the function takes no action..
*
* \param	pState	[IN]	State contaning OS free mapping.
* \param	pDst [IN]	Memory to release.
*
*
* \return	On success, a pointer to the memory block allocated by the function. The type of this pointer is\n
*			always void*, which can be cast to the desired type of data pointer in order to be dereferenceable.\n
*			If the function failed to allocate the requested block of memory, a null pointer is returned.
*/
extern void HCP_CALL hcp_Free(hcp_tState* pState, void* pDest);
/**	Clears a memory area, settig it to a specifed value.
*-----------------------------------------------------------------------------
* \par	Description:
*		Sets a memory area to a specified value. If no host (OS) function for memset \n
*		was set, a custom one will be used instead, this might effect performance for large\n
*		arrays.
*
* \param	pState	[IN]	Current state.
* \param	pDest	[OUT]	Memory area that will be cleared.
* \param	Value	[IN]	Value to set the memory elements to.
* \param	DestSize	[IN]	Number of bytes in [pDest].
*/
extern void HCP_CALL hcp_Memset(hcp_tState* pState, void* pDest, const unsigned char Value, const hcp_Size_t DestSize);

/**	Copy block of memory
*-----------------------------------------------------------------------------
* \par	Description:
*		Copies the values of num bytes from the location pointed to by source\n
*		directly to the memory block pointed to by destination.If the host\n
*		provided to memcpy function a built in one will be used which might\n
*		effect performance (and not in a good way).
*
* \param	pState	[IN]	Current state.
* \param	pDest	[OUT]	Memory area that will be cleared.
* \param	Value	[IN]	Value to set the memory elements to.
* \param	DestSize	[IN]	Number of bytes in [pDest].
*/
extern void HCP_CALL hcp_Memcpy(hcp_tState* pState, void* pDestination, const void* pSource, const hcp_Size_t NumBytes);

/**	Checks if the state has dynamic memory enabled.
*-----------------------------------------------------------------------------
* \par	Description:
*		Returns a value which indicates if a state has OS/platform wrappers which\n
*		provides functions for dynamic memory allocation.
*
* \param	pState	[IN]	State to check.
*
* \return	Result
* \retval	HCP_TRUE	=	The state has malloc and free mappings.
* \retval	HCP_FALSE	=	The state and all resources should use static memory.
*/
extern hcp_Boolean hcp_IsDynamic(hcp_tState* pState);


extern hcp_Int HCP_CALL hcp_SetString(hcp_tParameterSet* pParameters, const hcp_tString* Name, const hcp_tString* Value);
extern hcp_Int HCP_CALL hcp_GetUint8(const hcp_tProtocol* pProtocol, const hcp_szStr Key, hcp_Uint8* pDest);
extern hcp_Int HCP_CALL hcp_GetUint16(const hcp_tProtocol* pProtocol, const hcp_szStr Key, hcp_Uint16* pDest);
// todo: add more functions
extern void HCP_CALL hcp_InitializeBuffer(hcp_tBuffer* pBuffer);
extern hcp_Int HCP_CALL hcp_Resize(hcp_tBuffer* pBuffer, const hcp_Size_t Length);


extern hcp_Int HCP_CALL hcp_Uint8ToBytes(const hcp_Uint8 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess);
extern hcp_Int HCP_CALL hcp_Int8ToBytes(const hcp_Int8 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess);
extern hcp_Int HCP_CALL hcp_Uint16ToBytes(const hcp_Uint16 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess);
extern hcp_Int HCP_CALL hcp_Int16ToBytes(const hcp_Int16 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess);
extern hcp_Int HCP_CALL hcp_Uint32ToBytes(const hcp_Uint32 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess);
extern hcp_Int HCP_CALL hcp_Int32ToBytes(const hcp_Int32 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess);
extern hcp_Int HCP_CALL hcp_Uint64ToBytes(const hcp_Uint64 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess);
extern hcp_Int HCP_CALL hcp_Int64ToBytes(const hcp_Int64 Number, hcp_tBlob* pDestination, const hcp_Uint8 Endianess);

extern hcp_Int HCP_CALL hcp_BytesToUint8(const hcp_tBlob* pSource, const hcp_Size_t Offset, hcp_Uint8* Number, const hcp_Uint8 Endianess);
extern hcp_Int HCP_CALL hcp_BytesToInt8(const hcp_tBlob* pSource, const hcp_Size_t Offset, hcp_Int8* Number, const hcp_Uint8 Endianess);
extern hcp_Int HCP_CALL hcp_BytesToUint16(const hcp_tBlob* pSource, const hcp_Size_t Offset, hcp_Uint16* Number, const hcp_Uint8 Endianess);
extern hcp_Int HCP_CALL hcp_BytesToInt16(const hcp_tBlob* pSource, const hcp_Size_t Offset, hcp_Int16* Number, const hcp_Uint8 Endianess);
extern hcp_Int HCP_CALL hcp_BytesToUint32(const hcp_tBlob* pSource, const hcp_Size_t Offset, hcp_Uint32* Number, const hcp_Uint8 Endianess);
extern hcp_Int HCP_CALL hcp_BytesToInt32(const hcp_tBlob* pSource, const hcp_Size_t Offset, hcp_Int32* Number, const hcp_Uint8 Endianess);
extern hcp_Int HCP_CALL hcp_BytesToUint64(const hcp_tBlob* pSource, const hcp_Size_t Offset, hcp_Uint64* Number, const hcp_Uint8 Endianess);
extern hcp_Int HCP_CALL hcp_BytesToInt64(const hcp_tBlob* pSource, const hcp_Size_t Offset, hcp_Int64* Number, const hcp_Uint8 Endianess);

extern hcp_Int HCP_CALL hcp_BytesToParameters(hcp_tRuntime* R, const hcp_tBlob* pSource, const hcp_Size_t Offset, hcp_tParameterSet* pParameters, const hcp_Uint8 Endianess, hcp_DecodeString StringDecoder, void* pContext);

extern hcp_Int HCP_CALL hcp_AppendBytes(const hcp_Uint8* pSource, const hcp_Size_t Length, hcp_tBlob* pDestination);
extern hcp_Int HCP_CALL hcp_AppendByte(const hcp_Uint8 Value, hcp_tBlob* pDestination);
extern hcp_Int HCP_CALL hcp_AppendBlob(const hcp_tBlob* pSource, hcp_tBlob* pDestination);
extern hcp_Int HCP_CALL hcp_AppendZeroes(const hcp_Size_t Length, hcp_tBlob* pDestination);
extern hcp_Int HCP_CALL hcp_AppendString(const hcp_tString* pString, hcp_tBlob* pDestination, hcp_Uint8 Encoding);

extern hcp_Size_t HCP_CALL hcp_GetParameterSetSize(const hcp_tParameterSet* pParameters);
extern hcp_Size_t HCP_CALL hcp_GetTypeSize(const hcp_Int TypeId);
extern hcp_cszStr HCP_CALL hcp_GetTypeName(const hcp_Uint8 Id);
extern const hcp_Uint8 HCP_CALL hcp_GetTypeId(const hcp_tString* pName);

extern hcp_Int HCP_CALL hcp_InitializeCommandTemplates(hcp_tState* pState, hcp_tCommandTemplateSet* pTemplates);
extern hcp_Int HCP_CALL hcp_InitializeCommandTemplate(hcp_tState* pState, hcp_tCommandTemplate* pTemplate);
extern hcp_Int HCP_CALL hcp_InitializeProtocol(hcp_tState* pState, hcp_tProtocol* pProtocol);
extern hcp_Int HCP_CALL hcp_InitializeParameters(hcp_tState* pState, hcp_tParameterSet* pParameters);
extern hcp_Int HCP_CALL hcp_InitializeParameterTemplates(hcp_tState* pState, hcp_tParameterTemplateSet* pParameterTemplates);
extern hcp_Int HCP_CALL hcp_InitializeCommands(hcp_tState* pState, hcp_tCommandSet* pCommands, hcp_tModel* pTemplate);

extern hcp_Int HCP_CALL hcp_AppendParameters(hcp_tRuntime* R, hcp_tBlob* pDestination, hcp_tParameterSet* pParameters, const hcp_Uint8 Endianess, hcp_EncodeString StringEncoder, void* pContext);
extern void HCP_CALL hcp_Leftshift(hcp_tRuntime* R, hcp_tBlob* pSource, const hcp_Size_t Steps);
#endif /* Match the re-definition guard */

/*
*==============================================================================
* END OF FILE
*==============================================================================
*/
