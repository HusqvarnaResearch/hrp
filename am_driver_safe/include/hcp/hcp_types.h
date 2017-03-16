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
/* Traverse this file only once */
#ifndef _HCP_COMMON_H_
#define _HCP_COMMON_H_
/*
*==============================================================================
*  2.      INCLUDE FILES
*==============================================================================
*/

#include <stdint.h>
/*
*==============================================================================
*  3.      DECLARATIONS
*  3.1     Global constants
*==============================================================================
*/

#ifndef HCP_VERSION
#	define HCP_VERSION 2
#endif

#define HCP_NULL 0
#define HCP_FALSE 0
#define HCP_TRUE 1

#define HCP_MAXSIZE_NAME 32			/* maximum number of characters for a name object*/
#define HCP_MAXSIZE_BUFFER 512		/* Maximum number of bytes in a tBuffer */
#define HCP_MAXSIZE_DYNAMIC 0x7FFF


#define HCP_MAXSIZE_COMMANDSETS 1	/* maximum number of commands sets that can be loaded into a single state */
#define HCP_MAXSIZE_CODECS 1	/* Maximum number of codecs allowed when no dynamic memory is avalible */
#define HCP_MAXSIZE_PARAMS 1		/* Maximum input or output parameters for a command */
#define HCP_MAXSIZE_COMMANDS 1	/* Maximum number of commands that can be loaded into a command-set.*/
#define HCP_MAXSIZE_PROTOCOLS 1	/* Maximum number of allowed codecs*/
#define HCP_MAXSIZE_TIFTEMPLATES 1	/* Maximum allowed number of TIF-templates (TIF-files) */
#define HCP_MAXSIZE_LIBRARIES 2	/* maximum number of product libraries that can be loaded when no dynamic memory is avalible */

#define HCP_TYPE_INVALID 0
#define HCP_TYPE_TSTRING 1

#define HCP_BIGENDIAN 0
#define HCP_LITTLEENDIAN 1

#define HCP_ENCODING_ASCII 0

#define HCP_ENDIANESS HCP_BIGENDIAN

#ifndef _x64
#	ifdef _WIN64
#		define _x64
#	endif
#endif

#ifdef _x64
#define HCP_SIZEMASK 0xFFFFFFFFFFFFFFFF
#else
#define HCP_SIZEMASK 0xFFFFFFFF
#endif

/*
*==============================================================================
*  3.2     Global macros
*==============================================================================
*/

#ifndef HCP_EXPORT
#	define HCP_EXPORT __declspec(dllexport)
#endif

#ifndef HCP_NOEXPORT
#	ifdef __cplusplus
#		define HCP_API extern "C" HCP_EXPORT
#	else
#		define HCP_API extern HCP_EXPORT
#	endif
#else
#	ifdef __cplusplus
#		define HCP_API extern "C"
#	else
#		define HCP_API extern
#	endif
#endif

#ifndef HCP_CALL
#	ifdef __gnu_linux__
#		define HCP_CALL
#	else
#		define HCP_CALL __cdecl
#	endif
#endif


/** Creates a new vector type.
*-----------------------------------------------------------------------------
* \par	Description:
*		Vector of elements that allows instances to handle collection of values in a uniform way.\n
*		The vector supports both static and dynamic memory. If the host has specified malloc and free\n
*		the vector will be allowed to grow as big as needed, where as if static memory is used, the\n
*		vector will always have a known and fixed size.
*
* \par TYPE	[IN]	Value type.
* \par	NAME	[IN]	Name of the new vector type.
* \par	MAXSTATIC	[IN]	Maximum number of elements in the vector if dynamic memory is NOT avalible.
*/
#define HCP_VECTOR(TYPE, NAME, MAXSTATIC) \
		typedef struct NAME {\
			hcp_tVector header;\
			TYPE fixed[MAXSTATIC];\
		} NAME;


/*
*==============================================================================
*  3.3     Global type definitions
*==============================================================================
*/

typedef double hcp_Double;		/* double precision floating point */
typedef float hcp_Float;		/* floating point */
typedef uint8_t hcp_Boolean;	/* binary boolean */
typedef void hcp_Void;			/* void */

/* platform specific long (4 bytes on win32, 8 on x64) */
#ifdef _x64
typedef unsigned long long hcp_Size_t;
#else
typedef unsigned long hcp_Size_t;
#endif

typedef uint8_t hcp_Uint8;
typedef int8_t hcp_Int8;
typedef uint16_t hcp_Uint16;
typedef int16_t hcp_Int16;
typedef uint32_t hcp_Uint32;
typedef int32_t hcp_Int32;
typedef uint64_t hcp_Uint64;
typedef int64_t hcp_Int64;
typedef hcp_Int32 hcp_Int;		/* platform specific int */
typedef char hcp_Char;			/* single character element */
typedef char* hcp_szStr;		/* zero-terminated string */
typedef char const* hcp_cszStr;		/* zero-terminated string */
typedef uint32_t hcp_UnixTime;
typedef uint16_t hcp_SimpleVersion;

#define HCP_DOUBLE_NAME "double"
#define HCP_FLOAT_NAME "float"
#define HCP_BOOLEAN_NAME "bool"
#define HCP_VOID_NAME "void"
#define HCP_SIZET_NAME "size_t"
#define HCP_UINT8_NAME "uint8"
#define HCP_INT8_NAME "sint8"
#define HCP_UINT16_NAME "uint16"
#define HCP_INT16_NAME "sint16"
#define HCP_UINT32_NAME "uint32"
#define HCP_INT32_NAME "sint32"
#define HCP_UINT64_NAME "uint64"
#define HCP_INT64_NAME "sint64"
#define HCP_STRING_NAME "ascii"
#define HCP_UNIXTIME_NAME "tUnixTime"
#define HCP_SIMPLEVERSION_NAME "tSimpleVersion"
#define HCP_BLOB_NAME "byteArray"
#define HCP_INVALID_NAME "invalid"

#define HCP_INVALID 0
#define HCP_FLOAT_ID 1
#define HCP_BOOLEAN_ID 2
#define HCP_VOID_ID 3
#define HCP_SIZET_ID 4
#define HCP_UINT8_ID 5
#define HCP_INT8_ID 6
#define HCP_UINT16_ID 7
#define HCP_INT16_ID 8
#define HCP_UINT32_ID 9
#define HCP_INT32_ID 10
#define HCP_UINT64_ID 11
#define HCP_INT64_ID 12
#define HCP_STRING_ID 13
#define HCP_DOUBLE_ID 14
#define HCP_BLOB_ID 15
#define HCP_UNIXTIME_ID 16
#define HCP_SIMPLEVERSION_ID 17

typedef struct {
	hcp_Uint8 id;
	hcp_cszStr name;
} hcp_tType;

/**
 *	Generic buffer which primarly is intended for buffert-storing data while reading
 *	or writing from a stream.
 */
typedef struct {
	hcp_Uint8 value[HCP_MAXSIZE_BUFFER];	/* fixed data buffer */
	hcp_Size_t length;	/* number of positions used in [value] */
} hcp_tBuffer;

/**
 *	Non-length fixed binary large object
 */
#pragma pack(push, 8)
typedef struct hcp_tBlob {
	hcp_Uint8* value;	/* value of the blob */
	hcp_Size_t length;		/* number of elements in [value] */
	hcp_Size_t maxLength;	/* maximum number of elements in [value] */
} hcp_tBlob;
#pragma pack(pop)

/**
 *	Non-length fixed string.
 */
#pragma pack(push, 8)
typedef struct {
	hcp_Char const* value;		/* value of the string */
	hcp_Size_t length;			/* number of characters in [value] */
	hcp_Boolean zeroTerm;	/* equal to HCP_TRUE if the string is terminated by a zero */
} hcp_tString;
#pragma pack(pop)

/**
 *	Generic value type.
 */
#pragma pack(push, 8)
typedef union {
	hcp_Double d;
	hcp_Float f;
	hcp_Boolean b;
	hcp_Size_t sz;
	hcp_Uint8 u8;
	hcp_Int8 s8;
	hcp_Uint16 u16;
	hcp_Int16 i16;
	hcp_Uint32 u32;
	hcp_Int32 i32;
	hcp_Uint64 u64;
	hcp_Int64 i64;
	hcp_UnixTime time;
	hcp_SimpleVersion version;
	hcp_Int i;
	const hcp_Void* p;
	hcp_tString str;
	hcp_tBlob blb;
} hcp_tValue;
#pragma pack(pop)

typedef struct hcp_tState hcp_tState;

/**	Deallocates a value object.
*-----------------------------------------------------------------------------
* \par	Description:
*		Callback function implemented by vector instances that allows a value\n
*		to have a custom destructor/deallocator. Implementors will cast [pValue]\n
*		into the actual type and free all related resources.
*
* \param	pValue	[IN]	Value to free.
* \param	pContext	[IN]	Custom callback context, specified with the creation\n
*								of the vector.
*/
typedef void(*hcp_ClearValue)(void* pValue, void* pContext);

/**	Compares to vector values.
*-----------------------------------------------------------------------------
* \par	Description:
*		Callback function implemented by vector instances which allows a value\n
*		to be compared to another, not just by it's pointer value but rather\n
*		its contents. Implementors casts [pLhs] and [rRhs] into the actual type\n
*		and then performs a comparison.
*
* \param	pValue	[IN]	Value to free.
* \param	pContext	[IN]	Custom callback context, specified with the creation\n
*								of the vector.
*
* \return	Returns an integral value indicating the relationship between the values.
* \retval	0    = The contents of both values are equal.
* \retval	-1	= The first value that does not match has a lower value in [pLhs] than in [pRhs].
* \retval	1	= The first value that does not match has a greater value in [pLhs] than in [pRhs].
*/
typedef hcp_Int(*hcp_CompareValue)(void* pLhs, void* pRhs, void* pContext);

/**	Checks if a handle refers to a active value.
*-----------------------------------------------------------------------------
* \par	Description:
*		Implementors should check if [pValue] refers to a value which is in use.\n
*		The vector will call this function when trying to find a new slot to\n
*		push a new value before trying to expand the underlaying array.
*
* \param	pValue	[IN]	Value to check for existance (if it can be overwritten.
* \param	pContext	[IN]	Callback specific context.
*/
typedef hcp_Boolean(*hcp_IsValue)(void* pValue, void* pContext);

/** Creates a new vector type.
*-----------------------------------------------------------------------------
* \par	Description:
*		Header definition for vector types which allows vectors of different type to be used
*		in a generic fashion.
*/
typedef struct {
	hcp_Void* values;
	hcp_Size_t elementSize;
	hcp_Size_t length;		/* number of active elements*/
	hcp_Size_t capacity;	/* number of allocate elements */
	hcp_Size_t maxLength;
	hcp_ClearValue clear;
	hcp_CompareValue compare;
	hcp_IsValue isValue;
	hcp_tState* parent;
	void* context;
} hcp_tVector;


/*
*==============================================================================
*  3.4     Global variables (defined in some implementation file)
*==============================================================================
*/

static const hcp_tType hcp_Types[] = {
	{ HCP_INVALID , HCP_INVALID_NAME},
	{ HCP_FLOAT_ID , HCP_FLOAT_NAME },
	{ HCP_BOOLEAN_ID , HCP_BOOLEAN_NAME },
	{ HCP_VOID_ID , HCP_VOID_NAME },
	{ HCP_SIZET_ID , HCP_SIZET_NAME },
	{ HCP_UINT8_ID , HCP_UINT8_NAME },
	{ HCP_INT8_ID , HCP_INT8_NAME },
	{ HCP_UINT16_ID , HCP_UINT16_NAME },
	{ HCP_INT16_ID , HCP_INT16_NAME },
	{ HCP_UINT32_ID , HCP_UINT32_NAME },
	{ HCP_INT32_ID , HCP_INT32_NAME },
	{ HCP_UINT64_ID , HCP_UINT64_NAME },
	{ HCP_INT64_ID , HCP_INT64_NAME },
	{ HCP_STRING_ID , HCP_STRING_NAME },
	{ HCP_DOUBLE_ID , HCP_DOUBLE_NAME },
	{ HCP_BLOB_ID , HCP_BLOB_NAME},
	{ HCP_UNIXTIME_ID, HCP_UNIXTIME_NAME },
	{ HCP_SIMPLEVERSION_ID, HCP_SIMPLEVERSION_NAME},
	{0, HCP_NULL}
};

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


#endif /* Match the re-definition guard */

/*
*==============================================================================
* END OF FILE
*==============================================================================
*/
