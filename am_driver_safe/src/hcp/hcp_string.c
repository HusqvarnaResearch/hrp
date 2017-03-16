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
#include "hcp_string.h"
#include "hcp_error.h"
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

static hcp_Int hcp_ToBytes(void* pNumber, const hcp_Size_t NumberOfBytes, hcp_tBlob* pDestination, const hcp_Uint8 Endianess);

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

hcp_Int hcp_szStrCmp(hcp_cszStr Lhs, hcp_cszStr Rhs) {
	if (Lhs == HCP_NULL && Rhs == HCP_NULL) {
		return 0;
	}

	if (Lhs == HCP_NULL) {
		return -1;
	}

	if (Rhs == HCP_NULL) {
		return 1;
	}

	hcp_Char const* lhs = Lhs;
	hcp_Char const* rhs = Rhs;

	while (*lhs != HCP_NULL) {
		if (*rhs == HCP_NULL) {
			return 1;
		}

		if (*lhs != *rhs) {
			return 1;
		}

		lhs++; rhs++;
	}

	return *lhs == *rhs ? 0 : -1;;
}


hcp_Int32 hcp_Atio(const hcp_tString* pNumber) {
	if (pNumber == HCP_NULL || pNumber->length == 0) {
		return 0;
	}

	hcp_Char const* start = pNumber->value;;
	hcp_Int multiplyer = 1;
	hcp_Int base = 0;
	hcp_Size_t length = pNumber->length;
	hcp_Int32 scale = 1;

	if (hcp_IsHexadecimal(pNumber) == HCP_TRUE) {
		if (pNumber->length > 1) {
			// skip 0x ?
			if (*start == '0' && *(start + 1) == 'x') {
				start += 2;
				length -= 2;
			}
		}

		base = 16;
	}
	else if (hcp_IsDecimal(pNumber) == HCP_TRUE) {
		base = 10;
		
		if (*start == '-') {
			scale = -1;
			start++; length--;
		}
	}
	else {
		// invalid value
		return 0;
	}

	hcp_Char const* end = (hcp_Char*)((hcp_Size_t)start + length - 1);
	hcp_Int32 value = 0;

	for (; end >= start; end--) {
		value += hcp_CharacterToInt(*end) * multiplyer;
		multiplyer *= base;
	}

	return value*scale;
}

hcp_Int hcp_tStrSzCmp(const hcp_tString* pLhs, const hcp_cszStr pRhs) {
	if (pRhs == HCP_NULL) {
		return 1;
	}

	if (pLhs == HCP_NULL) {
		return -1;
	}

	const hcp_Size_t maxLength = pLhs->length;
	hcp_Size_t length = 0;
	hcp_Char const* lhs = pLhs->value;
	hcp_Char const* rhs = pRhs;

	while (length < maxLength) {

		if (*rhs == 0) {
			return 1;
		}

		if (*lhs != *rhs) {
			return -1;
		}


		lhs++; rhs++;
		length++;
	}

	if (pLhs->zeroTerm == HCP_FALSE && *(++rhs) != 0) {
		return 1;
	}

	return 0;
}

hcp_Boolean hcp_IsAlphaNumerical(const hcp_Char Character) {
	return ((Character >= 'A' && Character <= 'Z') || (Character >= 'a' && Character <= 'z')) ? HCP_TRUE : HCP_FALSE;
}

hcp_Boolean hcp_IsDigit(const hcp_Char Character) {
	return ((Character == '-') || (Character >= '0' && Character <= '9')) ? HCP_TRUE : HCP_FALSE;
}

hcp_Boolean hcp_IsHexadecimal(const hcp_tString* pNumber) {
	if (pNumber == HCP_NULL) {
		return HCP_FALSE;
	}

	hcp_Size_t length = pNumber->length;
	hcp_Char const* value = pNumber->value;

	if (length == 0 || value == HCP_FALSE) {
		return HCP_FALSE;
	}

	hcp_Boolean isHex = HCP_FALSE;

	// 0x?
	if (*value == '0' && pNumber->length > 1) {
		value++;

		if (*value == HCP_NULL || *value != 'x') {
			return HCP_FALSE;
		}

		value++;
		isHex = HCP_TRUE;
	}

	while (*value != HCP_NULL && length > 0) {
		if ((*value >= 65 && *value <= 70) || (*value >= 97 && *value <= 102)) {
			isHex = HCP_TRUE;
		}
		else if (!(*value >= 48 && *value <= 57)) {
			return HCP_FALSE;
		}

		value++;
		length--;
	}

	return isHex;
}

hcp_Boolean hcp_IsDecimal(const hcp_tString* Number) {
	if (Number == HCP_NULL) {
		return HCP_FALSE;
	}

	hcp_Size_t length = Number->length;
	hcp_Char const* value = Number->value;

	if (*value == '-') {
		value++; length--;
	}

	while (length > 0 && *value != HCP_NULL) {
		if (!(*value >= 48 && *value <= 57)) {
			return HCP_FALSE;
		}

		value++;
		length--;
	}

	return HCP_TRUE;
}

hcp_Int hcp_tStrCmp(const hcp_tString* pLhs, const hcp_tString* pRhs) {
	if (pLhs == 0) {
		return -1;
	}

	if (pRhs == 0) {
		return 1;
	}

	if (pLhs->length < pRhs->length) {
		return -1;
	}
	else if (pLhs->length > pRhs->length) {
		return 1;
	}

	hcp_Size_t length = 0;
	hcp_Char const* lhs = pLhs->value;
	hcp_Char const* rhs = pRhs->value;

	while (length < pLhs->length) {
		if (*lhs != *rhs) {
			return -1;
		}

		lhs++; rhs++; length++;
	}

	return 0;
}

hcp_Int hcp_szStrLen(hcp_cszStr String) {
	if (String == HCP_NULL) {
		return 0;
	}

	hcp_cszStr value = String;
	hcp_Int length = 0;

	while (*value != 0) {
		value++;
		length++;
	}

	return length;
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
