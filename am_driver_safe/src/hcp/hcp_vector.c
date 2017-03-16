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
#include "hcp_vector.h"
#include "hcp_error.h"
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

/**	Returns the index of the first free element.
*-----------------------------------------------------------------------------
* \par	Description:
*		Checks the internal array for a free slot by calling the HasValue\n
*		callback specified in [hcp_InitializeVector].
*
* \param	pHeader	[IN]	Vector to search.
*
* \return	Returns -1 of no free element was found.
*/
static hcp_Size_t hcp_FindFirstFree(hcp_tVector* pHeader);

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

void hcp_Pop(hcp_tVector* pHeader,const hcp_Size_t Index) {
	if (pHeader == HCP_NULL || Index >= pHeader->length) {
		// lets be super forgiving
		return;
	}

	void* value = (void*)((hcp_Size_t)pHeader->values + Index * pHeader->elementSize);
	// check if there is a custom clear specified which will allow
	// items that store dynamic items (such as a vector) to deallocate
	if (pHeader->clear != HCP_NULL) {
		pHeader->clear(value, pHeader->context);
	}
	else {
		hcp_Memset(pHeader->parent, value, 0, pHeader->elementSize);
	}

	pHeader->length--;
}

hcp_Int hcp_InitializeVector(hcp_tState* pState, hcp_tVector* pHeader, void* pValues, const hcp_Size_t ElementSize, const hcp_Size_t MaxLength, hcp_ClearValue OnClear, hcp_CompareValue OnCompare, hcp_IsValue IsValue, void* pContext) {
	if (MaxLength < 1) {
		return HCP_VECTORTOOSMALL;
	}

	pHeader->clear = OnClear;
	pHeader->compare = OnCompare;
	pHeader->isValue = IsValue;
	pHeader->context = pContext;
	pHeader->elementSize = ElementSize;
	pHeader->maxLength = MaxLength;
	pHeader->length = 0;
	pHeader->parent = pState;

	if (hcp_IsDynamic(pState) == HCP_TRUE) {
		pHeader->values = HCP_NULL;
		pHeader->capacity = 0;
	}
	else {
		pHeader->values = pValues;
		pHeader->capacity = MaxLength;
		// clear the value array so we can determine if the slots are free
		// which is done by checking for zero-values when HasValue is not
		// not specified. HCP provides a default memset even though the
		// may not have mapped one
		hcp_Memset(pState, pHeader->values, 0, ElementSize*MaxLength);
	}

	return HCP_NOERROR;
}

hcp_Size_t hcp_FindFirst(const hcp_tVector* pHeader, const hcp_Size_t StartIndex, void* pValue, hcp_Boolean* pSuccess) {
	hcp_Size_t index = 0;
	*pSuccess = HCP_TRUE;

	while (index < pHeader->length) {
		void* pLhs = (void*)((hcp_Size_t)pHeader->values + index * pHeader->elementSize);

		if (pHeader->compare != HCP_NULL) {
			if (pHeader->compare(pLhs, pValue, pHeader->context) == 0) {
				return index;
			}
		}
		else if (pLhs == pValue) {
			return index;
		}

		index++;
	}

	*pSuccess = HCP_FALSE;
	return 0;
}

void* hcp_ValueAt(const hcp_tVector* pVector, const hcp_Size_t Index) {
	if (Index >= pVector->length) {
		return HCP_NULL;
	}

	return (void*)((hcp_Size_t)pVector->values + Index*pVector->elementSize);
}

hcp_Int hcp_PushEmpty(hcp_tVector* pHeader, hcp_Size_t* pIndex) {
	*pIndex = -1;
	if (pHeader->length >= pHeader->maxLength) {
		return HCP_VECTORFULL;
	}

	*pIndex = -1;
	hcp_tState* state = pHeader->parent;
	void* destination = HCP_NULL;

	// try to find a free slot
	hcp_Size_t index = hcp_FindFirstFree(pHeader);

	if (index > -1) {
		destination = (void*)((hcp_Size_t)pHeader->values + index*pHeader->elementSize);
	}
	else {
		if (hcp_IsDynamic(state)) {

			hcp_Size_t oldSize = pHeader->elementSize*pHeader->length;
			hcp_Size_t newSize = pHeader->elementSize*(pHeader->length + 1);

			void* dest = hcp_Malloc(state, newSize);

			if (dest == HCP_NULL) {
				return HCP_MALLOCFAILED;
			}

			if (pHeader->values != HCP_NULL) {
				// copy the old values
				hcp_Memcpy(state, dest, pHeader->values, oldSize);
				// release the old values
				hcp_Free(state, pHeader->values);
			}

			// swap buffers
			pHeader->values = dest;
			// our capacity has changed by one
			pHeader->capacity++;
		}
		// in both static and dynamic mode we copy the value to the last position
		// in the array
		destination = (void*)((hcp_Size_t)pHeader->values +
			pHeader->length * pHeader->elementSize);
		index = pHeader->length;
	}

	// clear the region
	hcp_Memset(pHeader->parent, destination, 0, pHeader->elementSize);
	pHeader->length++;
	*pIndex = index;
	return HCP_NOERROR;
}

hcp_Int hcp_Push(hcp_tVector* pHeader, void* pValue, hcp_Size_t* pIndex) {
	hcp_Int error = hcp_PushEmpty(pHeader, pIndex);

	if (error != HCP_NOERROR) {
		return error;
	}

	void* destination = hcp_ValueAt(pHeader, *pIndex);
	// insert the new value
	hcp_Memcpy(pHeader->parent, destination, pValue, pHeader->elementSize);

	return HCP_NOERROR;
}

/*
*==============================================================================
*  5.   LOCAL FUNCTIONS (declared in Section 3.5)
*==============================================================================
*/

hcp_Size_t hcp_FindFirstFree(hcp_tVector* pHeader) {
	void* values = pHeader->values;
	hcp_Size_t index = 0;

	while (index < pHeader->capacity) {
		void* current = (void*)((hcp_Size_t)values + pHeader->elementSize*index);

		if (pHeader->isValue != HCP_NULL) {
			if (pHeader->isValue(current, pHeader->context) == HCP_FALSE) {
				return index;
			}
		}
		else {
			// check if the entire memory block is equal to zero
			hcp_Uint8* end = (hcp_Uint8*)((hcp_Size_t)current + pHeader->elementSize);
			hcp_Uint8* start = (hcp_Uint8*)current;

			do {
				if (*start != 0) {
					break;
				}
			} while (++start < end);
			// was all element equal to zero?
			if (current == end) {
				return index;
			}
		}

		index++;
	}

	return -1;
}

/*
*==============================================================================
* END OF FILE
*==============================================================================
*/
