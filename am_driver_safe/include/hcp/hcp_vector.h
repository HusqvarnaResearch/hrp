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
#ifndef _HCP_VECTOR_H_
#define _HCP_VECTOR_H_

/*
*==============================================================================
*  2.      INCLUDE FILES
*==============================================================================
*/

#include "hcp_types.h"
#include "hcp_error.h"
/*
*==============================================================================
*  3.      DECLARATIONS
*  3.1     Global constants
*==============================================================================
*/

#define HCP_MAXSIZE_VOIDVECTOR_STATIC 16	/* Default maximum number of elements in a hcp_tVoidVector\n
											 * when dynamic memory is NOT avalible. */

/*
*==============================================================================
*  3.2     Global macros
*==============================================================================
*/


	
	 /** Simplifies calls to [hcp_InitializeVector]
	  *-----------------------------------------------------------------------------
	  * \par	Description:
	  *			Makes calls to [hcp_InitializeVector] more compact.
	  *
	  * \par	STATE	[IN]	State that will be used for malloc (if avalible)
	  * \par	HEADER	[IN]	Handle to vector header.
	  * \par	FIXED	[IN]	Handle to fixed vector storage.
	  * \par	TYPE	[IN]	The type of element that should be stored.
	  * \par	ONCLEAR	[IN]	Callback when a value is cleared.
	  * \par	ONCOMPARE	[IN]	callback to find a element in a vector.
	  * \par	ISEMPTY	[IN]	Callback to determine if a slot is avalible
	  */
#define HCP_INITIALIZEVECTOR(STATE, HEADER, FIXED, TYPE, ONCLEAR, ONCOMPARE, ISEMPTY) \
	hcp_InitializeVector(STATE, HEADER, FIXED, sizeof(TYPE), \
		hcp_IsDynamic(STATE) == HCP_TRUE ? HCP_MAXSIZE_DYNAMIC : \
		sizeof(FIXED) / sizeof(TYPE),ONCLEAR,ONCOMPARE,ISEMPTY,STATE);

/*
*==============================================================================
*  3.3     Global type definitions
*==============================================================================
*/



	/** Vector of void elements.
	 *-----------------------------------------------------------------------------
	 * \par	Description:
	 *		Small void vector, main intent is to make sure the HCP_VECTOR is working.
	 */
	HCP_VECTOR(void*, hcp_tVoidVector, HCP_MAXSIZE_VOIDVECTOR_STATIC)
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
*  4.      GLOBAL FUNCTIONS
*==============================================================================
*/

	/**	Returns a value at a given index.
	 *-----------------------------------------------------------------------------
	 * \par	Description:
	 *		Returns the value stored at a specfic index in the vector. The element-\n
	 *		size specified in [hcp_InitializeVector] is used to determine the address\n
	 *		of the value.
	 *
	 * \param	pVector	[IN]	Vector who's value(s) to return.
	 * \param	Index	[IN]	Zero-based index where the value is located.
	 *
	 * \return	Returns a handle to the value.
	 * \retval	HCP_NULL	=	Invalid index (out of range).
	 * \retval	!= HCP_NULL	=	Requested value.
	 */
	extern void* HCP_CALL hcp_ValueAt(const hcp_tVector* pVector, const hcp_Size_t Index);
	/**	Initialzes a vector.
	 *-----------------------------------------------------------------------------
	 * \par	Description:
	 *
	 * \param	pState	[IN]	State contaning OS malloc and free mapping.
	 * \param	pHeader [IN]	Vector header.
	 * \param	pValues	[IN]	Reference to static store (fixed array) which is used when dynamic\n
	 *							memory isnt avalible.
	 * \param	ElementSize [IN]	The size of a individual element in the vector.
	 * \param	MaxLength [IN]	Maximum number of elements allowed in the vector. If dynamic\n
	 *						memory is not avalible, then this limit is ignored if it's larger\n
	 *						than the size of the static memory vector.
	 * \param	OnClear	[IN]	Custom free-function which allows one to provide a callback function\n
	 *						that is invoked each time a value is remove (deallocated) from the vector.
	 * \param	OnCompare	[IN]	Callback function which allows comparison to be made on object-level\n
	 *							rahter than just pointer handle value comparison.
	 * \param	HasValue	[IN]	Invoked when a push is made and the vector needs to find a empty slot.
	 * \param pContext	[IN]	Callback context which is passed to [OnClear] and [OnCompare].
	 *
	 *
	 *
	 * \return	Success status.
	 * \retval	HCP_NOERROR	=	Success	
	 * \retval	HCP_VECTORTOOSMALL	=	The vector must contain at least one element.
	 * \retval	HCP_MALLOCFAILED	=	Failed to allocate new memory array (when dynmic memory\n
	 *								was supported).
	 */
	extern hcp_Int HCP_CALL hcp_InitializeVector(hcp_tState* pState, hcp_tVector* pHeader, void* pValues, const hcp_Size_t ElementSize, const hcp_Size_t MaxLength, hcp_ClearValue OnClear, hcp_CompareValue OnCompare, hcp_IsValue IsValue, void* pContext);
	/** Pushes a value onto a vector.
	 *-----------------------------------------------------------------------------
	 * \par	Description:
	 *		Adds a value to a existing vector. If the state using which the vector\n
	 *		was created supports dynamic memory, the vector will expand by allocating
	 *		more memory.
	 *
	 * \param	pHeader	[IN]	Vector header.
	 * \param	pValue	[IN]	Value to store.
	 * \param	pIndex	[OUT]	Outputs the index where the new item was pushed.
	 *
	 * \return	Operation result.
	 * \retval	HCP_NOERROR	=	Success	
	 * \retval	HCP_MAXELEMENTS	=	The vector has reached the maximum number of allowed\n
	 *								elements.
	 * \retval	HCP_MALLOCFAILED	=	Returned when malloc fails when dynamic memory\n
	 *									is avalible.
	 *-----------------------------------------------------------------------------
	 */
	extern hcp_Int HCP_CALL hcp_Push(hcp_tVector* pHeader, void* pValue, hcp_Size_t* pIndex);
	/** Pushes a empty value onto a vector.
	 *-----------------------------------------------------------------------------
	 * \par	Description:
	 *		Adds a empty element to a existing vector. If the state using which the vector\n
	 *		was created supports dynamic memory, the vector will expand by allocating
	 *		more memory.
	 *
	 * \param	pHeader	[IN]	Vector header.
	 * \param	pValue	[IN]	Value to store.
	 * \param	pIndex	[OUT]	Outputs the index where the new item was pushed.
	 *
	 * \return	Operation result.
	 * \retval	HCP_NOERROR	=	Success
	 * \retval	HCP_MAXELEMENTS	=	The vector has reached the maximum number of allowed\n
	 *								elements.
	 * \retval	HCP_MALLOCFAILED	=	Returned when malloc fails when dynamic memory\n
	 *									is avalible.
	 *-----------------------------------------------------------------------------
	 */
	extern hcp_Int HCP_CALL hcp_PushEmpty(hcp_tVector* pHeader, hcp_Size_t* pIndex);
	/** Pops a value from a vector.
	*-----------------------------------------------------------------------------
	* \par	Description:
	*		Erases the memory at a vector position by calling the [OnClear] callback\n
	*		if specified. If no callback is avalible, the memory area will be set to\n
	*		zero, which is equivalent to calling memset on the specified element.
	*
	* \param	pHeader	[IN]	Vector header.
	* \param	Index	[IN]	Index where the element is stored.
	* \param	pIndex	[OUT]	Outputs the index where the new item was pushed.
	*
	*-----------------------------------------------------------------------------
	*/
	extern void HCP_CALL hcp_Pop(hcp_tVector* pHeader,const hcp_Size_t Index);
	/**	Returns the index of the first match of a value.
	 *-----------------------------------------------------------------------------
	 * \par	Description:
	 *		Attempts to locate the first instance of a object inside the vector by\n
	 *		comparing to [pValue]. If the vector had a compare method specified\n
	 *		when it was initialized via [hcp_InitializeVector], that callback will\n
	 *		be used to determined of two elements match.
	 *
	 * \param	pHeader	[IN]	Vector to search.
	 * \param	StartIndex	[IN]	Zero-based index to begin the search.
	 * \param	pValue	[IN]	Value to locate.
	 * \param	pSuccess [OUT]	Set to HCP_TRUE if a match was found.
	 *
	 * \return	Returns the index where the first match was located.
	 *-----------------------------------------------------------------------------
	 */
	extern hcp_Size_t HCP_CALL hcp_FindFirst(const hcp_tVector* pHeader, const hcp_Size_t StartIndex, void* pValue, hcp_Boolean* pSuccess);


#endif /* Match the re-definition guard */

/*
*==============================================================================
* END OF FILE
*==============================================================================
*/