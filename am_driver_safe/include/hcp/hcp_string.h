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
#ifndef _HCP_STRING_H_
#define _HCP_STRING_H_
/*
*==============================================================================
*  2.      INCLUDE FILES
*==============================================================================
*/

#include "hcp_types.h"
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


extern hcp_Int HCP_CALL hcp_szStrLen(hcp_cszStr String);
extern hcp_Int HCP_CALL hcp_szStrCmp(hcp_cszStr Lhs, hcp_cszStr Rhs);
extern hcp_Int HCP_CALL hcp_tStrCmp(const hcp_tString* pLhs, const hcp_tString* pRhs);
extern hcp_Int HCP_CALL hcp_tStrSzCmp(const hcp_tString* pLhs, const hcp_cszStr pRhs);
extern hcp_Boolean HCP_CALL hcp_IsDecimal(const hcp_tString* Number);
extern hcp_Boolean HCP_CALL hcp_IsAlphaNumerical(const hcp_Char Character);
extern hcp_Boolean HCP_CALL hcp_IsDigit(const hcp_Char Character);
extern hcp_Boolean HCP_CALL hcp_IsHexadecimal(const hcp_tString* Number);
extern hcp_Int32 HCP_CALL hcp_Atio(const hcp_tString* pNumber);
extern hcp_Int HCP_CALL hcp_CharacterToInt(const hcp_Char Character);
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
