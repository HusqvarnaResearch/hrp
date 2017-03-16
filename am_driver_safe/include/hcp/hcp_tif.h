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
#ifndef _HCP_TIF_H_
#define _HCP_TIF_H_
/*
*==============================================================================
*  2.      INCLUDE FILES
*==============================================================================
*/

#include "hcp_model.h"
#include "hcp_error.h"
/*
*==============================================================================
*  3.      DECLARATIONS
*  3.1     Global constants
*==============================================================================
*/

#define HCP_TIF_HEADER "header"
#define HCP_TIF_HEADER_SCHEMA "schema"
#define HCP_TIF_HEADER_VERSION "version"
#define HCP_TIF_HEADER_CREATED "created"
#define HCP_TIF_HEADER_PROTOCOL "protocol"

#define HCP_TIF_METHOD "methods"
#define HCP_TIF_COMMAND "command"
#define HCP_TIF_ELEMENTTYPE "elementType"
#define HCP_TIF_FAMILY "family"
#define HCP_TIF_INPARAMS "inParams"
#define HCP_TIF_OUTPARAMS "outParams"
#define HCP_TIF_PARAMNAME "name"
#define HCP_TIF_PARAMTYPE "type"
#define HCP_TIF_PROTOCOL "protocol"
#define HCP_TIF_PROTOCOLKEY "key"
#define HCP_TIF_PROTOCOLVALUE "value"
#define HCP_TIF_PARAMLENGTH "length"
#define HCP_TIF_TYPES "types"
#define HCP_TIF_TYPENAME "name"
#define HCP_TIF_TYPETYPE "type"

#define HCP_TIF_SEPARATOR '.'
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

	/**	Loads a TIF-Command definition file into a command set.
	 *-----------------------------------------------------------------------------
	 * \par	Description:
	 *		Parses [TIFString] into a JSON-object and then tries to extract TIF-\n
	 *		command definitions which is then loaded into a command set.
	 *
	 * \param	pState	[IN]	State to use for memory operations.
	 * \param	pText [IN]	String to parse.
	 * \param	pTemplate [OUT]	Output command set template which can be used\n
	 *								to create command-sets.
	 *
	 * \return	Success
	 * \retval	HCP_NOERROR	=	The text was successfully parsed.
	 *-----------------------------------------------------------------------------
	 */
	extern hcp_Int HCP_CALL hcp_LoadTIFTemplate(hcp_tState* pState,const hcp_tString* pText, hcp_tModel* pTemplate);
	/**	Parses a string into a TIF-command.
	*-----------------------------------------------------------------------------
	* \par	Description:
	*		Parses [TIFCommand] into a command element which is then resolved via\n
	*		[pCommands]. If a loaded command is found, that command is populated\n
	*		with the parsed data and outputted via [ppCommand].
	*
	* \param	TIFCommand [IN]	String to parse.
	* \param	pCommands [IN]	Loaded command set.
	* \param	ppCommand	[OUT]	Populated command. Will be null of the return\n
	*								value is not equal to HCP_NOERROR.
	*
	* \return	Returns a error structure which in detail describes any parse-error(s).
	*-----------------------------------------------------------------------------
	*/
	extern hcp_Int HCP_CALL hcp_ParseTifCommand(const hcp_tString* TIFCommand, hcp_tCommandSet* pCommands, hcp_tCommand** ppCommand);

#endif /* Match the re-definition guard */

/*
*==============================================================================
* END OF FILE
*==============================================================================
*/