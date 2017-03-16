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
#ifndef _HCP_ERROR_H_
#define _HCP_ERROR_H_
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

#define HCP_NOERROR 0			/* the operation completed successfully */
#define HCP_INVALIDSTATE -1		/* the provided handle is not a valid state object */
#define HCP_NOFREE -2			/* malloc was provided but no free method */
#define HCP_MALLOCFAILED -3		/* malloc returned zero */
#define HCP_MAXELEMENTS -4		/* the vector count exceeded MAXELEMENTS */
#define HCP_VECTORTOOSMALL -5	/* the vector must contain at least one element */
#define HCP_VECTORFULL -6		/* the vector is full and cannot contain more elements */
#define HCP_INVALIDLIB -7		/* the selected codec library has not been loaded */
#define HCP_NOUNLOCK -8			/* the host specified a trylock-function but no unlock */
#define HCP_LOCKTIMEOUT -9		/* unable to acquire mutex lock */
#define HCP_INVALIDID -10		/* invalid codec id*/
#define HCP_NONSZSTRING -11		/* expected zero-terminated string */
#define HCP_PARSEJSONFAILED -12	/* failed to convert the string into JSON */
#define HCP_MISSINGFAMILY -13	/* missing family element in tif-string */
#define HCP_MISSINGCOMMAND -14	/* missing command element in tif-string */
#define HCP_COMMANDNOTLOADED -15	/* the specified command has not been loaded into the codec */
#define HCP_NOARGUMENTNAME -16	/* the expected argument name */
#define HCP_NOMATCHINGPARAMETER -17	/* the specified argument name was no a part of the commands in parameter set */
#define HCP_MISSINGHEADER -18	/* missing header element */
#define HCP_MISSINGSCHEMA -19	/*the tif file did not contain a header-schema property */
#define HCP_MISSINGCODEC -20	/* the node did not contain a codec node or it wasnt an array */
#define HCP_MISSINGKEY -21		/* the codec item did not contain a key-node*/
#define HCP_MISSINGSTRING -22	/* the requested json-element was not found */
#define HCP_MISSING_PARAMETERNAME -23	/* parameter name node not found */
#define HCP_MISSING_PARAMETERTYPE -24	/* parameter type node not found */
#define HCP_INVALID_PARAMETERTYPE -25	/* the specified type id was invalid*/
#define HCP_INVALIDTEMPLATEID -26	/* the specified template id did not refer to any stored templates */
#define HCP_MISSINGENDOFSTRING -27	/* expected end of string */
#define HCP_INVALIDARGUMENTVALUE -28	/* unable to parse tif-argument value */
#define HCP_MISSINGDIGIT -29	/* expected next character to be digit*/
#define HCP_INVALIDSPACECHAR -30	/* encountered space in digits */
#define HCP_MISSINGTERMINATOR -31	/* missing string terminator */
#define HCP_MISSINGENDOFCOMMAND -32 /* expected , or ) */
#define HCP_INVALIDLIBRARY -33	/* null library handle */
#define HCP_INVALIDBLOB -34	/* null blob handle*/
#define HCP_BLOBOUTOFRANGE -35	/* blob too small */
#define HCP_MISSINGCODECNODE -36 /* missing key/vaue pair in codec collection */
#define HCP_INVALIDENDIANESS -37	/* the specified endinaess was not supported */
#define HCP_INVALIDTYPEID -38	/* the specified type is not supported */
#define HCP_INVALIDENCODING -39	/* invalid encoding */
#define HCP_SERIALIZENOTSUPPORTED -40	/* the selected library does not support serialization  */
#define HCP_DESERIALIZENOTSUPPORTED -41	/* the selected library does not support deserialization */
#define HCP_RESIZEFAILED -42	/* unable to resize buffer since it was too large and no malloc was avalible */
#define HCP_INVALIDPARAMETER_UNKNOWNSTRINGNOTATEND -43	/* unknown string length strings must be the last argument in a parameter list*/
#define HCP_NULLCODECLOAD -44	/** null load codec handle*/
#define HCP_CODECLOADFAILED -45
#define HCP_INVALIDCODECNAME -46 /* codec name null or already exists */
#define HCP_CODECALREADYLOADED -47
#define HCP_STRINGNOTSUPPORTED -48	/* codec library does not support strings */
#define HCP_MISSINGBYTEARRAY_SIZE -49 /* byte-array of unknown size must be the last parameter in a list */
#define HCP_INVALID_STRINGSIZE -50 /* the passed string length exceeded the max length set in the tif-file */

#define HCP_NOERROR_MSG "Success"
#define HCP_INVALIDSTATE_MSG "The state handle was invalid."
#define HCP_NOFREE_MSG "Malloc was provided but no Free method."
#define HCP_MALLOCFAILED_MSG "Malloc failed."
#define HCP_MAXELEMENTS_MSG "The vector has reached it'specified max capacity."
#define HCP_VECTORTOOSMALL_MSG "A vector must contain atleast one element."
#define HCP_VECTORFULL_MSG "Unable to push new element since the vector is full." 
#define HCP_INVALIDLIB_MSG "The selected codec library has not been loaded."
#define HCP_NOUNLOCK_MSG "The host specified a TryLock function but no Unlock."
#define HCP_LOCKTIMEOUT_MSG "Unable to acquire a thread-lock."
#define HCP_INVALIDID_MSG "The passed id did not refer to a valid codec."
#define HCP_NONSZSTRING_MSG "Expected a zero-terminated string."
#define HCP_PARSEJSONFAILED_MSG "An error occured while converting a string into a JSON object."
#define HCP_MISSINGFAMILY_MSG "The TIF-command did not contain a family."
#define HCP_MISSINGCOMMAND_MSG "The TIF-command did not contain a command."
#define HCP_COMMANDNOTLOADED_MSG "The specified command has not been loaded into the codec."
#define HCP_NOARGUMENTNAME_MSG "The TIF-command contain a argument with no name."
#define HCP_NOMATCHINGPARAMETER_MSG "The argument name was not found in the command's in-parameter set."
#define HCP_MISSINGHEADER_MSG "Missing header-node."
#define HCP_MISSINGSCHEMA_MSG "The header-node did not contain a schema node."
#define HCP_MISSINGCODEC_MSG "The codec node was missing or not an array."
#define HCP_MISSINGKEY_MSG "The codec item did not contain a key-node."
#define HCP_MISSINGSTRING_MSG "The requested JSON-node was not found."
#define HCP_MISSING_PARAMETERNAME_MSG "The parameter node did not contain a name."
#define HCP_MISSING_PARAMETERTYPE_MSG "The parameter node did not contain a type."
#define HCP_INVALID_PARAMETERTYPE_MSG "Invalid parameter type."
#define HCP_INVALIDTEMPLATEID_MSG "Invalid template id."
#define HCP_MISSINGENDOFSTRING_MSG "Expected end of string character '\""
#define HCP_INVALIDARGUMENTVALUE_MSG "Invalid argument-value format."
#define HCP_MISSINGDIGIT_MSG "Invalid argument-value, expected digit."
#define HCP_INVALIDSPACECHAR_MSG "Encountered a space when parsing numberical argument value."
#define HCP_MISSINGTERMINATOR_MSG "Expected string terminator."
#define HCP_MISSINGENDOFCOMMAND_MSG "Expected , or ) to terminate command. "
#define HCP_INVALIDLIBRARY_MSG "The specified codec library was invalid."
#define HCP_INVALIDBLOB_MSG "The blob handle was invalid (null)."
#define HCP_BLOBOUTOFRANGE_MSG "Attempted to write too many bytes to blob."
#define HCP_MISSINGCODECNODE_MSG "The requested key/value-pair was not found in the codec-set."
#define HCP_INVALIDENDIANESS_MSG "Invalid (non-supported) endinaess when converting to bytes."
#define HCP_INVALIDTYPEID_MSG "The specified type-id is not supported (invalid)."
#define HCP_INVALIDENCODING_MSG "Unable to cast string into byte(s), invalid encoding specified."
#define HCP_SERIALIZENOTSUPPORTED_MSG "The selected codec library does not support serialization."
#define HCP_DESERIALIZENOTSUPPORTED_MSG "The selected codec library does not support deserialization."
#define HCP_RESIZEFAILED_MSG "Unable to resize buffer, the requested length was too large when no dynamic memory was defined."
#define HCP_INVALIDPARAMETER_UNKNOWNSTRINGNOTATEND_MSG "Unknown-length-strings must be last argument in a parameter list."
#define HCP_NULLCODECLOAD_MSG "The codec-load method was empty."
#define HCP_CODECLOADFAILED_MSG "Codec library load did not return a header."
#define HCP_INVALIDCODECNAME_MSG "The codec name was empty or already loaded."
#define HCP_CODECALREADYLOADED_MSG "A codec with the same name has already been loaded."
#define HCP_STRINGNOTSUPPORTED_MSG "The codec does not support strings."
#define HCP_MISSINGBYTEARRAY_SIZE_MSG "Output byte-arrays with a unknown length must be the last parameter."
#define HCP_INVALID_STRINGSIZE_MSG "The string-parameter's length was larger than the length specified in the length field."
/*
*==============================================================================
*  3.2     Global macros
*==============================================================================
*/
#define HCP_NEWERROR(E, M, R, C) hcp_NewError(E, __LINE__, __FILE__, M, R, C);
/*
*==============================================================================
*  3.3     Global type definitions
*==============================================================================
*/

	/** Resolves a readable message from a error code.
	  *-----------------------------------------------------------------------------
	  * \par     Description:
	  *          Looks through a internal dictionary and copies a human readable\n
	  *          error message to [pDest]
	  *
	  * \param   Error  [IN]  Error code to resolve.
	  * \param   pDest [OUT] Destination string. 
	  * \param	pContext [IN] Callback specific context.
	  *
	  *-----------------------------------------------------------------------------
	  */
	typedef void(*hcp_ErrorSource)(const hcp_Int16 Error, hcp_tString* pDest, void* pContext);

	/** Error structure
	 *-----------------------------------------------------------------------------
	 * \par	Description:
	 *		A error structure primarly intended for codec libraries so they can provide in detail
	 *		error messages to ease fault tracing. External modules which is not a part of the core
	 *		runtime is also encouraged to return tError objects instead of integers to ease future
	 *		fault finding.
	 */
	typedef struct {
		const hcp_Int16 error;		/*	result code, equal to HCP_NOERROR if \n
									the operation completed successfully */
		const hcp_Int line;				/*  line number where the error occured */
		const hcp_tString file;		/* name of the file where the error occured */
		const hcp_tString module;	/*	name of the module which where the error was caught */
		hcp_ErrorSource resolve;	/*  function for resolving a readable error message */
		void* context;				/*  module specific context required to resolve the error message */
	} hcp_tError;

	typedef struct {
		hcp_Int code;
		hcp_cszStr message;
	}hcp_tErrorMessage;
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

	static const hcp_tErrorMessage hcp_Errors[] = {
		{ HCP_NOERROR , HCP_NOERROR_MSG },
		{ HCP_INVALIDSTATE , HCP_INVALIDSTATE_MSG },
		{ HCP_NOFREE, HCP_NOFREE_MSG },
		{ HCP_MALLOCFAILED , HCP_MALLOCFAILED_MSG },
		{ HCP_MAXELEMENTS, HCP_MAXELEMENTS_MSG },
		{ HCP_VECTORTOOSMALL , HCP_VECTORTOOSMALL_MSG },
		{ HCP_VECTORFULL, HCP_VECTORFULL_MSG },
		{ HCP_INVALIDLIB, HCP_INVALIDLIB_MSG },
		{ HCP_NOUNLOCK, HCP_NOUNLOCK_MSG },
		{ HCP_LOCKTIMEOUT, HCP_LOCKTIMEOUT_MSG},
		{ HCP_INVALIDID, HCP_INVALIDID_MSG },
		{ HCP_NONSZSTRING, HCP_NONSZSTRING_MSG },
		{ HCP_PARSEJSONFAILED , HCP_PARSEJSONFAILED_MSG},
		{ HCP_MISSINGFAMILY , HCP_MISSINGFAMILY_MSG },
		{ HCP_MISSINGCOMMAND, HCP_MISSINGCOMMAND_MSG },
		{ HCP_COMMANDNOTLOADED , HCP_COMMANDNOTLOADED_MSG },
		{ HCP_NOARGUMENTNAME , HCP_NOARGUMENTNAME_MSG},
		{ HCP_NOMATCHINGPARAMETER, HCP_NOMATCHINGPARAMETER_MSG},
		{ HCP_MISSINGHEADER, HCP_MISSINGHEADER_MSG },
		{ HCP_MISSINGSCHEMA , HCP_MISSINGSCHEMA_MSG},
		{ HCP_MISSINGCODEC, HCP_MISSINGCODEC_MSG},
		{ HCP_MISSINGKEY , HCP_MISSINGKEY_MSG },
		{ HCP_MISSINGSTRING , HCP_MISSINGSTRING_MSG },
		{ HCP_MISSING_PARAMETERNAME, HCP_MISSING_PARAMETERNAME_MSG },
		{ HCP_MISSING_PARAMETERTYPE , HCP_MISSING_PARAMETERTYPE_MSG},
		{ HCP_INVALID_PARAMETERTYPE, HCP_INVALID_PARAMETERTYPE_MSG},
		{ HCP_INVALIDTEMPLATEID , HCP_INVALIDTEMPLATEID_MSG },
		{ HCP_MISSINGENDOFSTRING, HCP_MISSINGENDOFSTRING_MSG },
		{ HCP_INVALIDARGUMENTVALUE , HCP_INVALIDARGUMENTVALUE_MSG },
		{ HCP_MISSINGDIGIT , HCP_MISSINGDIGIT_MSG},
		{ HCP_INVALIDSPACECHAR, HCP_INVALIDSPACECHAR_MSG },
		{ HCP_MISSINGTERMINATOR, HCP_MISSINGTERMINATOR_MSG },
		{ HCP_MISSINGENDOFCOMMAND , HCP_MISSINGENDOFCOMMAND_MSG },
		{ HCP_INVALIDLIBRARY , HCP_INVALIDLIBRARY_MSG },
		{ HCP_INVALIDBLOB , HCP_INVALIDBLOB_MSG },
		{ HCP_BLOBOUTOFRANGE , HCP_BLOBOUTOFRANGE_MSG},
		{ HCP_MISSINGCODECNODE , HCP_MISSINGCODECNODE_MSG },
		{ HCP_INVALIDENDIANESS , HCP_INVALIDENDIANESS_MSG },
		{ HCP_INVALIDTYPEID, HCP_INVALIDTYPEID_MSG },
		{ HCP_INVALIDENCODING , HCP_INVALIDENCODING_MSG },
		{ HCP_SERIALIZENOTSUPPORTED , HCP_SERIALIZENOTSUPPORTED_MSG },
		{ HCP_DESERIALIZENOTSUPPORTED , HCP_DESERIALIZENOTSUPPORTED_MSG },
		{ HCP_RESIZEFAILED , HCP_RESIZEFAILED_MSG },
		{ HCP_INVALIDPARAMETER_UNKNOWNSTRINGNOTATEND , HCP_INVALIDPARAMETER_UNKNOWNSTRINGNOTATEND_MSG},
		{ HCP_NULLCODECLOAD, HCP_NULLCODECLOAD_MSG },
		{ HCP_CODECLOADFAILED , HCP_CODECLOADFAILED_MSG },
		{ HCP_INVALIDCODECNAME , HCP_INVALIDCODECNAME_MSG },
		{ HCP_CODECALREADYLOADED , HCP_CODECALREADYLOADED_MSG },
		{ HCP_STRINGNOTSUPPORTED , HCP_STRINGNOTSUPPORTED_MSG },
		{ HCP_MISSINGBYTEARRAY_SIZE , HCP_MISSINGBYTEARRAY_SIZE_MSG },
		{ HCP_INVALID_STRINGSIZE , HCP_INVALID_STRINGSIZE_MSG },
		{HCP_NULL,HCP_NULL}
	};

/*
*==============================================================================
*  4.      GLOBAL FUNCTIONS (defined in some implementation file)
*==============================================================================
*/

	/**	Creates a new error structure.
	 *-----------------------------------------------------------------------------
	 * \par	Description:
	 *		Creates a new error structure which is intended to provide detailed\n
	 *		error information which to ease the use of the module and to find the\n
	 *		location where the source occured.
	 *
	 * \param	ErrorCode	[IN] Code which identifies the type of error that occured.
	 * \param	LineNumber [IN]	The exact line where the error occured.
	 * \param	File [IN]	Name of the file where the error occured.
	 * \param	Module [IN]	Name of the module.
	 * \param	Resolver [IN]	Callback function to resolve custom errors, that is\n
	 *							error which is specific to the module and not found\n
	 *							in [hcp_Errors].
	 * \param	pCallbackContext [IN]	Resolver specific callback context that is\n
	 *									passed to [Resolver] when invoked.
	 *
	 *
	 * \return	Returns a error structure populated with the input arguments.
	 *-----------------------------------------------------------------------------
	 */
	extern hcp_tError HCP_CALL hcp_NewError(const hcp_Int16 ErrorCode, const hcp_Int LineNumber, const hcp_tString File, hcp_tString Module, hcp_ErrorSource Resolver, void* pCallbackContext);
	/**	Resolves a error message expressing the need for a specific type.
	*-----------------------------------------------------------------------------
	* \par	Description:
	*		
	*
	* \param	TypeId	[IN] The type that was expected.
	*
	* \return	Returns a error structure populated with the input arguments.
	*-----------------------------------------------------------------------------
	*/
	extern hcp_Int HCP_CALL hcp_ExpectType(const hcp_Int TypeId);

#endif /* Match the re-definition guard */

/*
*==============================================================================
* END OF FILE
*==============================================================================
*/
