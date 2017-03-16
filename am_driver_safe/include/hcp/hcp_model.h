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
#ifndef _HCP_MODEL_H_
#define _HCP_MODEL_H_
/*
*==============================================================================
*  2.      INCLUDE FILES
*==============================================================================
*/

#include "hcp_types.h"
#include "hcp_codec.h"

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

/**	Vector with commands that will be populated during runtime
*/
HCP_VECTOR(hcp_tCommandTemplate, hcp_tCommandTemplateSet, HCP_MAXSIZE_COMMANDS);


typedef struct {
	hcp_tString schema;
	hcp_tString version;
	hcp_tString created;
	hcp_tString protocol;
} hcp_tModelHeader;

typedef struct hcp_tModel {
	hcp_tCommandTemplateSet commands;
	hcp_tModelHeader header;	/* template which identifies the node in a collection */
	hcp_tProtocol protocol;
	void* cache;
	hcp_Uint16 id;	/* id which identifies a template within a collection*/
} hcp_tModel;

typedef struct {
	hcp_tVector header;
	hcp_tModel fixed[HCP_MAXSIZE_TIFTEMPLATES];
	hcp_Uint16 nextId;
} hcp_tModelSet;

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



#endif /* Match the re-definition guard */

/*
*==============================================================================
* END OF FILE
*==============================================================================
*/