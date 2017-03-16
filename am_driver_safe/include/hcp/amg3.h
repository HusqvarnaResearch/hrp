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
#ifndef _HCP_AMG3_H_
#define _HCP_AMG3_H_
/*
*==============================================================================
*  2.      INCLUDE FILES
*==============================================================================
*/

#include "hcp_codec.h"
/*
*==============================================================================
*  3.      DECLARATIONS
*  3.1     Global constants
*==============================================================================
*/

#define AMG3_NAME "amg3"
#define AMG3_STX 0x2
#define AMG3_ETX 0x3
#define AMG3_PROTOCOL_SUBCMD "subCmd"
#define AMG3_PROTOCOL_MSGTYPE "msgType"
#define AMG3_ENDIANESS HCP_LITTLEENDIAN
#define AMG3_MESSAGESIZE 0x111	/* (273) */

#define AMG3_CMDRESOFFSET -30
#define AMG3_PARSEOFFSET -20

#define AMG3_PROTOCOL_EXTENDED 0x81
#define AMG3_PROTOCOL_EXTENDEDPSK 0xFE

#define AMG3_CMD_OK 0	/* OK */
#define AMG3_CMD_ERR_UNKNOWN 1	/* Failed, reason unknown. */
#define AMG3_CMD_ERR_VALUE 2	/* Failed, parameter of incorrect value */
#define AMG3_CMD_ERR_RANGE 3	/* Failed, parameter out of range */
#define AMG3_CMD_ERR_NOTAVAIL 4	/* Failed, operation not available for this mode, state or device */
#define AMG3_CMD_ERR_ACCESS 5	/* Failed, operation not available for this access level */
#define AMG3_CMD_ERR_GROUP 6	/* Failed, invalid group */
#define AMG3_CMD_ERR_ID 7	/* Failed, invalid ID */
#define AMG3_CMD_ERR_BUSY 8	/* Failed, device is busy */
#define AMG3_CMD_ERR_INVALID_PIN 9	/* Failed, access denied because of invalid PIN */
#define AMG3_CMD_ERR_MOWER_BLOCKED 10	/* Failed, access denied because of mower blocked */


// Protocol library specific error must start with 
#define AMG3_SUBCMDNOTFOUND			HCP_LIBERR + -1	/* the protocol node did not contain a subCmd key*/
#define AMG3_STRINGTOOLONG			HCP_LIBERR + -2	/* parameter string value was larger than the specified string */
#define AMG3_OUTPUTBUFERTOOSMALL	HCP_LIBERR & -3	/* unable to write string since the output buffer was too small */
#define AMG3_MISSINGMSGTYPE			HCP_LIBERR & -4	/* the protocol did not contain a 'msgType' key/value pair */
#define AMG3_NOPREVIOUSCOMMAND		HCP_LIBERR & -5	/* data was sent but no previous command was exeucted*/
#define AMG3_INVALIDPAYLOADSIZE		HCP_LIBERR & -6	/* invalid number of bytes in payload received */
#define AMG3_TYPENOTSUPPORTED		HCP_LIBERR & -7	/* tried to convert a unsupported value type into a parameter*/
#define AMG3_INVALIDMESSAGETYPE		HCP_LIBERR & -8	/* the message's message type was not supported */
// parse-errors
#define AMG3_PARSE_NOSTX			HCP_LIBERR + AMG3_PARSEOFFSET + -0	/* No STX byte found */
#define AMG3_PARSE_MISSINGDATA		HCP_LIBERR + AMG3_PARSEOFFSET + -1	/* The requested contained too little data */
#define AMG3_PARSE_INVALIDPROTOCOL	HCP_LIBERR + AMG3_PARSEOFFSET + -2	/* the message's protocol type is acknowledged but not supported */
#define AMG3_PARSE_INVALIDCRC		HCP_LIBERR + AMG3_PARSEOFFSET + -3	/* Invalid CRC */
#define AMG3_PARSE_NOETX			HCP_LIBERR + AMG3_PARSEOFFSET + -4	/* No ETX byte found */
#define AMg3_PARSE_INVALIDCMDRES	HCP_LIBERR + AMG3_PARSEOFFSET + -5	/* Unknown command-result*/
// command result erorrs
#define AMG3_PARSE_UNKNOWN			HCP_LIBERR + AMG3_CMDRESOFFSET + -0
#define AMG3_PARSE_VALUE			HCP_LIBERR + AMG3_CMDRESOFFSET + -1
#define AMG3_PARSE_RANGE			HCP_LIBERR + AMG3_CMDRESOFFSET + -2
#define AMG3_PARSE_NOTAVAIL			HCP_LIBERR + AMG3_CMDRESOFFSET + -3
#define AMG3_PARSE_ACCESS			HCP_LIBERR + AMG3_CMDRESOFFSET + -4
#define AMG3_PARSE_GROUP			HCP_LIBERR + AMG3_CMDRESOFFSET + -5
#define AMG3_PARSE_ID				HCP_LIBERR + AMG3_CMDRESOFFSET + -6
#define AMG3_PARSE_BUSY				HCP_LIBERR + AMG3_CMDRESOFFSET + -7
#define AMG3_PARSE_INVALID_PIN		HCP_LIBERR + AMG3_CMDRESOFFSET + -8
#define AMG3_PARSE_MOWER_BLOCKED	HCP_LIBERR + AMG3_CMDRESOFFSET + -9

#define AMG3_SUBCMDNOTFOUND_MSG "The protocol did not contain a subCmd-key."
#define AMG3_STRINGTOOLONG_MSG "Parameter string value too large."
#define AMG3_OUTPUTBUFERTOOSMALL_MSG "Unable to write string, the output buffer was too small."
#define AMG3_MISSINGMSGTYPE_MSG "The protocol did not contain a message-type key."
#define AMG3_NOPREVIOUSCOMMAND_MSG "Unable to process response without a initial request."
#define AMG3_INVALIDPAYLOADSIZE_MSG "The payload did not contain the same number of bytes as specified in the commands out-parameters."
#define AMG3_TYPENOTSUPPORTED_MSG "The AMG3 protocol does not support the specified parameter type."
#define AMG3_INVALIDMESSAGETYPE_MSG "The recieved message-type was not supported."
// parse errors
#define AMG3_PARSE_NOSTX_MSG "No STX byte found."
#define AMG3_PARSE_MISSINGDATA_MSG "The request did not contain enough data."
#define AMG3_PARSE_INVALIDPROTOCOL_MSG "The message's protocol was acknowledged but not supported."
#define AMG3_PARSE_INVALIDCRC_MSG "Message contained a invalid CRC-value."
#define AMG3_PARSE_NOETX_MSG "No ETX byte found."
#define AMg3_PARSE_INVALIDCMDRES_MSG "Unknown command result."
// command result error(s)
#define AMG3_PARSE_UNKNOWN_MSG "Failed, reason unknown."
#define AMG3_PARSE_VALUE_MSG "Failed, parameter of incorrect value."
#define AMG_PARSE_RANGE_MSG "Failed, parameter out of range"
#define AMG3_PARSE_NOTAVAIL_MSG "Failed, operation not available for this mode, state or device."
#define AMG3_PARSE_ACCESS_MSG "Failed, operation not available for this access level"
#define AMG3_PARSE_GROUP_MSG "Failed, invalid group"
#define AMG3_PARSE_ID_MSG "Failed, invalid ID"
#define AMG3_PARSE_BUSY_MSG "Failed, device is busy"
#define AMG3_PARSE_INVALID_PIN_MSG "Failed, access denied because of invalid PIN"
#define AMG3_PARSE_MOWER_BLOCKED_MSG "Failed, access denied because of mower blocked"

static const hcp_tErrorMessage amg3_Errors[] = {
	{ AMG3_SUBCMDNOTFOUND , AMG3_SUBCMDNOTFOUND_MSG },
	{ AMG3_STRINGTOOLONG , AMG3_STRINGTOOLONG_MSG },
	{ AMG3_OUTPUTBUFERTOOSMALL , AMG3_OUTPUTBUFERTOOSMALL_MSG },
	{ AMG3_MISSINGMSGTYPE , AMG3_MISSINGMSGTYPE_MSG },
	{ AMG3_NOPREVIOUSCOMMAND , AMG3_NOPREVIOUSCOMMAND_MSG },
	{ AMG3_INVALIDPAYLOADSIZE , AMG3_INVALIDPAYLOADSIZE_MSG },
	{ AMG3_TYPENOTSUPPORTED , AMG3_TYPENOTSUPPORTED_MSG },
	{ AMG3_INVALIDMESSAGETYPE , AMG3_INVALIDMESSAGETYPE_MSG },
	{ AMG3_PARSE_INVALIDPROTOCOL , AMG3_PARSE_INVALIDPROTOCOL_MSG },
	// parse errors
	{ AMG3_PARSE_NOSTX , AMG3_PARSE_NOSTX_MSG },
	{ AMG3_PARSE_MISSINGDATA , AMG3_PARSE_MISSINGDATA_MSG },
	{ AMG3_PARSE_INVALIDCRC, AMG3_PARSE_INVALIDCRC_MSG },
	{ AMG3_PARSE_NOETX, AMG3_PARSE_NOETX_MSG },
	{ AMg3_PARSE_INVALIDCMDRES , AMg3_PARSE_INVALIDCMDRES_MSG },
	// command result error(s)
	{ AMG3_PARSE_UNKNOWN , AMG3_PARSE_UNKNOWN_MSG },
	{ AMG3_PARSE_VALUE, AMG3_PARSE_VALUE_MSG },
	{ AMG3_PARSE_RANGE, AMG_PARSE_RANGE_MSG },
	{ AMG3_PARSE_NOTAVAIL, AMG3_PARSE_NOTAVAIL_MSG },
	{ AMG3_PARSE_ACCESS , AMG3_PARSE_ACCESS_MSG },
	{ AMG3_PARSE_GROUP, AMG3_PARSE_GROUP_MSG },
	{ AMG3_PARSE_ID , AMG3_PARSE_ID_MSG },
	{ AMG3_PARSE_BUSY , AMG3_PARSE_BUSY_MSG },
	{ AMG3_PARSE_INVALID_PIN, AMG3_PARSE_INVALID_PIN_MSG },
	{ AMG3_PARSE_MOWER_BLOCKED , AMG3_PARSE_MOWER_BLOCKED_MSG },
	{HCP_NULL, HCP_NULL}
};

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

// Copied from "AMV3-0004 IRS for Robotic Mower G3.doc
static const hcp_Uint8 CRC8_TABLE[256] = {
	0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
	157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
	35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
	190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
	70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
	219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
	101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
	248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
	140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
	17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
	175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
	50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
	202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
	87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
	233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
	116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

static const hcp_Uint8 InitCrc = 0;

/*
*==============================================================================
*  4.      GLOBAL FUNCTIONS (defined in some implementation file)
*==============================================================================
*/

HCP_API hcp_tCodecLibrary* HCP_CALL hcp_GetLibrary(void);

#endif /* Match the re-definition guard */

/*
*==============================================================================
* END OF FILE
*==============================================================================
*/