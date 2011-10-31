/** @file b-Cap.h
 *
 *  @brief b-CAP client library header
 *
 *  @version	1.0
 *	@date		2009/03/01
 *	@author		DENSO WAVE (y)
 *
 */

#ifndef B_CAP_H
#define B_CAP_H

/**
 * @enum	BCAP_HRESULT
 * @brief	BCAP_HRESULT values
 */
typedef enum BCAP_HRESULT {

	BCAP_S_OK				= 0			,	/*	OK 							*/
	BCAP_E_NOTIMPL			= 0x80004001,	/*	Not implemented	function is called	*/
	BCAP_E_ABORT			= 0x80004004,	/*	Function aborted			*/
	BCAP_E_FAIL				= 0x80004005,	/*	Function failed				*/
	BCAP_E_UNEXPECTED		= 0x8000FFFF,	/*	Fatal Error occurred		*/
	BCAP_E_INVALIDRCVPACKET	= 0x80010001,	/*	Invalid packet is received. */
											/* When this error is occurred, robot controller disconnect from client immediately.*/
											/* Please make sure the packet that you sent. */

	BCAP_E_INVALIDSNDPACKET	= 0x80010002,	/*	Invalid packet is sent		*/
	BCAP_E_INVALIDARGTYPE	= 0x80010003,	/*	Invalid argument type		*/
	BCAP_E_ROBOTISBUSY		= 0x80010004,	/*	Robot is busy (Wait for a while)	*/
	BCAP_E_INVALIDCOMMAND	= 0x80010005,	/*	Invalid command string is received 	*/

	BCAP_E_PACKETSIZEOVER	= 0x80010011,	/*	Received packet size over ( > 16Mbytes) */

	BCAP_E_ARGSIZEOVER		= 0x80010012,	/*	An argument siez over of the received packet. ( > 16Mbytes) */
	BCAP_E_ACCESSDENIED		= 0x80070005,	/*	Access denied				*/
	BCAP_E_HANDLE			= 0x80070006,	/*	Invalid handle				*/
	BCAP_E_OUTOFMEMORY		= 0x8007000E,	/*	Out of memory				*/
	BCAP_E_INVALIDARG		= 0x80070057	/*	Invalid argument			*/

} BCAP_HRESULT;



/* b-CAP Type id */
#define	VT_EMPTY				0			/* (0Byte) */
#define	VT_NULL					1			/* (0Byte) */
#define	VT_ERROR				10			/* (2Byte) */
#define	VT_UI1					17			/* (1Byte) */
#define	VT_I2					2			/* (2Byte) */
#define	VT_UI2					18			/* (2Byte) */
#define	VT_I4					3			/* (4Byte) */
#define	VT_UI4					19			/* (4Byte) */
#define	VT_R4					4			/* (4Byte) */
#define	VT_R8					5			/* (8Byte) */
#define	VT_CY					6			/* (8Byte) */
#define	VT_DATE					7			/* (8Byte) */
#define	VT_BOOL					11			/* (2Byte) */
#define	VT_BSTR					8			/* (ascii string length *2 + 4 Byte) */
											/* Double bytes per character */
#define	VT_VARIANT				12			/* Variant */
#define	VT_ARRAY				0x2000		/* Array */


/* b-CAP Utility macros */
#ifndef SUCCEEDED
	#define SUCCEEDED(Status) ((int)(Status) >= 0)
#endif

#ifndef FAILED
	#define FAILED(Status) ((int)(Status) < 0)
#endif

/* b-CAP standard types */
#ifndef u_char
typedef unsigned char u_char;
#endif
#ifndef u_short
typedef unsigned short u_short;
#endif
#ifndef u_long
typedef unsigned long u_long;
#endif




/* b-CAP Functions */
BCAP_HRESULT	bCap_Open(const char *pIPStr, int iPort, int *piSockFD);
BCAP_HRESULT	bCap_Close(int iSockFD);

BCAP_HRESULT	bCap_ServiceStart(int iSockFd);
BCAP_HRESULT	bCap_ServiceStop(int iSockFd);

/* b-CAP Controller Functions */
BCAP_HRESULT	bCap_ControllerConnect(int iSockFd,char *pStrCtrlname, char *pStrProvName, char *pStrPcName, char *pStrOption, u_long *plhController);
BCAP_HRESULT	bCap_ControllerDisconnect(int iSockFD, u_long lhController);

BCAP_HRESULT	bCap_ControllerGetRobot(int iSockFd, u_long lhController, char *pStrRobotName, char *pStrOption, u_long *lhRobot);
BCAP_HRESULT	bCap_ControllerGetVariable(int iSockFd, u_long lhController, char *pVarName, char *pstrOption, u_long *plhVar);
BCAP_HRESULT	bCap_ControllerGetTask(int iSockFd, u_long lhController, char *pTskName, char *pstrOption, u_long *plhVar);
BCAP_HRESULT	bCap_ControllerExecute(int iSockFd, u_long lhController, char *pStrCommand, char *pStrOption, long *plResult);

/* b-CAP Robot Functions */
BCAP_HRESULT	bCap_RobotRelease(int iSockFd, u_long lhRobot);
BCAP_HRESULT	bCap_RobotGetVariable(int iSockFD, u_long lhRobot, char *pVarName, char *pStrOption, u_long *lhVarCurJnt);
BCAP_HRESULT	bCap_RobotExecute(int iSockFd, u_long lhRobot, char *pStrCommand, u_short iParameterType, int iParameterArrayLen, void * pParameter, void *plResult);
BCAP_HRESULT	bCap_RobotChange(int iSockFd, u_long lhRobot, char *pStrCommand);
BCAP_HRESULT	bCap_RobotMove(int iSockFd, u_long lhRobot, long lComp, char *pStrPose, char *pStrOption);

/* b-CAP Task Functions */
BCAP_HRESULT	bCap_TaskRelease(int iSockFd, u_long lhTask);
BCAP_HRESULT	bCap_TaskGetVariable(int iSockFd, u_long lhTask, char *pVarName, char *pstrOption, u_long *plhVar);
BCAP_HRESULT	bCap_TaskStart(int iSockFd, u_long lhTask, long lMode, char *pStrOption);
BCAP_HRESULT	bCap_TaskStop(int iSockFd, u_long lhTask, long lMode, char *pStrOption);

/* b-CAP Variable Functions */
BCAP_HRESULT 	bCap_VariableRelease(int iSockFd, u_long lhVar);
BCAP_HRESULT	bCap_VariableGetValue(int iSockFd, u_long lhVar, void *pVntValue);
BCAP_HRESULT	bCap_VariablePutValue(int iSockFd, u_long lhVar, u_short iType, u_long lArrays, void  *pVntValue);


#endif

