/** @file b-Cap.c
 *
 *  @brief b-CAP client library
 *
 *  @version	1.0
 *	@date		2009/03/01
 *	@author		DENSO WAVE (y)
 *
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef WIN32
	#include <winsock2.h>

#else

	#include <unistd.h>

	#include <sys/types.h>
	#include <sys/socket.h>
	#include <arpa/inet.h>
	#include <netinet/in.h>
	#include <netdb.h>
#endif


#include "b-Cap/b-Cap.hpp"


/* ENDIAN switching */
#if !defined(__LITTLE_ENDIAN__)
#if !defined(__BIG_ENDIAN__)

#if __BYTE_ORDER == __LITTLE_ENDIAN
#define __LITTLE_ENDIAN__
#elif __BYTE_ORDER == __BIG_ENDIAN
#define __BIG_ENDIAN__
#endif

#endif
#endif

/* DEBUG */
//#define	DEBUG					1

#ifdef DEBUG
#define DebugPrint( message )   fprintf( stderr, message )
#define DebugPrint2( f, a, b )  fprintf( stderr, f, a, b )
#else
#define DebugPrint( message )
#define DebugPrint2( f, a, b )
#endif


/* Constant values */

/* length of temporary string buffer */
#define	LOCALBUFFER_SZ			256
/* length of temporary recieve buffer (must be >= 16bytes) */
#define	LOCALRECBUFFER_SZ			256


/* b-CAP packet constant */
#define	BCAP_SOH				0x01		/* size of packet header(SOH) */
#define	BCAP_EOT				0x04		/* size of packet terminater(EOT) */
#define	BCAP_SIZE_SOH			1			/* size of header(SOH)   */
#define	BCAP_SIZE_EOT			1			/* size of terminater(EOT)  */
#define	BCAP_SIZE_LEN			4			/* size of message size */
#define	BCAP_SIZE_SERIAL		2			/* size of serial number */
#define	BCAP_SIZE_RESERVE		2			/* size of reserved */
#define	BCAP_SIZE_FUNCID		4			/* size of FunctionID */
#define	BCAP_SIZE_ARGNUM		2			/* size of Args */

											/* b-CAP Packet header size */
#define	BCAP_SIZE_BASE			(BCAP_SIZE_SOH + BCAP_SIZE_EOT + \
								 BCAP_SIZE_LEN + BCAP_SIZE_SERIAL + \
								 BCAP_SIZE_RESERVE + BCAP_SIZE_FUNCID + \
								 BCAP_SIZE_ARGNUM)


/* b-CAP argument constant */
#define	BCAP_SIZE_ARGLEN		4			/* size of length  */
#define	BCAP_SIZE_ARGTYPE		2			/* size of type */
#define	BCAP_SIZE_ARGARRAYS		4			/* size of arrays */
#define	BCAP_SIZE_ARGBASE		(BCAP_SIZE_ARGLEN+BCAP_SIZE_ARGTYPE+BCAP_SIZE_ARGARRAYS)
											/* b-CAP Arg header size  */
#define	BCAP_SIZE_ARGSTRLEN		4			/* size of string length */


#define BCAP_MAX_PACKET_SIZE	0x1000000	/* max packet size (bytes) */
#define BCAP_MAX_ARG_SIZE		0x1000000	/* max arg size (bytes) */


/* b-CAP function IDs */
#define	BCAP_FUNC_Service_Start	1
#define	BCAP_FUNC_Service_Stop	2
#define	BCAP_FUNC_Controller_Connect	3
#define	BCAP_FUNC_Controller_Disconnect	4
#define	BCAP_FUNC_Controller_GetRobot	7
#define	BCAP_FUNC_Controller_GetTask	8
#define	BCAP_FUNC_Controller_GetVariable	9
#define	BCAP_FUNC_Controller_Execute	17

#define	BCAP_FUNC_Robot_GetVariable	62
#define	BCAP_FUNC_Robot_Execute	64
#define	BCAP_FUNC_Robot_Change	66
#define	BCAP_FUNC_Robot_Move	72
#define BCAP_FUNC_Robot_Speed	74
#define	BCAP_FUNC_Robot_Release	84

#define	BCAP_FUNC_Task_GetVariable	85
#define	BCAP_FUNC_Task_Start	88
#define	BCAP_FUNC_Task_Stop	89
#define	BCAP_FUNC_Task_Release	99

#define	BCAP_FUNC_Variable_PutValue	102
#define	BCAP_FUNC_Variable_GetValue	101
#define	BCAP_FUNC_Variable_PutValue	102
#define BCAP_FUNC_Variable_Release 111


/* b-CAP Argment structure */
/**
 * @struct	BCAP_ARG
 * @brief	BCAP_ARG
 */
typedef struct BCAP_ARG {
	u_long	lLength;

	u_short iType;
	u_long	lArrays;
	void	*data;

	/* long	lStrlen; */

	struct BCAP_ARG	*pNextArg;						/* pointer to the next argument  */
} BCAP_ARG;

/* b-CAP Packet structure */
/**
 * @struct	BCAP_PACKET
 * @brief	BCAP_PACKET
 */
typedef struct BCAP_PACKET {

	u_long	lMsgLength;

	u_short iSerialNo;
	u_short iReserved;

	u_long	lFuncID;

	u_short iArgs;

	struct BCAP_ARG	*pArg;							/* pointer to the first argment */
} BCAP_PACKET;


/* module function prototypes */
static BCAP_HRESULT		Packet_Send(int iSd, BCAP_PACKET *pPacket);

/* packet class */
static BCAP_PACKET		*Packet_Create(u_long lFuncID);
static void				Packet_Release( BCAP_PACKET *pPacket);
static BCAP_HRESULT		Packet_Serialize(BCAP_PACKET *pPacket, void *pBinData );		/* struct ---> bin */
static BCAP_HRESULT		Packet_Deserialize(void *pBinData, BCAP_PACKET *pPacket );		/* bin ---> struct  */
static BCAP_HRESULT		Packet_AddArg( BCAP_PACKET *pPacket, BCAP_ARG *pNewArg);

/* argument class */
static BCAP_ARG			*Arg_Create( u_short iType, u_long lArrays, u_long lLength, void *data);
static void				Arg_Release( BCAP_ARG *pArg);	/* free allocated argument */
/* static BCAP_HRESULT	Arg_Serialize(void *pBinData);	*/									/* bin <--- struct  */
/* static BCAP_HRESULT	Arg_Deserialize(BCAP_ARG *pArg, void *pBinData);	*/				/* struct <--- bin  */
/* static u_long 		Arg_CopyValue(void *pDst, BCAP_ARG *pSrcArg); */

/* module socket utility functions */
static BCAP_HRESULT		bCapSendAndRec(int iSockFd, BCAP_PACKET *pSndPacket, BCAP_PACKET *pRecPacket);
static BCAP_HRESULT		sendBinary(int iSockFd, u_char *pBuff, u_long lSize);
static u_char 			*receivePacket(int iSockFd);

/* module utility functions */
static u_long			sizeOfVarType(u_short iType);
static u_long			copyValue(void *pDst, void *pVntValue, u_long lLength);
static u_long			copyToBSTR(void *pbDstPtr, void *pbSrcPtr);
static u_long			copyFromBSTR(void *pDstAsciiPtr, void *pSrcBstrPtr);
static void				*bMalloc(size_t size);
static void				bFree(void *pPtr);

/* module variables */
static u_short m_lSerialNo = 0;						/* packet serial number: cyclic from 0x0000 to 0xFFFF */



/*--------------------------------------------------------------------
			b-Cap library public functions
 --------------------------------------------------------------------*/


/**	Init and Open socket
 *
 * Init and Open socket
 *
 *	@param	iSockFd	:	[in]  Socket descriptor
 *	@retval	BCAP_HRESULT
 *
 */
BCAP_HRESULT	bCap_Open(const char *pIPStr, int iPort, int *iSockFD) {

	struct sockaddr_in	serverAddr;			/* server's socket address */
	int					sockAddrSize;		/* size of socket address structure */

	BCAP_HRESULT hr = BCAP_E_FAIL;

#ifdef WIN32
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2,0), &wsaData);
#endif

	sockAddrSize = sizeof( struct sockaddr_in);
	memset( (char *)&serverAddr, 0, sockAddrSize);
	serverAddr.sin_family = AF_INET;

	serverAddr.sin_port = htons( iPort);					/* SERVER_PORT_NUM */

#ifdef WIN32
	 serverAddr.sin_addr.S_un.S_addr = inet_addr(pIPStr);	/* SERVER_IP_ADDRESS */
#else
	inet_aton(pIPStr, &(serverAddr.sin_addr));
#endif


	/* socket  */
	if( (*iSockFD = socket( AF_INET, SOCK_STREAM, 0)) == -1) {
		perror( "Fail.. Function:socket() in bCap_Open()");
		hr = BCAP_E_UNEXPECTED;
	}
	else{
		/* connect to server */
		int iResult;
		iResult = connect(*iSockFD, (struct sockaddr *)&serverAddr, sizeof(serverAddr));

		if (iResult == 0){
			hr = BCAP_S_OK;
		}
		else{
			hr = BCAP_E_UNEXPECTED;
		}

	}

	return hr;
}


/**	Close socket
 *
 * Close socket
 *
 *	@param	iSockFd	:	[in]  Socket descriptor
 *	@retval	BCAP_HRESULT
 *
 */
BCAP_HRESULT	bCap_Close( int iSockFd){

	BCAP_HRESULT hr = BCAP_E_FAIL;

#ifdef WIN32
	/* end of TCP session */
	closesocket(iSockFd);
	if (WSACleanup() == 0){
		hr = BCAP_S_OK;
	}
#else
	close(iSockFd);
	hr = BCAP_S_OK;
#endif

	return hr;
}


/**	Start b-Cap service
 *
 * Start b-Cap servic
 *
 *	@param	iSockFd	:	[in]  Socket descriptor
 *	@retval	BCAP_HRESULT
 *
 */
BCAP_HRESULT bCap_ServiceStart(int iSockFd){
	BCAP_PACKET		*pPacket;
	BCAP_PACKET		*pRecPacket;

	BCAP_HRESULT hr = BCAP_E_FAIL;

	pPacket = Packet_Create(BCAP_FUNC_Service_Start);		/* Service_Start */
	if (pPacket != NULL){

		{
			pRecPacket = Packet_Create(BCAP_S_OK);					/* storing a new packet from RC    */
			if (pRecPacket != NULL){
				hr = bCapSendAndRec(iSockFd, pPacket, pRecPacket);

			}
			Packet_Release(pRecPacket);
		}
		Packet_Release(pPacket);
	}

	return hr;

}

/**	Stop b-Cap service
 *
 * Stop b-Cap service
 *

 *	@param	iSockFd	:	[in]  Socket descriptor
 *	@retval	BCAP_HRESULT
 *
 */
BCAP_HRESULT bCap_ServiceStop(int iSockFd){
	BCAP_PACKET		*pPacket;
	BCAP_PACKET		*pRecPacket;

	BCAP_HRESULT hr = BCAP_E_FAIL;

	pPacket = Packet_Create(BCAP_FUNC_Service_Stop);		/* alloc new packet:Service_Stop */
	if (pPacket != NULL){

		{
			pRecPacket = Packet_Create(BCAP_S_OK);					/*  alloc new packet storing a packet from RC    */
			if (pRecPacket != NULL){
				hr = bCapSendAndRec(iSockFd, pPacket, pRecPacket);

			}
			Packet_Release(pRecPacket);
		}
		Packet_Release(pPacket);
	}

	return hr;

}



/**	Controller_Connect
 *
 * Controller_Connect
 *
 *	@param	iSockFd			:	[in]  Socket descriptor
 *	@param	pStrCtrlname	:	[in]  CtrlName in AsciiZ
 *	@param	pStrProvName	:	[in]  ProvName in AsciiZ
 *	@param	pStrPcName		:	[in]  PCName in AsciiZ
 *	@param	pStrOption		:	[in]  Option string in AsciiZ
 *	@param	plhController	:	[out]  handle of the controller that returned from the robot controller.
 *	@retval	BCAP_HRESULT
 *
 */
BCAP_HRESULT bCap_ControllerConnect(	int iSockFd,
										char *pStrCtrlname,
										char *pStrProvName,
										char *pStrPcName,
										char *pStrOption,
										u_long *plhController){
	BCAP_PACKET		*pPacket;
	BCAP_PACKET		*pRecPacket;
	BCAP_ARG		*pArg;

	BCAP_HRESULT hr = BCAP_E_FAIL;

	pPacket = Packet_Create(BCAP_FUNC_Controller_Connect);		/* Controller_Connect */
	if (pPacket != NULL){

		u_char buff[LOCALBUFFER_SZ];
		u_long lLen;

		{
			lLen = copyToBSTR(buff,pStrCtrlname);
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pPacket, pArg);
			}
		}
		{
			lLen = copyToBSTR(buff,pStrProvName);
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pPacket, pArg);
			}
		}
		{
			lLen = copyToBSTR(buff,pStrPcName);
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pPacket, pArg);
			}
		}
		{
			lLen = copyToBSTR(buff,pStrOption);
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pPacket, pArg);
			}
		}

		{
			pRecPacket = Packet_Create(BCAP_S_OK);
			if (pRecPacket != NULL){
				hr = bCapSendAndRec(iSockFd, pPacket, pRecPacket);
				if SUCCEEDED(hr){
					copyValue(plhController, pRecPacket->pArg->data, 4);
				}

			}
			Packet_Release(pRecPacket);
		}
		Packet_Release(pPacket);
	}

	return hr;

}


/**	Disconnect b-Cap
 *
 * Controller_Disconnect
 *
 *	@param	iSockFd	:	[in]  Socket descriptor
 *	@param	lhController		:	[in]  controller handle to disconnect
 *	@retval	BCAP_HRESULT
 *
 */
BCAP_HRESULT bCap_ControllerDisconnect(int iSockFd, u_long lhController){
	BCAP_PACKET		*pPacket;
	BCAP_PACKET		*pRecPacket;

	BCAP_HRESULT hr = BCAP_E_FAIL;

	pPacket = Packet_Create(BCAP_FUNC_Controller_Disconnect);		/* Controller_Disconnect */
	if (pPacket != NULL){


		{
			pRecPacket = Packet_Create(BCAP_S_OK);					/* storing a new packet from RC    */
			if (pRecPacket != NULL){
				hr = bCapSendAndRec(iSockFd, pPacket, pRecPacket);

			}
			Packet_Release(pRecPacket);
		}
		Packet_Release(pPacket);
	}

	return hr;

}


/**	Controller_GetRobot
 *
 * Controller_GetRobot
 *
 *	@param	iSockFd	:	[in]  Socket descriptor
 *	@param	lhController		:	[in]  controller handle
 *	@param	pStrRobotName		:	[in]  Robot name string in AsciiZ
 *	@param	pStrOption			:	[in]  Option string in AsciiZ
 *	@param	lhRobot				:	[out]  robot handle
 *	@retval	BCAP_HRESULT
 *
 */
BCAP_HRESULT bCap_ControllerGetRobot(int iSockFd, u_long lhController, char *pStrRobotName, char *pStrOption, u_long *plhRobot){
	BCAP_PACKET		*pSndPacket;
	BCAP_PACKET		*pRecPacket;
	BCAP_ARG		*pArg;

	u_char buff[LOCALBUFFER_SZ];
	u_long lLen;

	BCAP_HRESULT hr = BCAP_E_FAIL;

	pSndPacket = Packet_Create(BCAP_FUNC_Controller_GetRobot);		/* BCAP_FUNC_Controller_GetRobot */
	if (pSndPacket != NULL){

		{
			pArg = Arg_Create( VT_I4, 1, 4, &lhController);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}
		{
			lLen = copyToBSTR(buff,pStrRobotName);
			pArg = Arg_Create( VT_BSTR, 1, lLen, buff);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}
		{
			lLen = copyToBSTR(buff,pStrOption);
			pArg = Arg_Create( VT_BSTR, 1, lLen, buff);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}
		{
			pRecPacket = Packet_Create(BCAP_S_OK);
			if (pRecPacket != NULL){
				hr = bCapSendAndRec(iSockFd, pSndPacket, pRecPacket);
				if SUCCEEDED(hr){
					copyValue(plhRobot, pRecPacket->pArg->data, 4);
				}
				else{
					/* NO Argument */
					hr = BCAP_E_FAIL;
				}

			}
			Packet_Release(pRecPacket);
		}
		Packet_Release(pSndPacket);
	}

	return hr;
}


/**	Controller_GetVariable
 *
 * Controller_GetVariable
 *
 *	@param	iSockFd			:	[in]  Socket descriptor
 *	@param	lhController	:	[in]  controller handle
 *	@param	pVarName		:	[in]  Variable name string in AsciiZ
 *	@param	pstrOption		:	[in]  Option string in AsciiZ
 *	@param	plhVar			:	[out]  variable handle
 *	@retval	BCAP_HRESULT
 *
 */

BCAP_HRESULT bCap_ControllerGetVariable(int iSockFd, u_long lhController, char *pVarName, char *pstrOption, u_long *plhVar){
	BCAP_PACKET		*pSndPacket;
	BCAP_PACKET		*pRecPacket;
	BCAP_ARG		*pArg;

	BCAP_HRESULT hr = BCAP_E_FAIL;

	pSndPacket = Packet_Create(BCAP_FUNC_Controller_GetVariable);		/* BCAP_FUNC_Controller_GetVariable */
	if (pSndPacket != NULL){

		u_char buff[LOCALBUFFER_SZ];
		u_long lLen;

		{
			pArg = Arg_Create( VT_I4, 1, 4, &lhController);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			lLen = copyToBSTR(buff,pVarName);
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			lLen = copyToBSTR(buff,pstrOption);
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			pRecPacket = Packet_Create(BCAP_S_OK);
			if (pRecPacket != NULL){
				hr = bCapSendAndRec(iSockFd, pSndPacket, pRecPacket);
				if SUCCEEDED(hr){
					if (pRecPacket->iArgs >= 1){
						copyValue(plhVar, pRecPacket->pArg->data, 4);
					}
					else{
						/* NO Argument */
						hr = BCAP_E_FAIL;
					}
				}

			}
			Packet_Release(pRecPacket);
		}
		Packet_Release(pSndPacket);
	}

	return hr;
}

/**	Controller_GetTask
 *
 * Controller_GetTask
 *
 *	@param	iSockFd	:	[in]  Socket descriptor
 *	@param	lhController		:	[in]  controller handle
 *	@param	pTskName		:	[in]  task name string in AsciiZ
 *	@param	pstrOption		:	[in]  option string in AsciiZ
 *	@param	plhVar		:	[out]  task handle
 *	@retval	BCAP_HRESULT
 *
 */
BCAP_HRESULT bCap_ControllerGetTask(int iSockFd, u_long lhController, char *pTskName, char *pstrOption, u_long *plhVar){
	BCAP_PACKET		*pSndPacket;
	BCAP_PACKET		*pRecPacket;
	BCAP_ARG		*pArg;

	BCAP_HRESULT hr = BCAP_E_FAIL;

	pSndPacket = Packet_Create(BCAP_FUNC_Controller_GetTask);		/* BCAP_FUNC_Controller_GetTask */
	if (pSndPacket != NULL){

		u_char buff[LOCALBUFFER_SZ];
		u_long lLen;

		{
			pArg = Arg_Create( VT_I4, 1, 4, &lhController);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			lLen = copyToBSTR(buff,pTskName);
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			lLen = copyToBSTR(buff, pstrOption);
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			pRecPacket = Packet_Create(BCAP_S_OK);
			if (pRecPacket != NULL){
				hr = bCapSendAndRec(iSockFd, pSndPacket, pRecPacket);
				if SUCCEEDED(hr){
					if (pRecPacket->iArgs >= 1){
						copyValue(plhVar, pRecPacket->pArg->data, 4);
					}
					else{
						/* NO Argument */
						hr = BCAP_E_FAIL;
					}
				}

			}
			Packet_Release(pRecPacket);
		}
		Packet_Release(pSndPacket);
	}

	return hr;
}


/**	Controller_Execute
 *
 * Controller_Execute
 *
 *	@param	iSockFd	:	[in]  Socket descriptor
 *	@param	lhController		:	[in]  controller handle
 *	@param	pStrCommand		:	[in]  command string in AsciiZ
 *	@param	pstrOption		:	[in]  command string in AsciiZ
 *	@param	plhVar		:	[out]  result value
 *	@retval	BCAP_HRESULT
 *
 */
BCAP_HRESULT bCap_ControllerExecute(int iSockFd, u_long lhController, char *pStrCommand, char *pStrOption, long *plResult){
	BCAP_PACKET		*pSndPacket;
	BCAP_PACKET		*pRecPacket;
	BCAP_ARG		*pArg;

	BCAP_HRESULT hr = BCAP_E_FAIL;

	pSndPacket = Packet_Create(BCAP_FUNC_Controller_Execute);		/* BCAP_FUNC_Controller_Execute */
	if (pSndPacket != NULL){

		u_char buff[LOCALBUFFER_SZ];
		u_long lLen;

		{
			pArg = Arg_Create( VT_I4, 1, 4, &lhController);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			lLen = copyToBSTR(buff,pStrCommand);
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			lLen = copyToBSTR(buff, pStrOption);
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			pRecPacket = Packet_Create(BCAP_S_OK);
			if (pRecPacket != NULL){
				hr = bCapSendAndRec(iSockFd, pSndPacket, pRecPacket);
				if SUCCEEDED(hr){
					if (pRecPacket->iArgs >= 1){
						copyValue(plResult, pRecPacket->pArg->data, 4);
					}
					else{
						/* NO Argument */
						hr = BCAP_E_FAIL;
					}
				}

			}
			Packet_Release(pRecPacket);
		}
		Packet_Release(pSndPacket);
	}

	return hr;
}



/**	Robot_Release
 *
 * Robot_Release
 *
 *	@param	iSockFd	:	[in]  Socket descriptor
 *	@param	lhRobot		:	[in]  robot handle
 *	@retval	BCAP_HRESULT
 *
 */
BCAP_HRESULT bCap_RobotRelease(int iSockFd, u_long lhRobot){
	BCAP_PACKET		*pSndPacket;
	BCAP_PACKET		*pRecPacket;
	BCAP_ARG		*pArg;


	BCAP_HRESULT hr = BCAP_E_FAIL;

	pSndPacket = Packet_Create(BCAP_FUNC_Robot_Release);		/* FuncID:RobotRelease */
	if (pSndPacket != NULL){

		{
			pArg = Arg_Create( VT_I4, 1, 4, &lhRobot);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);				/* Add 1st argument */
			}
		}

		{
			pRecPacket = Packet_Create(BCAP_S_OK);				/* Alloc for storing received packet */
			if (pRecPacket != NULL){
				hr = bCapSendAndRec(iSockFd, pSndPacket, pRecPacket);

			}
			Packet_Release(pRecPacket);							/* Release recieved packet */
		}
		Packet_Release(pSndPacket);
	}

	return hr;
}



/**	Robot_GetVariable
 *
 * Robot_GetVariable
 *
 *	@param	iSockFd			:	[in]  Socket descriptor
 *	@param	lhRobot			:	[in]  robot handle
 *	@param	pVarName		:	[in]  Variable name string in AsciiZ
 *	@param	pstrOption		:	[in]  Option string in AsciiZ
 *	@param	plhVar			:	[out]  result value = variable handle
 *	@retval	BCAP_HRESULT
 *
 */
BCAP_HRESULT bCap_RobotGetVariable(int iSockFd, u_long lhRobot, char *pVarName, char *pstrOption, u_long *plhVar){
	BCAP_PACKET		*pSndPacket;
	BCAP_PACKET		*pRecPacket;
	BCAP_ARG		*pArg;

	BCAP_HRESULT hr = BCAP_E_FAIL;

	pSndPacket = Packet_Create(BCAP_FUNC_Robot_GetVariable);		/* BCAP_FUNC_Robot_GetVariable */
	if (pSndPacket != NULL){

		u_char buff[LOCALBUFFER_SZ];
		u_long lLen;

		{
			pArg = Arg_Create( VT_I4, 1, 4, &lhRobot);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			lLen = copyToBSTR(buff,pVarName);
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			lLen = copyToBSTR(buff, pstrOption);
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			pRecPacket = Packet_Create(BCAP_S_OK);
			if (pRecPacket != NULL){
				hr = bCapSendAndRec(iSockFd, pSndPacket, pRecPacket);
				if SUCCEEDED(hr){
					if (pRecPacket->iArgs >= 1){
						copyValue(plhVar, pRecPacket->pArg->data, 4);
					}
					else{

					}
				}
			}
			Packet_Release(pRecPacket);
		}
		Packet_Release(pSndPacket);
	}

	return hr;
}

/**	Robot_Change
 *
 * Robot_Change
 *
 *	@param	iSockFd	:	[in]  Socket descriptor
 *	@param	lhRobot		:	[in]  robot handle
 *	@param	pStrCommand		:	[in]  command string in AsciiZ
 *	@param	pstrOption		:	[in]  option string in AsciiZ
 *	@param	plhVar		:	[out]  result value
 *	@retval	BCAP_HRESULT
 *
 */
/**	Robot_Execute
 *
 * Robot_Execute
 *
 *	@param	iSockFd			:	[in]  Socket descriptor
 *	@param	lhRobot			:	[in]  robot handle
 *	@param	pStrCommand		:	[in]  command string in AsciiZ
 *	@param	pstrOption		:	[in]  option string in AsciiZ
 *	@param	plhVar			:	[out]  result value
 *	@retval	BCAP_HRESULT
 *
 */
BCAP_HRESULT bCap_RobotExecute(int iSockFd, u_long lhRobot, char *pStrCommand, u_short iParameterType, int iParameterArrayLen, void * pParameter, void *plResult) {
	BCAP_PACKET		*pSndPacket;
	BCAP_PACKET		*pRecPacket;
	BCAP_ARG		*pArg;

	BCAP_HRESULT hr = BCAP_E_FAIL;

	pSndPacket = Packet_Create(BCAP_FUNC_Robot_Execute);		/* BCAP_FUNC_Robot_Execute */
	if (pSndPacket != NULL){

		u_char buff[LOCALBUFFER_SZ];
		u_long lLen;

		{
			pArg = Arg_Create( VT_I4, 1, 4, &lhRobot);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			lLen = copyToBSTR(buff,pStrCommand);
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			if (iParameterType == VT_BSTR)	{
				lLen = copyToBSTR(buff, pParameter);
				pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			}
			else {
				lLen = sizeOfVarType(iParameterType);
				if (iParameterArrayLen) {
					iParameterType = iParameterType | VT_ARRAY;
					lLen *= iParameterArrayLen;
				}


				pArg = Arg_Create( iParameterType, 1, lLen,pParameter);
			}

			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			pRecPacket = Packet_Create(BCAP_S_OK);
			if (pRecPacket != NULL){

				hr = bCapSendAndRec(iSockFd, pSndPacket, pRecPacket);
				if SUCCEEDED(hr){

			/*		if (pRecPacket->iArgs >= 1){

						copyValue(plResult, pRecPacket->pArg->data, 4);

					}
					else{
						hr = BCAP_E_FAIL;
					}*/
				}

			}
			Packet_Release(pRecPacket);
		}
		Packet_Release(pSndPacket);
	}

	return hr;
}


/**	Robot_Change
 *
 * Robot_Change
 *
 *	@param	iSockFd	:	[in]  Socket descriptor
 *	@param	lhRobot		:	[in]  robot handle
 *	@param	pStrCommand		:	[in]  command string in AsciiZ
 *	@param	pstrOption		:	[in]  option string in AsciiZ
 *	@param	plhVar		:	[out]  result value
 *	@retval	BCAP_HRESULT
 *
 */
BCAP_HRESULT bCap_RobotChange(int iSockFd, u_long lhRobot, char *pStrCommand){
	BCAP_PACKET		*pSndPacket;
	BCAP_PACKET		*pRecPacket;
	BCAP_ARG		*pArg;

	BCAP_HRESULT hr = BCAP_E_FAIL;

	pSndPacket = Packet_Create(BCAP_FUNC_Robot_Change);		/* BCAP_FUNC_Robot_Change */
	if (pSndPacket != NULL){

		u_char buff[LOCALBUFFER_SZ];
		u_long lLen;

		{
			pArg = Arg_Create( VT_I4, 1, 4, &lhRobot);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			lLen = copyToBSTR(buff,pStrCommand);
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			pRecPacket = Packet_Create(BCAP_S_OK);
			if (pRecPacket != NULL){
				hr = bCapSendAndRec(iSockFd, pSndPacket, pRecPacket);
				if SUCCEEDED(hr){
					;
				}

			}
			Packet_Release(pRecPacket);
		}
		Packet_Release(pSndPacket);
	}

	return hr;
}


/**	Robot_Move
 *
 * Robot_Move
 *
 *	@param	iSockFd	:	[in]  Socket descriptor
 *	@param	lhRobot		:	[in]  robot handle
 *	@param	lComp		:	[in]  completion parameter
 *	@param	pStrPose		:	[in]  Pose string in AsciiZ
 *	@param	pstrOption		:	[in]  Option string in AsciiZ
 *	@retval	BCAP_HRESULT
 *
 */
BCAP_HRESULT bCap_RobotMove(int iSockFd, u_long lhRobot, long lComp, char *pStrPose, char *pStrOption){
	BCAP_PACKET		*pSndPacket;
	BCAP_PACKET		*pRecPacket;
	BCAP_ARG		*pArg;

	BCAP_HRESULT hr = BCAP_E_FAIL;

	pSndPacket = Packet_Create(BCAP_FUNC_Robot_Move);		/* BCAP_FUNC_Robot_Move */
	if (pSndPacket != NULL){

		u_char buff[LOCALBUFFER_SZ];
		u_long lLen;

		{
			pArg = Arg_Create( VT_I4, 1, 4, &lhRobot);		/* Arg1 Handle of the robot */
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			pArg = Arg_Create( VT_I4, 1, 4, &lComp);		/* Arg2 Completion param */
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			lLen = copyToBSTR(buff,pStrPose);				/* Arg3 Pose param */
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			lLen = copyToBSTR(buff,pStrOption);				/* Arg4 option param */
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}


		{
			pRecPacket = Packet_Create(BCAP_S_OK);
			if (pRecPacket != NULL){
				hr = bCapSendAndRec(iSockFd, pSndPacket, pRecPacket);
				if SUCCEEDED(hr){
					;
				}

			}
			Packet_Release(pRecPacket);
		}
		Packet_Release(pSndPacket);
	}

	return hr;
}


/**	Task_Release
 *
 * Task_Release
 *
 *	@param	iSockFd	:	[in]  Socket descriptor
 *	@param	lhTask		:	[in]  task handle
 *	@retval	BCAP_HRESULT
 *
 */
BCAP_HRESULT bCap_TaskRelease(int iSockFd, u_long lhTask){
	BCAP_PACKET		*pSndPacket;
	BCAP_PACKET		*pRecPacket;
	BCAP_ARG		*pArg;


	BCAP_HRESULT hr = BCAP_E_FAIL;

	pSndPacket = Packet_Create(BCAP_FUNC_Task_Release);		/* BCAP_FUNC_Task_Release */
	if (pSndPacket != NULL){

		{
			pArg = Arg_Create( VT_I4, 1, 4, &lhTask);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);				/* Arg1 lhTask */
			}
		}

		{
			pRecPacket = Packet_Create(BCAP_S_OK);				/* Alloc for storing recieved packet */
			if (pRecPacket != NULL){
				hr = bCapSendAndRec(iSockFd, pSndPacket, pRecPacket);

			}
			Packet_Release(pRecPacket);							/* Release recieved packet */
		}
		Packet_Release(pSndPacket);
	}

	return hr;
}


/**	Task_GetVariable
 *
 * Task_GetVariable
 *
 *	@param	iSockFd			:	[in]  Socket descriptor
 *	@param	lhTask			:	[in]  task handle
 *	@param	pVarName		:	[in]  Variable name string in AsciiZ
 *	@param	pstrOption		:	[in]  Option string in AsciiZ
 *	@param	plhVar			:	[out]  result value = variable handle
 *	@retval	BCAP_HRESULT
 *
 */
BCAP_HRESULT bCap_TaskGetVariable(int iSockFd, u_long lhTask, char *pVarName, char *pstrOption, u_long *plhVar){
	BCAP_PACKET		*pSndPacket;
	BCAP_PACKET		*pRecPacket;
	BCAP_ARG		*pArg;

	BCAP_HRESULT hr = BCAP_E_FAIL;

	pSndPacket = Packet_Create(BCAP_FUNC_Task_GetVariable);		/* BCAP_FUNC_Task_GetVariable */
	if (pSndPacket != NULL){

		u_char buff[LOCALBUFFER_SZ];
		u_long lLen;

		{
			pArg = Arg_Create( VT_I4, 1, 4, &lhTask);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			lLen = copyToBSTR(buff,pVarName);
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			lLen = copyToBSTR(buff, pstrOption);
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			pRecPacket = Packet_Create(BCAP_S_OK);
			if (pRecPacket != NULL){
				hr = bCapSendAndRec(iSockFd, pSndPacket, pRecPacket);
				if SUCCEEDED(hr){
					if (pRecPacket->iArgs >= 1){
						copyValue(plhVar, pRecPacket->pArg->data, 4);
					}
					else{

					}
				}
			}
			Packet_Release(pRecPacket);
		}
		Packet_Release(pSndPacket);
	}

	return hr;
}


/**	Task_Start
 *
 * Task_Start
 *
 *	@param	iSockFd	:	[in]  Socket descriptor
 *	@param	lhTask		:	[in]  task handle
 *	@param	lMode		:	[in]  start parameter
 *	@param	pstrOption		:	[in]  Option string in AsciiZ
 *	@retval	BCAP_HRESULT
 *
 */
BCAP_HRESULT bCap_TaskStart(int iSockFd, u_long lhTask, long lMode, char *pStrOption){
	BCAP_PACKET		*pSndPacket;
	BCAP_PACKET		*pRecPacket;
	BCAP_ARG		*pArg;

	BCAP_HRESULT hr = BCAP_E_FAIL;

	pSndPacket = Packet_Create(BCAP_FUNC_Task_Start);		/* BCAP_FUNC_Task_Start */
	if (pSndPacket != NULL){

		u_char buff[LOCALBUFFER_SZ];
		u_long lLen;

		{
			pArg = Arg_Create( VT_I4, 1, 4, &lhTask);		/* Arg1 Handle of the task */
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			pArg = Arg_Create( VT_I4, 1, 4, &lMode);		/* Arg2 start param */
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}


		{
			lLen = copyToBSTR(buff,pStrOption);				/* Arg3 option param */
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}


		{
			pRecPacket = Packet_Create(BCAP_S_OK);
			if (pRecPacket != NULL){
				hr = bCapSendAndRec(iSockFd, pSndPacket, pRecPacket);
				if SUCCEEDED(hr){
					;
				}

			}
			Packet_Release(pRecPacket);
		}
		Packet_Release(pSndPacket);
	}

	return hr;
}


/**	Task_Stop
 *
 * Task_Stop
 *
 *	@param	iSockFd	:	[in]  Socket descriptor
 *	@param	lhTask		:	[in]  task handle
 *	@param	lMode		:	[in]  stop parameter
 *	@param	pstrOption		:	[in]  Option string in AsciiZ
 *	@retval	BCAP_HRESULT
 *
 */
BCAP_HRESULT bCap_TaskStop(int iSockFd, u_long lhTask, long lMode, char *pStrOption){
	BCAP_PACKET		*pSndPacket;
	BCAP_PACKET		*pRecPacket;
	BCAP_ARG		*pArg;

	BCAP_HRESULT hr = BCAP_E_FAIL;

	pSndPacket = Packet_Create(BCAP_FUNC_Task_Stop);		/* BCAP_FUNC_Task_Stop */
	if (pSndPacket != NULL){

		u_char buff[LOCALBUFFER_SZ];
		u_long lLen;

		{
			pArg = Arg_Create( VT_I4, 1, 4, &lhTask);		/* Arg1 Handle of the task */
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			pArg = Arg_Create( VT_I4, 1, 4, &lMode);		/* Arg2 stop param */
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}


		{
			lLen = copyToBSTR(buff,pStrOption);				/* Arg3 option param */
			pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}


		{
			pRecPacket = Packet_Create(BCAP_S_OK);
			if (pRecPacket != NULL){
				hr = bCapSendAndRec(iSockFd, pSndPacket, pRecPacket);
				if SUCCEEDED(hr){
					;
				}

			}
			Packet_Release(pRecPacket);
		}
		Packet_Release(pSndPacket);
	}

	return hr;
}


/**	Variable_Release
 *
 * Variable_Release
 *
 *	@param	iSockFd	:	[in]  Socket descriptor
 *	@param	lhVariable		:	[in]  variable handle
 *	@retval	BCAP_HRESULT
 *
 */
BCAP_HRESULT bCap_VariableRelease(int iSockFd, u_long lhVar){
	BCAP_PACKET		*pSndPacket;
	BCAP_PACKET		*pRecPacket;
	BCAP_ARG		*pArg;


	BCAP_HRESULT hr = BCAP_E_FAIL;

	pSndPacket = Packet_Create(BCAP_FUNC_Variable_Release);		/* BCAP_FUNC_Task_Release */
	if (pSndPacket != NULL){

		{
			pArg = Arg_Create( VT_I4, 1, 4, &lhVar);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);				/* Arg1 lhTask */
			}
		}

		{
			pRecPacket = Packet_Create(BCAP_S_OK);				/* Alloc for storing recieved packet */
			if (pRecPacket != NULL){
				hr = bCapSendAndRec(iSockFd, pSndPacket, pRecPacket);

			}
			Packet_Release(pRecPacket);							/* Release recieved packet */
		}
		Packet_Release(pSndPacket);
	}

	return hr;
}


/**	Variable_GetValue
 *
 * Variable_GetValue
 *
 *	@param	iSockFd			:	[in]  Socket descriptor
 *	@param	lhVar			:	[in]  robot handle
 *	@param	*pVntValue		:	[out]  value stored pointer
 *	@retval	BCAP_HRESULT
 *	@detail	Note:	 This function write value into *pVntValue,
 *					So, Client program must allocate enough memory as *pVntValue.
 */
BCAP_HRESULT bCap_VariableGetValue(int iSockFd, u_long lhVar, void *pVntValue){
	BCAP_PACKET		*pSndPacket;
	BCAP_PACKET		*pRecPacket;
	BCAP_ARG		*pArg;

	BCAP_HRESULT hr = BCAP_E_FAIL;

	pSndPacket = Packet_Create(BCAP_FUNC_Variable_GetValue);		/* BCAP_FUNC_Variable_GetValue */
	if (pSndPacket != NULL){

		{
			pArg = Arg_Create( VT_I4, 1, 4, &lhVar);				/* Arg1: Variable handle */
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			pRecPacket = Packet_Create(BCAP_S_OK);
			if (pRecPacket != NULL){
				hr = bCapSendAndRec(iSockFd, pSndPacket, pRecPacket);
				if SUCCEEDED(hr){
					if (pRecPacket->iArgs >= 1){
						{	/* Copy values */
							u_long i;
							u_long lSize;
							u_short iType;
							BCAP_ARG	*pArgValue = pRecPacket->pArg;

							iType = (pArgValue->iType) & ~VT_ARRAY;	/* Mask "Array" */
							if (iType == VT_BSTR){
								u_char *pDstAscii = (u_char *)pVntValue;
								u_char *pSrcBstr = (u_char *)pArgValue->data;

								for (i = 0;i < pArgValue->lArrays;i++){
									lSize = copyFromBSTR(pDstAscii, pSrcBstr);
									pDstAscii += lSize;
									pSrcBstr += BCAP_SIZE_ARGSTRLEN + ((lSize -1) * 2);	/* lSize include Terminator,so (lSize -1) * 2) */
								}
							}
							else{
								lSize = sizeOfVarType(pArgValue->iType);
								if (lSize != 0){
									u_char *pDst = (u_char *)pVntValue;
									u_char *pSrc = (u_char *)(u_char *)pArgValue->data;

									for (i = 0;i < pArgValue->lArrays;i++){
										copyValue(pDst, pSrc, lSize);
										pDst += lSize;
										pSrc += lSize;
									}
								}
							}
						}
					}
					else{
						/* NO Argument */
						hr = BCAP_E_FAIL;
					}
				}

			}
			Packet_Release(pRecPacket);
		}
		Packet_Release(pSndPacket);
	}

	return hr;
}

/**	Variable_PutValue
 *
 * Variable_PutValue
 *
 *	@param	iSockFd			:	[in]  Socket descriptor
 *	@param	lhVar			:	[in]  robot handle
 *	@param	iType			:	[in]  variable type
 *	@param	lArrays			:	[in]  array counter
 *	@param	*pVntValue		:	[in]  value stored pointer
 *	@retval	BCAP_HRESULT
 */
BCAP_HRESULT bCap_VariablePutValue(int iSockFd, u_long lhVar, u_short iType, u_long lArrays, void  *pVntValue){
	BCAP_PACKET		*pSndPacket;
	BCAP_PACKET		*pRecPacket;
	BCAP_ARG		*pArg;

	BCAP_HRESULT hr = BCAP_E_FAIL;

	pSndPacket = Packet_Create(BCAP_FUNC_Variable_PutValue);		/* BCAP_FUNC_Variable_PutValue */
	if (pSndPacket != NULL){

		{
			pArg = Arg_Create( VT_I4, 1, 4, &lhVar);
			if (pArg != NULL){
				Packet_AddArg(pSndPacket, pArg);
			}
		}

		{
			u_long lDataLen = 0;

			if ((iType  & ~VT_ARRAY) == VT_BSTR) {		/* Mask "Array" */	/* IMPL:Not Support String array now. */
				/* String data */

				u_char buff[LOCALBUFFER_SZ];
				u_long lLen;

				lLen = copyToBSTR(buff,pVntValue);
				pArg = Arg_Create( VT_BSTR, 1, lLen,buff);
				if (pArg != NULL){
					Packet_AddArg(pSndPacket, pArg);
				}
			}
			else{
				/* Not string data */

				u_long lLen;
				lLen = sizeOfVarType(iType  & ~VT_ARRAY);
				lDataLen = lLen * lArrays;
				if (lDataLen != 0){
					u_long i;
					u_char *pSrcData = (u_char *)pVntValue;
					u_char *pDstData = (u_char *)bMalloc(lDataLen);
					if (pDstData != NULL){
						u_char *pDstPtr = pDstData;
						for (i = 0;i < lArrays;i++){

							copyValue(pDstPtr, pSrcData, lLen);
							pDstPtr += lLen;
							pSrcData += lLen;
						}

						pArg = Arg_Create( iType, lArrays, lDataLen, pDstData);
						if (pArg != NULL){
							Packet_AddArg(pSndPacket, pArg);
						}
						bFree(pDstData);
					}
				}
				else{	/* lDataLen = 0,then Unknown data type */
					Packet_Release(pSndPacket);
					return (BCAP_E_INVALIDARGTYPE);
				}
			}



		}


		{
			pRecPacket = Packet_Create(BCAP_S_OK);
			if (pRecPacket != NULL){
				hr = bCapSendAndRec(iSockFd, pSndPacket, pRecPacket);
				if SUCCEEDED(hr){
					;
				}

			}
			Packet_Release(pRecPacket);
		}
		Packet_Release(pSndPacket);
	}

	return hr;
}

/*--------------------------------------------------------------------
			b-Cap library utility functions
 --------------------------------------------------------------------*/


/**	Send and Receive bCap Packet
 *
 * Send a b-Cap packet and Receive a Packet
 *
 *	@param	pSndPacket		:	[in]  Send packet pointer
 *	@param	pRecPacket		:	[out]  Received packet pointer
 *	@retval	BCAP_HRESULT
 *
 */
static BCAP_HRESULT bCapSendAndRec(int iSockFd, BCAP_PACKET		*pSndPacket, BCAP_PACKET		*pRecPacket){

	BCAP_HRESULT hr = BCAP_E_FAIL;

	if ( (pRecPacket != NULL) && (pSndPacket != NULL) ){

		if (Packet_Send(iSockFd, pSndPacket) == BCAP_S_OK){

			u_char *pRec = receivePacket(iSockFd);	/* Receive packet and alloc memory for storing received binary */
			if(pRec != NULL){
				hr = Packet_Deserialize(pRec, pRecPacket );
				if SUCCEEDED(hr){
					hr = (BCAP_HRESULT)pRecPacket->lFuncID;
				}


				bFree(pRec);						/* Free received packet */
			}
			else{
				/* allcation error */
				hr = BCAP_E_UNEXPECTED;
			}
		}
		else{
			/* send error  */
			hr = BCAP_E_UNEXPECTED;
		}
	}
	else{
		/* Null pointer is asigned */
		hr = BCAP_E_FAIL;
	}

	return hr;

}


/**	Receive bCap Packet
 *
 * Receive bCap Packet from server
 *
 *	@retval	u_char pointer storing recieved packet
 *	@detail	Note:	If valid pointer is returned, this function allocate memory for sotring packet.
 *					Never forget to free() the pointer.
 *					And If NULL pointer is returned , then was allocation error/
 *
 */
static u_char *receivePacket(int iSockFd){

	u_char	pRcvBuffer[LOCALRECBUFFER_SZ];
	u_char *pTop;
	u_char	*pPacketBuff = NULL;
	u_char	*pRemainData;

	u_long lRecvSize;
	int lRecLen;
	u_long lHeaderLen;

	/* b-CAP header = 15 bytes, this should be recieved at first */
	lHeaderLen = BCAP_SIZE_SOH + BCAP_SIZE_LEN +
				BCAP_SIZE_SERIAL + BCAP_SIZE_RESERVE +
				BCAP_SIZE_FUNCID + BCAP_SIZE_ARGNUM;

	/* Receive b-Cap header */
		lRecvSize = 0;
		while (lRecvSize < lHeaderLen) {
			lRecLen = recv(iSockFd, (char *)&(pRcvBuffer[lRecvSize]), lHeaderLen - lRecvSize, 0);
			if(lRecLen <= 0) {	/* if sock errer has detected, then exit  */
				goto ExitPoint;
			}
			lRecvSize += lRecLen;			/* add read bytes */

			pTop = (u_char *)memchr((const void *)pRcvBuffer, BCAP_SOH, lRecvSize);
			if (pTop == NULL){				/* Is there SOH ? */
				lRecvSize = 0;				/* If No SOH, then all read data are discarded */
			}
			else{
				if (pTop != pRcvBuffer){	/* if (pTop == pRcvBuffer) then SOH is already in the top. */
					lRecvSize = lRecvSize - (pTop - pRcvBuffer);	/* exclude before SOH  */
					memmove (pRcvBuffer, pTop, lRecvSize);
				}
			}
		}

	/* Receive the left data of this packet */
	{
		u_long lPacketSize;
		u_long lRemainSize;

		copyValue(&lPacketSize, &(pRcvBuffer[1]), BCAP_SIZE_LEN);
		lRemainSize  = lPacketSize - lRecvSize;

		pPacketBuff = (unsigned char *)bMalloc(lPacketSize);
		if (pPacketBuff != NULL){
			memcpy(pPacketBuff, pRcvBuffer, lRecvSize);
			pRemainData = pPacketBuff + lRecvSize;
		}
		else{
			goto ExitPoint;	/* out of memory */
		}

		lRecvSize = 0;
		while (lRecvSize < lRemainSize) {
			lRecLen = recv(iSockFd, (char *)&(pRemainData[lRecvSize]), lRemainSize -lRecvSize , 0);
			if(lRecLen <= 0) {	/* if sock errer has detected, then exit  */

				goto ExitPoint;
			}
			lRecvSize += lRecLen;			/* add read bytes */
		}

		/* Check Terminator EOT  */
		if (pPacketBuff[lPacketSize - 1] != BCAP_EOT) {
			goto ExitPoint;
		}

	}


	return pPacketBuff;

ExitPoint:
	if (pPacketBuff != NULL) {
		free(pPacketBuff);
		pPacketBuff = NULL;
	}
	return NULL;

}



/**	Send bCap Packet
 *
 *  Send bCap Packet to server
 *
 *	@param	pRecPacket	:	[out]  Received packet pointer
 *	@retval	u_char pointer storing recieved packet
 *			Note:	If valid pointer is returned, this function allocate memory for sotring packet.
 *					Never forget to free() the pointer.
 *					And If NULL pointer is returned , then was allocation error/
 *
 */
 static BCAP_HRESULT sendBinary(int iSockFd, u_char *pBuff, u_long lSize){

	BCAP_HRESULT hr = BCAP_E_FAIL;
	int iLen;
	int iSent;

	iLen = (int)lSize;
	if (iLen > 0){

		iSent = send(iSockFd, (char *)pBuff, iLen, 0);
		if (iSent == iLen){
			 hr = BCAP_S_OK;
		}

	}
	return hr;
}


/**	Send b-Cap packet
 *
 * Convert Send a b-Cap packet and Receive a Packet
 *
 *	@param	iSd	:	[in]  Socket descriptor
 *	@param	pPacket		:	[in]  bCap packet data to send
 *	@retval	BCAP_HRESULT is returned.
 *
 */
static BCAP_HRESULT			Packet_Send(int iSockFd, BCAP_PACKET *pPacket){

	BCAP_HRESULT hr = BCAP_E_FAIL;

	void *pbSendData;

	pbSendData = bMalloc(pPacket->lMsgLength);
	if (pbSendData != NULL){
		if (Packet_Serialize(pPacket, pbSendData) == BCAP_S_OK){

			hr = sendBinary(iSockFd, (u_char *)pbSendData, pPacket->lMsgLength);	/* Send packet  */
			if (hr == BCAP_S_OK){
				;
			}
		}
		bFree(pbSendData);
	}

	return hr;
}


/**	Packet_GetLastArgHandle
 *
 * Get last argument pointer in the packet
 *
 *	@param	*pPacket	:	[in]  packet pointer
 *	@retval	The last arg pointer is returned.
 *
 */
static BCAP_ARG		**Packet_GetLastArgHandle( BCAP_PACKET *pPacket){
	BCAP_ARG **pLastArg = NULL;
	BCAP_ARG *pArg;

	if (pPacket != NULL){

		pArg = pPacket->pArg;					/* set base pointer in packet struct */
		pLastArg = 	&(pPacket->pArg);				/* if this is NULL in this time, this packet has no argment.
													So, pPacket->pArg is set as the last arg. */

		/* search the last arg pointer */
		while(pArg != NULL){
			pLastArg = &(pArg->pNextArg);

			pArg = pArg->pNextArg;				/* set next arg pointer. it might be NULL */
		}
	}

	if (pLastArg != NULL){
		return pLastArg;
	}
	else{
		return NULL;
	}
}


/**	Packet_AddArg
 *
 * Add the arg into the packet
 *
 *	@param	*pPacket	:	[in]  Packet pointer .
 *	@param	*pNewArg	:	[in]  Arg pointer should be added into pPacket.
 *	@retval	BCAP_HRESULT is returned.
 *
 */
static BCAP_HRESULT			Packet_AddArg( BCAP_PACKET *pPacket, BCAP_ARG *pNewArg){

	BCAP_HRESULT hr = BCAP_S_OK;
	BCAP_ARG **pLastArg = NULL;

	if ( (pPacket != NULL) && (pNewArg != NULL)) {
		pLastArg = Packet_GetLastArgHandle(pPacket);	/* get last arg pointer */
		if (pLastArg == NULL){

			/* Bad pointer */
			hr = BCAP_E_FAIL;
		}
		else{
			/* Not NULL: then success to find the last arg */
			*pLastArg  = pNewArg;		/* Set the new arg pointer as the next arg pointer */
		}

		pPacket->iArgs++;						/* Increment arg counter of this packet */

		pPacket->lMsgLength += (pNewArg->lLength + BCAP_SIZE_ARGLEN);
												/* Add arg size into Packet length */
		hr = BCAP_S_OK;
	}
	else{

		/* Bad pointer */
		hr = BCAP_E_FAIL;
	}

	return hr;
}

/**	Packet_Release
 *
 * Release packet and all args in recursive.
 *
 *	@param	*pPacket	:	[in]  Packet pointer .
 *	@retval	void
 *
 */
static void			Packet_Release( BCAP_PACKET *pPacket){


	/* release poacket  */
	if (pPacket != NULL){
		/* release all args  */
		Arg_Release(pPacket->pArg);

		/* release instance of the packet */
		bFree(pPacket);
	}
}

/**	Packet_Create
 *
 * Create and allocate new packet structure.
 *
 *	@param	lFuncID		:	[in]  Function ID.
 *	@retval	New packet pointer is returned.
 *
 */
static BCAP_PACKET	*Packet_Create( u_long lFuncID){

	 BCAP_PACKET *pNewPacket = NULL;

	 pNewPacket = (BCAP_PACKET *)bMalloc(sizeof(BCAP_PACKET));	/* alloc packet */
	 if (pNewPacket != NULL){

		pNewPacket->iArgs = 0;						/* args count */
		pNewPacket->iReserved = 0;
		pNewPacket->iSerialNo = m_lSerialNo;
		pNewPacket->lFuncID = lFuncID;

		pNewPacket->lMsgLength = BCAP_SIZE_BASE;

		pNewPacket->pArg = NULL;

		m_lSerialNo++;								/* increment serial number */
	 }

	return pNewPacket;
}



/**	Serialize from packet into byte arrays.
 *
 * Change from struct into binary array.
 *
 *	@param	*pSrcPacket		:	[in]  Packet pointer to send.
 *	@param	*pDstBinData		:	[out]  byte pointer to store packet
 *	@retval	BCAP_HRESULT is returned.
 *
 */
static BCAP_HRESULT		Packet_Serialize(BCAP_PACKET *pSrcPacket, void *pDstBinData){


	BCAP_HRESULT hr = BCAP_S_OK;
	u_char *pDstPtr;

	pDstPtr = (u_char *)pDstBinData;

	/* SOH */
	*pDstPtr = BCAP_SOH;
	pDstPtr += BCAP_SIZE_SOH;
	/* Header */
	pDstPtr += copyValue( pDstPtr, &(pSrcPacket->lMsgLength), sizeof(pSrcPacket->lMsgLength));
	pDstPtr += copyValue( pDstPtr, &(pSrcPacket->iSerialNo), sizeof(pSrcPacket->iSerialNo));
	pDstPtr += copyValue( pDstPtr, &(pSrcPacket->iReserved), sizeof(pSrcPacket->iReserved));
	pDstPtr += copyValue( pDstPtr, &(pSrcPacket->lFuncID), sizeof(pSrcPacket->lFuncID));
	pDstPtr += copyValue( pDstPtr, &(pSrcPacket->iArgs), sizeof(pSrcPacket->iArgs));


	{	/* Copy args */
		unsigned int i,j;
		BCAP_ARG *pArgPtr;
		pArgPtr = pSrcPacket->pArg;
		u_char *pbValue;

		for (i =0 ; i < pSrcPacket->iArgs ; i++){
			if (pArgPtr != NULL){
				{
					pDstPtr += copyValue( pDstPtr, &(pArgPtr->lLength), sizeof(pArgPtr->lLength));
					pDstPtr += copyValue( pDstPtr, &(pArgPtr->iType), sizeof(pArgPtr->iType));
					pDstPtr += copyValue( pDstPtr, &(pArgPtr->lArrays), sizeof(pArgPtr->lArrays));

					pbValue = (u_char *)pArgPtr->data;					/* value stored pointer is set */

					for (j= 0; j < pArgPtr->lArrays; j++){
						u_long lValueSize;
						u_short iVarType = pArgPtr->iType & ~VT_ARRAY;	/* Mask "Array" */
						switch (iVarType){
							case VT_UI1:
							case VT_I2:
							case VT_UI2:
							case VT_I4:
							case VT_UI4:
							case VT_R4:
							case VT_R8:
							case VT_BOOL:
								{
									lValueSize = sizeOfVarType(iVarType);
									pDstPtr += copyValue( pDstPtr, pbValue, lValueSize);
									pbValue += lValueSize;					/* value stored pointer is added */
								}
								break;

							case VT_BSTR:
								{
									u_long lStrLen;
									copyValue( &lStrLen, pbValue, BCAP_SIZE_ARGSTRLEN);
									pDstPtr += copyValue( pDstPtr, &lStrLen, BCAP_SIZE_ARGSTRLEN);	/* Set String length (4 bytes) */
									pbValue += BCAP_SIZE_ARGSTRLEN;		/* value stored pointer is added */

									pDstPtr += copyValue( pDstPtr, pbValue, lStrLen);	/* Set string data */
									pbValue += lStrLen;
								}
								break;

							case VT_VARIANT:
								/* NOT_IMPLEMENT */
								{


								}


								break;

							default:
								break;
						}
					}

				}
				pArgPtr = pArgPtr->pNextArg;	/* Set pointer to the next arg. */
			}
		}

	}

	/* EOT */
	*pDstPtr = BCAP_EOT;
	pDstPtr += BCAP_SIZE_EOT;

	return hr;
}


/** Deserialize from byte array into packet.
 *
 * Change from byte arrays into struct.
 *
 *	@param	*pSrcBinData		:	[in]  Byte pointer to send.
 *	@param	*pDstPacket			:	[out]  packet pointer to store
 *	@retval	BCAP_HRESULT is returned.
 *	@detail	bin  ---> struct
 *			Note:	If something wrong, then relwase this packet and args that included
 */

static BCAP_HRESULT		Packet_Deserialize(void *pSrcBinData, BCAP_PACKET *pDstPacket){


	BCAP_HRESULT hr = BCAP_S_OK;
	u_char *pSrcPtr;

	u_short	iArgs;
	u_long	lMsgLength;

	pSrcPtr = (u_char *)pSrcBinData;

	/* SOH */
	/* pDstPtr += copyValue( &(pDstPacket->Header), pDstPtr, BCAP_SIZE_SOH)); */
	pSrcPtr += BCAP_SIZE_SOH;

	/* Header */
	/* "pkt->lMsgLength" and "pkt->lArgs" are calcurated in Packet_AddArg()  automatically */
	pSrcPtr += copyValue( &lMsgLength				, pSrcPtr, sizeof(pDstPacket->lMsgLength));	/*  calcurated in Packet_AddArg()  automatically */
	pSrcPtr += copyValue( &(pDstPacket->iSerialNo)	, pSrcPtr, sizeof(pDstPacket->iSerialNo));
	pSrcPtr += copyValue( &(pDstPacket->iReserved)	, pSrcPtr, sizeof(pDstPacket->iReserved));
	pSrcPtr += copyValue( &(pDstPacket->lFuncID)	, pSrcPtr, sizeof(pDstPacket->lFuncID));
	pSrcPtr += copyValue( &iArgs					, pSrcPtr, sizeof(pDstPacket->iArgs));		/*  calcurated in Packet_AddArg()  automatically */


	{	/* Copy args */
		unsigned int i;
		BCAP_ARG *pArgPtr;
		pArgPtr = pDstPacket->pArg;

		for (i = 0 ; i < iArgs ; i++){
			u_long	lDataSize;	/* size of "*data" */
			u_long	lLength;	/* size of argument block */
			u_short iType;
			u_long	lArrays;

			pSrcPtr += copyValue( &lLength, pSrcPtr, sizeof(lLength));		/* size of a argument block  */
			pSrcPtr += copyValue( &iType, pSrcPtr, sizeof(iType));
			pSrcPtr += copyValue( &lArrays, pSrcPtr, sizeof(lArrays));

			lDataSize = lLength - BCAP_SIZE_ARGTYPE -BCAP_SIZE_ARGARRAYS;	/* size of "*data" */

			pArgPtr = Arg_Create(iType, lArrays, lDataSize, pSrcPtr);		/* Create new arg */
			if (pArgPtr != NULL){
				pSrcPtr = pSrcPtr + lDataSize;								/* skip "*data"  */
			}
			else{
				hr = BCAP_E_FAIL;
				break;
			}

			if (Packet_AddArg(pDstPacket, pArgPtr) != BCAP_S_OK){				/* Add new arg to packet */
				Arg_Release(pArgPtr);										/* Fail to add, then release this arg. */
				hr = BCAP_E_FAIL;
				break;
			}
		}

	}

	/* EOT */
	if (hr == BCAP_S_OK){
		if (*pSrcPtr != BCAP_EOT){											/* If end of the binnary is not EOT, then error */
			hr = BCAP_E_UNEXPECTED;
		}
	}
	pSrcPtr += BCAP_SIZE_EOT;

	return hr;
}


/**	Arg_Create
 *
 * Create and allocate b-Cap argument structure
 *
 *	@param	iType		:	[in]  Variable type
 *	@param	lArrays		:	[in]  Arrays
 *	@param	lDataSize	:	[in]  total byte of ( "*data" )
 *	@param	 *data		:	[in]  value pointer
 *	@retval	allocated pointer is returned
 *
 *	@detail	Note:When BSTR is used, *data must be store "Length:4byte" + "DoubleByte String"
 *					See alose function CopyToBSTR().
 */
static BCAP_ARG		*Arg_Create( u_short iType, u_long lArrays, u_long lDataSize, void *data){

	BCAP_ARG *pNewArg = NULL;

	if ((0 < lDataSize) && (lDataSize < BCAP_MAX_ARG_SIZE)){		/* (0 < ) has something data, (<BCAP_MAX_ARG_SIZE) not over size */

		pNewArg = (BCAP_ARG *)bMalloc(sizeof(BCAP_ARG));			/* alloc argument */
		if (pNewArg != NULL){
			pNewArg->iType = iType;
			pNewArg->lArrays = lArrays;
			pNewArg->lLength = lDataSize + BCAP_SIZE_ARGTYPE + BCAP_SIZE_ARGARRAYS;
			pNewArg->pNextArg = NULL;
			pNewArg->data = NULL;

			pNewArg->data = bMalloc(lDataSize);
			if (pNewArg->data != NULL ){
				if (data != NULL){
					memcpy(pNewArg->data, data, lDataSize);
				}
				else{
					/* Fail to copy from *data, then release this argumant.  */
					Arg_Release(pNewArg);
					pNewArg = NULL;								/* return value (allocated pointer) */
				}

			}
			else{	/* fail to alloc memory */
				/* Fail to alloc memory, then release this argumant.  */
				Arg_Release(pNewArg);
				pNewArg = NULL;								/* return value (allocated pointer) */
			}
		}
	 }


	return pNewArg;
}


/**	Arg_Release
 *
 *	Release all args in recursive
 *
 *	@param	*pArg		:	[in]  Arg pointer
 *	@retval	void
 *
 */
static void			Arg_Release( BCAP_ARG *pArg){

	while (pArg != NULL){
		BCAP_ARG *pNextArg;

		pNextArg = pArg->pNextArg;							/* store next pointer in temporary */

		if (pArg->data != NULL){
			bFree(pArg->data);								/* free value of argument */
		}

		bFree(pArg);											/* free argument */
		pArg = pNextArg;									/* set next pointer */
	}
}

/**	get size of variable
 *
 * get size of variable
 *
 * @param	iType		:	[in]  Variable type
 * @retval	size of variable.
 * @detail	Note 1: If (iType = VT_BSTR), then this function returns BCAP_SIZE_ARGSTRLEN (= 4 bytes)
 *			Note 2: Not support VT_VARIANT,VT_EMPTY,VT_NULL,VT_ERROR,VT_CY,VT_DATE
 */
static u_long sizeOfVarType(u_short iType){

	 u_long lValueSize = 0;

	 switch (iType & ~VT_ARRAY){
		case VT_UI1:
			lValueSize = 1;
			break;
		case VT_I2:
			lValueSize = 2;
			break;
		case VT_UI2:
			lValueSize = 2;
			break;
		case VT_I4:
			lValueSize = 4;
			break;
		case VT_UI4:
			lValueSize = 4;
			break;
		case VT_R4:
			lValueSize = 4;
			break;
		case VT_R8:
			lValueSize = 8;
			break;
		case VT_BOOL:
			lValueSize = 2;
			break;
		case VT_BSTR:
			/* In this function  */
			lValueSize = BCAP_SIZE_ARGSTRLEN;
			break;

		case VT_VARIANT:	/* Not implement */
			break;

		default:		/*  Not implement */
						/*	VT_EMPTY	 */
						/*	VT_NULL		 */
						/*	VT_ERROR	 */
						/*	VT_CY		 */
						/*	VT_DATE		 */
			break;
	}

	 return lValueSize;
}

/**	Convert into BSTR from AsciiZ
 *
 * Convert Send a b-Cap packet and Receive a Packet
 *
 *	@param	pDstPtr	:	[out]  BSTR pointer
 *	@param	pSrcPtr		:	[in]  String pointer in AsciiZ
 *	@retval	total size of BSTR is returned.
 *
 */
 u_long copyToBSTR(void *pDstBstrPtr, void *pSrcAsciiPtr){
	u_char *pbDst = (u_char *)pDstBstrPtr;
	u_char *pbSrc = (u_char *)pSrcAsciiPtr;
	u_long	lStrLen,lLen2;
	u_long	i;

	lStrLen = strlen((const char *)pbSrc);									/* length of source string (ascii) */
	lLen2 = lStrLen * 2;

	if (pbDst != NULL){
		pbDst += copyValue(pbDst, &lLen2, BCAP_SIZE_ARGSTRLEN);

		for (i = 0;i < lStrLen; i++){
			*pbDst++ =  *pbSrc++;
			*pbDst++ =  0;
		}

	}
	return (lStrLen * 2 + BCAP_SIZE_ARGSTRLEN);
}

 /**	Convert From BSTR into AsciiZ
 *
 * Convert Send a b-Cap packet and Receive a Packet
 *
 *	@param	pDstPtr		:	[out]  String pointer in AsciiZ
 *	@param	pSrcPtr		:	[in]  BSTR pointer
 *	@retval	total size of BSTR is returned.
 *
 */
static u_long copyFromBSTR(void *pDstAsciiPtr, void *pSrcBstrPtr){
	u_char *pbDst = (u_char *)pDstAsciiPtr;
	u_char *pbSrc = (u_char *)pSrcBstrPtr;
	u_long	lStrLen,lLen2;
	u_long	i;

	copyValue(&lStrLen, pbSrc, BCAP_SIZE_ARGSTRLEN);					/* Get BStr length */
	pbSrc += BCAP_SIZE_ARGSTRLEN;

	lLen2 = lStrLen / 2;

	if (pbDst != NULL){
		for (i = 0;i < lLen2; i++){
			*pbDst = *pbSrc;
			pbDst += 1; /* Ascii */
			pbSrc += 2;	/* BSTR */

		}
		*pbDst = 0;		/* Zero termination */
		lLen2++;		/* +1 = (Zero termination) */
	}
	return (lLen2);
}

 /*
 * 	copy value in the "Little Endian"
 *
 *		alternate of htonl()/ntohl()
 */
static u_long copyValue(void *pDst, void *pVntValue, u_long lLength){

#if defined(__BIG_ENDIAN__)

	/* SPARC/MC68xxx */

	/* copy values inversion. b-CAP is based on little-endian */
	{
		u_long i;

		u_char *pbDst;
		u_char *pbSrc;

		pbSrc = (u_char *)(pVntValue) + lLength -1;
		pbDst = (u_char *)pDst;

		for (i = 0 ; i < lLength ; i++){
			*pbDst++ = *pbSrc--;
		}
	}
#else

	memcpy(pDst, pVntValue, lLength);
#endif

	return lLength;									/* return copied bytes size */
}

/*
 *	Memory allocation counter
 */
static long m_lAllocCount = 0;
static long m_lAllocSize = 0;

/*
 *	alternative of Malloc()
 */
static void *bMalloc(size_t size){

	void *pPtr;

	m_lAllocCount++;
	m_lAllocSize += size;

	pPtr = malloc(size);

#ifdef DEBUG
	printf("AllocCount:%d\n",m_lAllocCount);
#endif

	return pPtr;
}

/*
 *	alternative of Free()
 */
static void bFree(void *pPtr){

	m_lAllocCount--;
#ifdef DEBUG
	printf("AllocCount:%d\n",m_lAllocCount);
#endif
	free(pPtr);
}

