#ifndef XPlane_GIMcom_H
#define XPlane_GIMcom_H

#include <windows.h>


#define PackageSize  19
#define txBuffSize   32
#define rxBuffSize   125

#define COM_PORT "\\\\.\\COM10"


class XPlane_GIMcom
{
protected:

	DCB				dcb;
	HANDLE			hCom;
	unsigned char	TxBuffer[txBuffSize];
	DWORD			dwBytesWritten;


public:
	XPlane_GIMcom();

	bool   wrtieSatet;
	DWORD  errCode;

	float roll;
	float pitch;
	float yaw;
	float roll_rate;
	float pitch_rate;
	float yaw_rate;

	
	UINT16 roll_ENCODE;
	UINT16 pitch_ENCODE;
	UINT16 yaw_ENCODE;
	UINT16 roll_rate_ENCODE;
	UINT16 pitch_rate_ENCODE;
	UINT16 yaw_rate_ENCODE;


	void Ini();
	void Close();
	virtual void send_to_gimbal();
	unsigned char checksum(unsigned char *buff, unsigned int length);
	void Transfer(unsigned char *buff, unsigned char length);

};

#endif