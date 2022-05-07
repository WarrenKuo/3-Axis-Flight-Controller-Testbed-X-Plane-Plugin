#include "XPlane_GIMcom.h"

XPlane_GIMcom::XPlane_GIMcom(): wrtieSatet(false),
								errCode(0),
								roll(0),
								pitch(0),
								yaw(0),
								roll_rate(0),
								pitch_rate(0),
								yaw_rate(0),
								roll_ENCODE(0),
								pitch_ENCODE(0),
								yaw_ENCODE(0),
								roll_rate_ENCODE(0),
								pitch_rate_ENCODE(0),
								yaw_rate_ENCODE(0)
{
	memset(TxBuffer, 0, txBuffSize);
}

//===========================//
void  XPlane_GIMcom::Ini()
{
	hCom = CreateFile(	TEXT(COM_PORT),
						GENERIC_READ | GENERIC_WRITE,
						0,      //  must be opened with exclusive-access
						NULL,   //  default security attributes
						OPEN_EXISTING, //  must use OPEN_EXISTING
						0,      //  not overlapped I/O
						NULL); //  hTemplate must be NULL for comm devices

	errCode = GetLastError();
	SecureZeroMemory(&dcb, sizeof(DCB));

	dcb.DCBlength = sizeof(DCB);
	dcb.BaudRate  = CBR_115200;     //  baud rate
	dcb.ByteSize  = 8;             //  data size, xmit and rcv
	dcb.Parity    = NOPARITY;      //  parity bit
	dcb.StopBits  = ONESTOPBIT;    //  stop bit
}

//===========================//
void  XPlane_GIMcom::Close()
{
	CloseHandle(hCom);
}

//===========================//
void XPlane_GIMcom::Transfer(unsigned char *buff, unsigned char length)  {  
	wrtieSatet = WriteFile(	hCom,           // open file handle
							TxBuffer,      // start of data to write
							PackageSize,  // number of bytes to write
							&dwBytesWritten, // number of bytes that were written
							NULL);            // no overlapped structure
}

//===========================//
void XPlane_GIMcom::send_to_gimbal()
{
	
  int index = 3;
  
  TxBuffer[0]  = '$';
  TxBuffer[1]  = 'G';
  TxBuffer[2]  = 'I';
  TxBuffer[3]  = 'M';
  
  roll_ENCODE  = (int)(roll*57.2957795  /0.0054933);						// 180/PI=57.295779513
  pitch_ENCODE = (int)(pitch*57.2957795 /0.0027467);						// 180/PI=57.295779513
  yaw_ENCODE   = (unsigned int)( yaw>0? yaw*57.2957795/0.0054933 :
                               (yaw+6.2831853)*57.2957795/0.0054933 );       
																			// 180/PI=57.295779513,   2*PI=6.283185307
  TxBuffer[index + 1]  = roll_ENCODE;
  TxBuffer[index + 2]  = roll_ENCODE >>8;
  TxBuffer[index + 3]  = pitch_ENCODE;
  TxBuffer[index + 4]  = pitch_ENCODE >>8;
  TxBuffer[index + 5]  = yaw_ENCODE;
  TxBuffer[index + 6]  = yaw_ENCODE >>8;
  
  TxBuffer[index + 13] = checksum(TxBuffer + index + 1, 12);
  TxBuffer[index + 14] = 0x0A;
  TxBuffer[index + 15] = 0x0D;
  
  Transfer(TxBuffer, index + 16);

}

//===========================//
unsigned char XPlane_GIMcom::checksum(unsigned char *buff, unsigned int length)
{
    unsigned char cs=0;
    int i;
    for (i=0;i<length;i++) {
      cs^=buff[i];
    }
    return cs;
}
