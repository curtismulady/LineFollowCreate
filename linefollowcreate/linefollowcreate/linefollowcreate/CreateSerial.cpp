#include "stdafx.h"
#include <windows.h>
#include <stdio.h>
#include "CreateSerial.h"
#include <tchar.h>

CreateSerial::CreateSerial(int comnum)
{

	swprintf(COMString,_T("\\\\.\\COM%d"), comnum);
	printf("\nOpening COMx: Serial Port . . . ");
	hSerial = CreateFile((COMString), GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	// Check for file open errors
	if (hSerial == INVALID_HANDLE_VALUE){
		printf("file open errors\n");
	}
	// Modify Com Port settings (i.e. Baud Rate, #bits, parity etc)
	if(!ModifyCommSettings (hSerial)){
		printf("com port settings errors\n");
	}
	// Set Communication event mask for WaitCommEvent for rxchar (receive character) in buffer
	//SetCommMask(hSerial, EV_RXCHAR | EV_ERR);
	printf("done.\n");

}

CreateSerial::~CreateSerial()
{
	CloseHandle(hSerial);
}

void CreateSerial::StartRobot(void)
{
	data_out[0] = 128;
	DWORD len = 1;
	if (!WriteFile(hSerial, data_out, len, &len, NULL)){
		printf("\rfile write errors\n");
	}
}

void CreateSerial::SetMode_Full(void)
{
	data_out[0] = 132;
	DWORD len = 1;
	if (!WriteFile(hSerial, data_out, len, &len, NULL)){
		printf("\rfile write errors\n");
	}
}

int  CreateSerial::GetBumperStatus(void)
{
	data_out[0] = 7;
	DWORD len = 1;
	if (!WriteFile(hSerial, data_out, len, &len, NULL)){
		printf("\rfile write errors\n");
	}
	printf("Der datas sent outz! Waiteeng.\n");
	DWORD dwMask;
	WaitCommEvent(hSerial,&dwMask,0);
	printf("Finish'd da waiteeng.\n");
	DWORD cbytes_in = 1;
	while(cbytes_in)
	{
		if(ReadFile(hSerial,data_in,79,&cbytes_in,NULL)){
			if(!cbytes_in) break;
			printf("%x\n",data_in[0]);
			return(data_in[0]);
		}
	}

}

void CreateSerial::DriveMotor(signed int vel, signed int radius, unsigned char flags)
{
	data_out[0] = 137;
	data_out[1] = (unsigned char)((vel&0xFF00)>>8);
	data_out[2] = (unsigned char)(vel&0xFF);
	data_out[3] = (unsigned char)((radius&0xFF00)>>8);
	data_out[4] = (unsigned char)(radius&0xFF);
	DWORD len = 5;
	if (!WriteFile(hSerial, data_out, len, &len, NULL)){
		printf("\rfile write errors\n");
	}
}

void CreateSerial::DriveMotorDirect(signed int right_vel, signed int left_vel)
{
	data_out[0] = 145;
	data_out[1] = (unsigned char)((right_vel&0xFF00)>>8);
	data_out[2] = (unsigned char)(right_vel&0xFF);
	data_out[3] = (unsigned char)((left_vel&0xFF00)>>8);
	data_out[4] = (unsigned char)(left_vel&0xFF);
	DWORD len = 5;
	if (!WriteFile(hSerial, data_out, len, &len, NULL)){
		printf("\rfile write errors\n");
	}
}

void CreateSerial::Beep(void)
{
	data_out[0] = 0x8C;
	data_out[1]=0x00;
	data_out[2]=0x01;
	data_out[3]=0x45;
	data_out[4]=0x40;
	data_out[5] = 0x8D;
	data_out[6] = 	0x00;
	
	DWORD len = 7;
	if (!WriteFile(hSerial, data_out, len, &len, NULL)){
		printf("\rfile write errors\n");
	}
}

void CreateSerial::QuickBeep(void)
{
	data_out[0] = 0x8C;
	data_out[1]=0x00;
	data_out[2]=0x01;
	data_out[3]=0x65;
	data_out[4]=0x08;
	data_out[5] = 0x8D;
	data_out[6] = 	0x00;
	
	DWORD len = 7;
	if (!WriteFile(hSerial, data_out, len, &len, NULL)){
		printf("\rfile write errors\n");
	}
}

void CreateSerial::DoubleBeep(void)
{
	data_out[0] = 0x8C;
	data_out[1]=0x00;
	data_out[2]=0x03;
	data_out[3]=0x35;
	data_out[4]=0x10;
	data_out[5]=0x0;
	data_out[6]=0x10;
	data_out[7]=0x35;
	data_out[8]=0x10;
	data_out[9] = 0x8D;
	data_out[10] = 0x00;
	
	DWORD len = 11;
	if (!WriteFile(hSerial, data_out, len, &len, NULL)){
		printf("\rfile write errors\n");
	}
}

void CreateSerial::TripleBeep(void)
{
	data_out[0] = 0x8C;
	data_out[1]=0x00;
	data_out[2]=0x05;
	data_out[3]=0x45;
	data_out[4]=0x08;
	data_out[5]=0x0;
	data_out[6]=0x08;
	data_out[7]=0x45;
	data_out[8]=0x08;
	data_out[9]=0x0;
	data_out[10]=0x08;
	data_out[11]=0x45;
	data_out[12]=0x08;
	data_out[13] = 0x8D;
	data_out[14] = 0x00;
	
	DWORD len = 15;
	if (!WriteFile(hSerial, data_out, len, &len, NULL)){
		printf("\rfile write errors\n");
	}
}

void CreateSerial::LightLED(bool play_led_state, bool advance_led_state, unsigned char power_led_color, unsigned char power_led_intensity)
{
	data_out[0] = 139;
	data_out[1] = 0 | ((play_led_state&1)<<1) | ((advance_led_state&1)<<3);
	data_out[2] = power_led_color;
	data_out[3] = power_led_intensity;
	DWORD len = 4;
	if (!WriteFile(hSerial, data_out, len, &len, NULL)){
		printf("\rfile write errors\n");
	}
}


BOOL   CreateSerial::ModifyCommSettings (HANDLE hComPort)
{
    COMMTIMEOUTS ctos;
	DCB PortDCB;
	// Initialize the DCBlength member. 
	PortDCB.DCBlength = sizeof (DCB); 
	// Get the default serial port settings DCB information.
	GetCommState (hSerial, &PortDCB);
	// Change the common DCB structure settings to modify serial port settings.
	PortDCB.BaudRate = 57600;              // Current baud 
	PortDCB.fBinary = TRUE;               // Binary mode; no EOF check 
	PortDCB.fParity = TRUE;               // Enable parity checking 
	PortDCB.fOutxCtsFlow = FALSE;         // No CTS output flow control 
	PortDCB.fOutxDsrFlow = FALSE;         // No DSR output flow control 
	PortDCB.fDtrControl = DTR_CONTROL_ENABLE; // DTR flow control type 
	PortDCB.fDsrSensitivity = FALSE;      // DSR sensitivity 
	PortDCB.fTXContinueOnXoff = TRUE;     // XOFF continues Tx 
	PortDCB.fOutX = FALSE;                // No XON/XOFF out flow control 
	PortDCB.fInX = FALSE;                 // No XON/XOFF in flow control 
	PortDCB.fErrorChar = FALSE;           // Disable error replacement 
	PortDCB.fNull = FALSE;                // Disable null stripping 
	PortDCB.fRtsControl = RTS_CONTROL_ENABLE; // RTS flow control 
	PortDCB.fAbortOnError = FALSE;        // Do not abort reads/writes on error
	PortDCB.ByteSize = 8;                 // Number of bits/byte, 4-8 
	PortDCB.Parity = NOPARITY;            // 0-4=no,odd,even,mark,space 
	PortDCB.StopBits = ONESTOPBIT;        // 0,1,2 = 1, 1.5, 2 
	// Configure the port settings according to the new specifications  
	// of the DCB structure.
	if (!SetCommState (hSerial, &PortDCB)){
	    printf("Unable to configure the serial port"); 
		Sleep(4000);
		return false;
		}
	// Set read time outs 
	ctos.ReadIntervalTimeout = MAXDWORD;
	ctos.ReadTotalTimeoutMultiplier = MAXDWORD;
	ctos.ReadTotalTimeoutConstant = 1;
	ctos.WriteTotalTimeoutMultiplier = 0;
	ctos.WriteTotalTimeoutConstant = 0;
	if(!SetCommTimeouts(hSerial, &ctos)){
	    printf("Unable to configure the serial port"); 
		Sleep(4000);
		return false;
	}
	return true;
}