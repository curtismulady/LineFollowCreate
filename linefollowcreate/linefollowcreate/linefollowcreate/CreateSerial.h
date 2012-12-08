
#ifndef CREATE_SERIAL_H
#define CREATE_SERIAL_H

class CreateSerial
{

public:
	CreateSerial(int comnum);
	~CreateSerial();
	void StartRobot(void);
	void SetMode_Full(void);
	void DriveMotor(signed int vel, signed int radius, unsigned char flags);
	void DriveMotorDirect(signed int right_vel, signed int left_vel);
	void LightLED(bool play_led_state, bool advance_led_state, unsigned char power_led_color, unsigned char power_led_intensity);
	int  GetBumperStatus(void);
	void Beep(void);
	void TripleBeep(void);
	void DoubleBeep(void);
	void QuickBeep(void);

private:
	BOOL ModifyCommSettings (HANDLE hComPort);
	wchar_t COMString[100];
	char data_out[100];
	char data_in[100];
	HANDLE hSerial;
	DWORD dwMask;

};


#endif
