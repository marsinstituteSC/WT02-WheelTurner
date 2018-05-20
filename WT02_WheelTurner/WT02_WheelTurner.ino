/*
This code controls the position of each wheel on the Viking 3 - T1 rover using the SS02 steering servo module.

The module consits of a Hitec HS-755MG servo motor, a potensiometer to read the position, 
an Arduino pro micro to controll the module and a MCP2515 CAN bus breakout board for the CAN bus communication.

The position of the wheels are set by the decired turning radius for the rover, based on the wheels position 
relative to the rovers center point. The desired radius is read from the CAN bus and used as set point (SP) 
through the modules PID controller. The actual angle of the wheel is sendt back to the motherboard as feedback.
The module also sends status and error messages to enable the rovers high level of self diagnosis.

NOTE: Remember to change the position and CAN message variables to the appropriate value for this steering servo



Author: Rein Åsmund Torsvik, MISC 2018
*/



//includes
//_________________________________________________________________________________________________________________
#include <SPI.h>
#include <Servo.h>
#include <Timer.h>
#include "PID\PID_v1.h"
#include "mcp_can.h"



//constants
//_________________________________________________________________________________________________________________
#define CAN_CS_PIN		10
#define PIN_SERVO		9
#define PIN_POT			A0

#define GLOB_DRIVE		0x100
#define STEER_XX_STAT	0x310
#define STEER_XX_ANGL	0x311
#define STEER_XX_PID	0x312
#define STEED_XX_CAL	0x313

#define SS02_POS_X		600 //Wheel rotaters x position relative to rovers center point in mm
#define SS02_POS_Y		400 //Wheel rotaters x position relative to rovers center point in mm

#define MAX_ANGLE		100	//Maximum alowed angle for steering servo (turning past this may damage module)
#define MIN_ANGLE		-100//Minimum alowed angle for steering servo (turning past this may damage module)



//message buffers
//_________________________________________________________________________________________________________________
byte buf_drive[6];
byte buf_stat[2];
byte buf_angl[2];
byte buf_pid[8];
byte buf_cal[4];


//objects
//_________________________________________________________________________________________________________________
MCP_CAN CAN(CAN_CS_PIN);
PID myPID(&pid_PV, &pid_CV, &pid_SP, 2, 5, 1, DIRECT);
Servo servo;


//global variables
//_________________________________________________________________________________________________________________
bool pastMaxAngle;			//Servo past end points
bool positionReached;		//Servo within +/-5 deg of set point
int POTMINUS90DEG;			//Potensiometer analog value at -90deg
int POTPLUS90DEG;			//Potensiometer analog value at +90deg

byte status;				//status byte
byte alive;					//Wachdog (increment by 1/second)

long radius;				//turning radius in mm (from center of rover)

bool pid_man;				//PID control auto/man, 0 = auto, 1 = manual
double pid_p, pid_i, pid_d;
double pid_SP, pid_PV, pid_CV; //PID Setpoint and process value (in degrees) and control value




//timer variables
//_________________________________________________________________________________________________________________
unsigned long t_cantransmit_prev;
int t_cantransmit_interval = 1000;


//Setup
//_________________________________________________________________________________________________________________
void setup()
{

	Serial.begin(9600);
	Serial.println("Initializing wheel rotater:\n");

	//Initialize CAN bus with baud rate 500kbps and clockset 8MHz
	Serial.print("Initializing CAN bus...\t");
	if (CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz))
		Serial.println("[FAIL]");
	else
		Serial.println("[SUCCESS]");

	//initialize servo
	Serial.print("Initializing servo...\t");
	servo.attach(PIN_SERVO);
	Serial.println("[DONE]");

	//initialize variables
	status = 0x01;
	pid_man = false;
	POTMINUS90DEG = 100;
	POTPLUS90DEG = 924;



}



//Program loop
//_________________________________________________________________________________________________________________
void loop()
{
	//calculate angle SP from desired turning radius
	pid_SP = radiusToDeg(radius);

	//Read angle of steering servo
	pid_PV = dmap(analogRead(PIN_POT), POTMINUS90DEG, POTPLUS90DEG, -90, 90);

	//set status bits for angles
	pastMaxAngle = (pid_PV < MIN_ANGLE || pid_PV > MAX_ANGLE);
	positionReached = (pid_PV >= pid_SP - 5 || pid_PV <= pid_SP + 5);

	bitWrite(buf_stat[0], 1, positionReached);
	bitWrite(buf_stat[0], 2, pastMaxAngle);



	//Recieve CAN messages
	if (CAN_MSGAVAIL == CAN.checkReceive())
	{
		//Read data
		CAN.readMsgBuf(&len, rxbuf);

		//Print data to serial
		Serial.print("Recieved ID: ");
		Serial.println(CAN.getCanId(), HEX);

		for (int i = 0; i<len; i++)
		{
			Serial.print(txbuf[i], HEX);
			Serial.print("\t");
		}
		Serial.println();
	}

	//Send CAN messages
	unsigned long id = 0x100;	//Identifier
	byte frametype = 0;			//0=stadard, 1=extended
	byte len = 8;				//Data length
	byte txbuf[8] = { 0x11, 0x22 ,0x33, 0x44, 0x55, 0x66, 0x77, 0xAA };

	float fvalue = 3.14;
	byte *b = (byte *)&fvalue;

	txbuf[0] = b[0];
	txbuf[1] = b[1];
	txbuf[2] = b[2];
	txbuf[3] = b[3];

	int ivalue = 1337;

	txbuf[4] = (byte)(ivalue >> 8);
	txbuf[5] = (byte)(ivalue >> 0);




	CAN.sendMsgBuf(id, frametype, len, txbuf);

	byte fbytes[4];
	fbytes[0] = rxbuf[0];
	fbytes[1] = rxbuf[1];
	fbytes[2] = rxbuf[2];
	fbytes[3] = rxbuf[3];

	float fvalue = *((float*)(fbytes));

	int ivalue = (rxbuf[4] << 8) | (rxbuf[5] << 0);

	//transmit can messages



	//runtime variables
	alive = (millis() / 1000) % 256;

}

//radiusToDeg() converts the desired rover turning radius into desired angle using Ackerman steering formulas
//including some conditions for logical positioning of wheels and rotating around your own axis. 
double radiusToDeg(long radius)
{

}

double dmap(double x, double in_min, double in_max, double out_min, double out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
