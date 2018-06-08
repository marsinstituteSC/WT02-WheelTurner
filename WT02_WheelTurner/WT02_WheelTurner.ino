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



//HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK
//det er kasnkje nødvendig å invertere vinkler fra venstre til høyre styreservo
//HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK HUSK



//includes
//_________________________________________________________________________________________________________________
#include <SPI.h>
#include <Servo.h>
#include <math.h>
#include <limits.h>
#include "AutoPID\AutoPID.h"
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

#define FRAMETYPE		0	//frame type of CAN bus. 0=stadard, 1=extended 

#define SS02_POS_X		600 //Wheel rotaters x position relative to rovers center point in mm
#define SS02_POS_Y		400 //Wheel rotaters x position relative to rovers center point in mm

#define MAX_ANGLE		110	//Maximum alowed angle for steering servo (turning past this may damage module)
#define MIN_ANGLE		-110//Minimum alowed angle for steering servo (turning past this may damage module)
#define SERVO_ZERO		1580//Value at which servo potmeter is centered, must be calibrated
#define MAX_SPEED		0.2 //Maximum speed factor from the max speed of the servo motor.

#define PID_INIT_KP		10
#define PID_INIT_KI		0
#define PID_INIT_KD		0

#define POTMINUS90DEG	820 //potensiometer value when steering servo at +90deg
#define POTPLUS90DEG	170


//message buffers
//_________________________________________________________________________________________________________________
byte buf_drive[6];
byte buf_stat[2];
byte buf_angl[2];
byte buf_pid[8];
byte buf_cal[4];



//global variables
//_________________________________________________________________________________________________________________
bool pastMaxAngle;			//Servo past end points
bool positionReached;		//Servo within +/-5 deg of set point
int potMinus90deg;			//Potensiometer analog value at -90deg
int potPlus90deg;			//Potensiometer analog value at +90deg

byte status;				//status byte
byte alive;					//Wachdog (increment by 1/second)

long radius;				//turning radius in mm (from center of rover)

bool pid_man;				//PID control auto/man, 0 = auto, 1 = manual
double pid_p, pid_i, pid_d;
double pid_SP, pid_PV, pid_CV; //PID Setpoint and process value (in degrees) and control value
char pid_CV_man;




//timer variables
//_________________________________________________________________________________________________________________
unsigned long t_cantransmit_prev;
int t_cantransmit_interval = 1000;



//objects
//_________________________________________________________________________________________________________________
MCP_CAN CAN(CAN_CS_PIN);
AutoPID myPID(&pid_PV, &pid_SP, &pid_CV, -500*MAX_SPEED, 500*MAX_SPEED, PID_INIT_KP, PID_INIT_KI, PID_INIT_KD);
//AutoPID myPID(&pid_PV, &pid_SP, &pid_CV, PID_MIN_CV, PID_MAX_CV, PID_INIT_KP, PID_INIT_KI, PID_INIT_KD);
Servo servo;



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

	//init pins
	pinMode(PIN_POT, INPUT);
	pinMode(PIN_SERVO, OUTPUT);

	//initialize servo
	Serial.print("Initializing servo...\t");
	servo.attach(PIN_SERVO);
	Serial.println("[DONE]");

	//initialize PID
	myPID.setTimeStep(100);

	//initialize variables
	status = 0x01;
	pid_man = false;
	pid_CV_man = 0;
	potMinus90deg = POTMINUS90DEG;
	potPlus90deg = POTPLUS90DEG;
	radius = LONG_MIN;



}



//Program loop
//_________________________________________________________________________________________________________________
void loop()
{

	//CAN Recieve
	//_____________________________________________________________________________________________________________
	if (CAN_MSGAVAIL == CAN.checkReceive())
	{
		//Read data
		unsigned long ID;
		byte rxlen = 0;
		byte rxbuf[8];
		CAN.readMsgBufID(&ID, &rxlen, rxbuf);

		//handle data
		switch (ID)
		{
		case GLOB_DRIVE:
			radius = rxbuf[2] << 24 | rxbuf[3] << 16 | rxbuf[4] << 8 | rxbuf[5];
			break;
		case STEER_XX_PID:
			pid_man = rxbuf[0] & 0x01;
			pid_p = (rxbuf[1] << 8 | rxbuf[2]) / 100.0;
			pid_i = (rxbuf[3] << 8 | rxbuf[4]) / 100.0;
			pid_d = (rxbuf[5] << 8 | rxbuf[6]) / 100.0;
			pid_CV_man = rxbuf[7];

			myPID.setGains(pid_p, pid_i, pid_d);
			break;
		case STEED_XX_CAL:
			potMinus90deg = (rxbuf[0] << 8 | rxbuf[1]);
			potMinus90deg = (rxbuf[2] << 8 | rxbuf[3]);
			break;
		default:
			break;
		}

		
	}



	//Position computations and handling
	//_____________________________________________________________________________________________________________

	//calculate angle SP from desired turning radius
	pid_SP = radiusToDeg(radius);

	//Read angle of steering servo
	pid_PV = dmap(analogRead(PIN_POT), potMinus90deg, potPlus90deg, -90, 90);

	//Handle angle out of bounds
	if ((pid_PV < MIN_ANGLE || pid_PV > MAX_ANGLE) && servo.attached())
	{
		pastMaxAngle = true;
		servo.detach(); //if servo past max Angle, detach servo for safety

		Serial.println("Servo detached");
	}
	//handle angle back within bounds (*0.2 for hysteresis)
	else if ((pid_PV > MIN_ANGLE*0.2 && pid_PV < MAX_ANGLE*0.2) && !servo.attached())
	{
		servo.attach(PIN_SERVO);
		Serial.println("Servo attached");
	}
	
	positionReached = (pid_PV >= pid_SP - 5 && pid_PV <= pid_SP + 5);
	
	//set status bits
	bitWrite(buf_stat[0], 1, positionReached);
	bitWrite(buf_stat[0], 2, pastMaxAngle);




	//PID Position control
	//_____________________________________________________________________________________________________________

	//AUTO
	if (!pid_man)
		myPID.run();
	//MANUAL
	else
		pid_CV = (double)(500 * pid_CV_man / 100.0);

	servo.writeMicroseconds(SERVO_ZERO + pid_CV);



	

	//CAN Transmit 
	//_____________________________________________________________________________________________________________
	if (millis() - t_cantransmit_prev > t_cantransmit_interval)
	{
		CAN.sendMsgBuf(STEER_XX_STAT, FRAMETYPE, 2, buf_stat);
		CAN.sendMsgBuf(STEER_XX_ANGL, FRAMETYPE, 2, buf_angl);
		CAN.sendMsgBuf(STEER_XX_STAT, FRAMETYPE, 8, buf_pid);
		CAN.sendMsgBuf(STEER_XX_STAT, FRAMETYPE, 4, buf_cal);

		Serial.print("r: ");
		Serial.print(radius);
		Serial.print(" a: ");
		Serial.print(analogRead(PIN_POT));
		Serial.print(" pos r: ");
		Serial.print(positionReached);
		Serial.print(" SP: ");
		Serial.print(pid_SP);
		Serial.print(" PV: ");
		Serial.print(pid_PV);
		Serial.print(" CV: ");
		Serial.print(pid_CV);
		Serial.print(" A/M: ");
		Serial.print(pid_man);
		Serial.print(" HB: ");
		Serial.print(alive);
		Serial.println();

		t_cantransmit_prev = millis();
	}



	//runtime variables
	alive = (millis() / 1000) % 256;

}

//radiusToDeg() converts the desired rover turning radius into desired angle using Ackerman steering formulas
double radiusToDeg(long radius)
{
	double tan_phi = SS02_POS_X / (double)(radius - SS02_POS_Y);
	double phi = atan(tan_phi) / (double)(2*PI)*360;

	return phi;
}

double dmap(double x, double in_min, double in_max, double out_min, double out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
