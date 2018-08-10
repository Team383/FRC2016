/* ---OPERATOR CONTROLS---
 * Shooter angle up: DPAD UP
 * Shooter angle down: DPAD DOWN
 * Shooter angle preset: B
 * Shooter trigger: A
 * Shooter on: RB
 * Shooter reverse: X
 * Shooter off: LB
 * Climber up: LS
 * Climber down: RS
 * Climber: Start + Back
 * ------------------------
 */

#include "WPILib.h"
#include "CAMTalon.h"

//Auto speed constants
const float SPEED_OFFSET = 0.15;
const float SPEED_PORTCULLIS = -1.0 * (0 + SPEED_OFFSET);
const float SPEED_CHEVAL_DE_FRISE = -1.0 * (0 + SPEED_OFFSET);
const float SPEED_RAMPARTS = -1.0 * (0.6 + SPEED_OFFSET);
const float SPEED_MOAT = -1.0 * (0.6 + SPEED_OFFSET);
const float SPEED_DRAWBRIDGE = -1.0 * (0 + SPEED_OFFSET);
const float SPEED_SALLY_PORT = -1.0 * (0 + SPEED_OFFSET);
const float SPEED_ROCK_WALL = -1.0 * (0.45 + SPEED_OFFSET);
const float SPEED_FAST_ROCK_WALL = -1.0 * (0.7 + SPEED_OFFSET); //Speed boost
const float SPEED_ROUGH_TERRAIN = -1.0 * (0.6 + SPEED_OFFSET);
const float SPEED_RAMP = -1.0 * (0.4 + SPEED_OFFSET);
const float SPEED_LOW_BAR = -1.0 * (0.4 + SPEED_OFFSET);

//Xbox DPAD
const int D_UP = 0;
const int D_UP_RIGHT = 45;
const int D_RIGHT = 90;
const int D_DOWN_RIGHT = 135;
const int D_DOWN = 180;
const int D_DOWN_LEFT = 225;
const int D_LEFT = 270;
const int D_UP_LEFT = 315;

class Robot: public IterativeRobot
{

	RobotDrive myRobot; // robot drive system

	Joystick stickMain;
	Joystick stickRot;
	Joystick stickXbox;

	CANTalon shootAngle;

	Talon shootL;
	Talon shootR;
	Talon climberL;
	Talon climberR;
	Talon climberAdjust;

	CANTalon FL;
	CANTalon RL;
	CANTalon FR;
	CANTalon RR;

	Servo shootTrig;

	DigitalInput autoSwitch0;
	DigitalInput autoSwitch1;
	DigitalInput autoSwitch2;
	DigitalInput autoSwitch3;
	DigitalInput lsShootAngleUp;
	DigitalInput lsShootAngleDown;

	AnalogGyro gyro;

	AnalogInput ultra;

	int autoCounter, binToDec, shooterOn;
	float autoSpeed, autoRot, mainY, rotX, climbSpeed, encPosRaw, encoderAngle, encPosMax, encPosMin, angleSpeed;
	bool angleManual, xboxA, xboxB, xboxX, xboxY, xboxLB, xboxRB, xboxBack, xboxStart, xboxLS, xboxRS, lsAngleMin, lsAngleMax, encCalib;

public:
	Robot() :
		myRobot(FL, RL, FR, RR),

		stickMain(0),
		stickRot(1),
		stickXbox(2),

		shootAngle(2),

		shootL(2),
		shootR(0),
		climberL(1),
		climberR(3),
		climberAdjust(4),

		FL(22),
		RL(21),
		FR(20),
		RR(23),

		shootTrig(5),

		autoSwitch0(0),
		autoSwitch1(1),
		autoSwitch2(2),
		autoSwitch3(3),
		lsShootAngleUp(5),
		lsShootAngleDown(4),

		gyro(1),

		ultra(0)
{
		myRobot.SetExpiration(0.1);

		myRobot.SetSafetyEnabled(false);

		myRobot.SetInvertedMotor(myRobot.kFrontLeftMotor, true);
		myRobot.SetInvertedMotor(myRobot.kRearRightMotor, true);
		myRobot.SetInvertedMotor(myRobot.kFrontRightMotor, true);
		myRobot.SetInvertedMotor(myRobot.kRearLeftMotor, true);
}

private:
	void AutonomousInit()
	{
		encPosMin = -999;
		encPosMax = -999;

		gyro.Reset();

		binToDec = (!autoSwitch0.Get() * 1) + (!autoSwitch1.Get() * 2) + (!autoSwitch2.Get() * 4) + (!autoSwitch3.Get() * 8);

		autoCounter = 0;
		autoSpeed = 0;
		autoRot = 0;

	}

	void AutonomousPeriodic()
	{
		lsAngleMax = !lsShootAngleUp.Get();
		lsAngleMin = !lsShootAngleDown.Get();

		//------ENCODER RECALIBRATION------
		encPosRaw = shootAngle.GetPosition();

		if(encPosMin == -999){
			shootAngle.Set(-0.7);
			if(lsAngleMin){
				shootAngle.SetPosition(100);
				encPosMin = encPosRaw;
			}
		}
		else if(encPosMax == -999){
			shootAngle.Set(0.7);
			if(lsAngleMax){
				encPosMax = encPosRaw;
			}
		}
		else{
			if(binToDec == 8){
				shootAngle.Set(5 * (((encPosMin - encPosRaw) / (encPosMax))) );
			}
			else{
				shootAngle.Set(5 * ((((encPosMax * 61 / 100) - encPosRaw) / (encPosMax))) );
			}
		}
		//------------------
		switch(binToDec){
		case 8:		//PORTCULLIS
			SmartDashboard::PutString("Auto Mode", "PORTCULLIS");

			break;
		case 1:		//CHEVAL DE FRISE
			SmartDashboard::PutString("Auto Mode", "CHEVAL DE FRISE");

			break;
		case 2:		//RAMPARTS
			SmartDashboard::PutString("Auto Mode", "RAMPARTS");

			if (autoCounter < 3){
				autoSpeed = SPEED_RAMPARTS;
			}
			else if (autoCounter >= 225){
				autoSpeed = 0;
			}

			autoRot = 0;
			myRobot.ArcadeDrive(autoSpeed, autoRot);

			break;
		case 3:		//MOAT
			SmartDashboard::PutString("Auto Mode", "MOAT");

			if (autoCounter < 3){
				autoSpeed = SPEED_MOAT;
			}
			else if (autoCounter >= 250){
				autoSpeed = 0;
			}

			autoRot = 0;
			myRobot.ArcadeDrive(autoSpeed, autoRot);

			break;
		case 4:		//DRAWBRIDGE
			SmartDashboard::PutString("Auto Mode", "DRAWBRIDGE");

			break;
		case 5:		//SALLY PORT
			SmartDashboard::PutString("Auto Mode", "SALLY PORT");

			break;
		case 6:		//ROCK WALL
			SmartDashboard::PutString("Auto Mode", "ROCK WALL");

			if (autoCounter < 3){
				autoSpeed = SPEED_ROCK_WALL;
			}
			else if (autoCounter >= 160 && autoCounter < 250){
				autoSpeed = SPEED_FAST_ROCK_WALL;
			}
			else if (autoCounter >= 250 && autoCounter < 275){
				autoSpeed = SPEED_ROCK_WALL;
			}
			else if (autoCounter >= 250){
				autoSpeed = 0;
			}

			autoRot = 0;
			myRobot.ArcadeDrive(autoSpeed, autoRot);

			break;
		case 7:		//ROUGH TERRAIN
			SmartDashboard::PutString("Auto Mode", "ROUGH TERRAIN");
			if (autoCounter < 3){
				autoSpeed = SPEED_ROUGH_TERRAIN;
			}
			else if (autoCounter >= 375){
				autoSpeed = 0;
			}

			autoRot = 0;
			myRobot.ArcadeDrive(autoSpeed, autoRot);

			break;
		case 0:		//LOW BAR
			SmartDashboard::PutString("Auto Mode", "LOW BAR");

			if (autoCounter < 3){
				autoSpeed = 0;
			}
			else if (autoCounter >= 250 && autoCounter <= 475){
				autoSpeed = SPEED_LOW_BAR;
			}
			else if(autoCounter > 475){
				autoSpeed = 0;
			}

			autoRot = 0;
			myRobot.ArcadeDrive(autoSpeed, autoRot);


			break;
		case 9:	//RAMP
			SmartDashboard::PutString("Auto Mode", "RAMP");

			if (autoCounter < 3){
				autoSpeed = SPEED_RAMP;
			}
			else if (autoCounter >= 250){
				autoSpeed = 0;
			}

			autoRot = 0;
			myRobot.ArcadeDrive(autoSpeed, autoRot);


			break;
		case 10:		//TESTING
			SmartDashboard::PutString("Auto Mode", "TEST");

			break;
		default:
			SmartDashboard::PutString("Auto Mode", "DEFAULT");

			break;
		}

		autoCounter++;


		SmartDashboard::PutNumber("encoderRaw", encPosRaw);
		SmartDashboard::PutNumber("encPosMin", encPosMin);
		SmartDashboard::PutNumber("encPosMax", encPosMax);
	}

	void TeleopInit()
	{
		encCalib = false;
		gyro.Reset();
	}

	void TeleopPeriodic()
	{
		binToDec = (!autoSwitch0.Get() * 1) + (!autoSwitch1.Get() * 2) + (!autoSwitch2.Get() * 4) + (!autoSwitch3.Get() * 8);
		SmartDashboard::PutNumber("auto mode", binToDec);

		//------XBOX BUTTONS------
		xboxA = stickXbox.GetRawButton(1);
		xboxB = stickXbox.GetRawButton(2);
		xboxX = stickXbox.GetRawButton(3);
		xboxY = stickXbox.GetRawButton(4);
		xboxLB = stickXbox.GetRawButton(5);
		xboxRB = stickXbox.GetRawButton(6);
		xboxBack = stickXbox.GetRawButton(7);
		xboxStart = stickXbox.GetRawButton(8);
		xboxLS = stickXbox.GetRawButton(9);
		xboxRS = stickXbox.GetRawButton(10);
		//------------------

		//------LIMIT SWITCHES------
		lsAngleMax = !lsShootAngleUp.Get();
		lsAngleMin = !lsShootAngleDown.Get();
		//------------------

		//------ENCODER RECALIBRATION------
		if(stickMain.GetRawButton(8)){
			encCalib = true;
			encPosMin = -999;
			encPosMax = -999;
		}
		if(encCalib){
			if(encPosMin == -999){
				shootAngle.Set(-0.7);
				if(lsAngleMin){
					shootAngle.SetPosition(100);
					encPosMin = encPosRaw;
				}
			}
			else if(encPosMax == -999){
				shootAngle.Set(0.7);
				if(lsAngleMax){
					encPosMax = encPosRaw;
					encCalib = false;
				}
			}
			else{
				shootAngle.Set(5 * ((((encPosMax * 61 / 100) - encPosRaw) / (encPosMax))) );
			}

		}
		//------------------


		//------ANGLE------
		if(!encCalib){
			if(xboxB){
				angleManual = false;
			}

			if(stickXbox.GetPOV() == D_UP_LEFT || stickXbox.GetPOV() == D_UP || stickXbox.GetPOV() == D_UP_RIGHT){
				angleManual = true;
				if(lsShootAngleUp.Get()){
					shootAngle.Set(0.75);
				}
				else{
					shootAngle.Set(0);
				}
			}
			else if(stickXbox.GetPOV() == D_DOWN_LEFT || stickXbox.GetPOV() == D_DOWN || stickXbox.GetPOV() == D_DOWN_RIGHT){
				angleManual = true;
				if(lsShootAngleDown.Get()){
					shootAngle.Set(-0.75);
				}
				else{
					shootAngle.Set(0);
				}
			}
			else if(angleManual){
				shootAngle.Set(0);
			}

			if(!angleManual){
				shootAngle.Set(5 * ((((encPosMax * 61 / 100) - encPosRaw) / (encPosMax))) );
			}

			if((encPosRaw >= (encPosMax * 61 / 100) - 50) && (encPosRaw <= (encPosMax * 61 / 100) + 50)){
				angleManual = true;
			}
		}
		//------------------

		//------SHOOTER------
		if(xboxRB){ //Shooting
			shooterOn = 1;
		}
		else if(xboxX){ //Feeding
			shooterOn = -1;
		}
		else if(xboxY){
			shooterOn = 2;
		}

		if(xboxLB){ //Shooter off
			shooterOn = 0;
		}

		if(shooterOn == 1){ // High goals
			shootL.Set(1);
			shootR.Set(-1);
		}
		else if(shooterOn == -1){ // sucky sucky
			shootL.Set(-0.7);
			shootR.Set(0.7);
		}
		else if(shooterOn == 2){ // Low goals
			shootL.Set(0.4);
			shootR.Set(-0.4);
		}
		else{
			shootL.Set(0);
			shootR.Set(0);
		}
		//Shooter trigger
		if(xboxA){
			shootTrig.SetAngle(160);
		}
		else{
			shootTrig.SetAngle(50);
		}
		//------------------

		//------CLIMBER------

		if(xboxRS){		//Retract climber
			if(encPosRaw > 400){
				climbSpeed = -0.5;
			}
			else{
				climbSpeed = 0;
				climberL.Set(0);
				climberR.Set(0);
			}
		}
		else if(xboxLS){	//Extend climber
			if(encPosRaw > 400){
				climbSpeed = 0.5;
			}
			else{
				climbSpeed = 0;
				climberL.Set(0);
				climberR.Set(0);
			}
		}
		else if(xboxStart && xboxBack){		//Climb
			if(encPosRaw > 2000){
				climberL.Set(-1);
				climberR.Set(1);
				climbSpeed = -1;
			}
		}
		else if(stickMain.GetRawButton(5) && stickMain.GetRawButton(6)){ //Push Bottom 11 12 . 15 16
			climberL.Set(0.7);
			climberR.Set(-0.7);
			climbSpeed = 0;
		}
		else if(stickMain.GetRawButton(10) && stickMain.GetRawButton(9)){	//Pull bottom
			climberL.Set(-0.7);
			climberR.Set(0.7);
			climbSpeed = 0;
		}
		else if(stickMain.GetRawButton(11) && stickMain.GetRawButton(12)){	//Push all
			climberL.Set(0.7);
			climberR.Set(-0.7);
			climbSpeed = -0.7;
		}
		else if(stickMain.GetRawButton(15) && stickMain.GetRawButton(16)){	//Pull all
			climberL.Set(-0.7);
			climberR.Set(0.7);
			climbSpeed = 0.7;
		}
		else{
			climberL.Set(0);
			climberR.Set(0);
			climbSpeed = 0;
		}

		climberAdjust.Set(climbSpeed);
		//-------------------

		encPosRaw = shootAngle.GetPosition();

		//------DASHBOARD------
		SmartDashboard::PutNumber("encPosRaw", encPosRaw);
		SmartDashboard::PutNumber("encPosMin", encPosMin);
		SmartDashboard::PutNumber("encPosMax", encPosMax);

		SmartDashboard::PutNumber("gyroAngle", gyro.GetAngle());

		SmartDashboard::PutBoolean("lsAngleMax", lsAngleMax);
		SmartDashboard::PutBoolean("lsAngleMin", lsAngleMin);

		SmartDashboard::PutNumber("batteryVoltage", DriverStation::GetInstance().GetBatteryVoltage());
		SmartDashboard::PutNumber("matchTime", (135 - DriverStation::GetInstance().GetMatchTime()));

		float ultraDist = ultra.GetVoltage() / 0.00977;
		SmartDashboard::PutNumber("Ultrasonic Sensor Dist", ultraDist);

		//-----------------
		myRobot.ArcadeDrive(stickMain.GetY(), stickRot.GetX());
		Wait(0.005);

	}

	void TestPeriodic()
	{
	}
};

START_ROBOT_CLASS(Robot)
