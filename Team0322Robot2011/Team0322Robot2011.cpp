/*
 * Team322Robot2011.cpp
 *
 *  Created on: Jan 20, 2011
 *      Author: 322Programmer (Raa'Shaun L. Hunter)
 */

#include <math.h>
#include "DriverStationLCD.h"
#include "WPILib.h"
#include "Vision/AxisCamera.h"
#include "Team0322Robot2011.h"
#include "ZomBDashboard.h"

class Team322Robot2011 : public IterativeRobot
{
	// Declare variable for the robot drive system
	RobotDrive *m_robotDrive;		// robot will use PWM 1&2 on each sidecar for drive motors
	
	// Declare a variable to use to access the driver station object
	DriverStation *m_ds;						// Driver Station object
	DriverStationLCD *m_dsLCD;					// Driver Station LCD screen object
	UINT32 m_priorPacketNumber;					// keep track of the most recent packet number from the DS
	UINT8 m_dsPacketsReceivedInCurrentSecond;	// keep track of the ds packets received in the current second
	
	// Declare the ZomB Dashboard variable
	ZomBDashboard zb;
	
	// Declare the AxisCamera variable
	AxisCamera& camera;

	// Declare variables for the joysticks being used
	Joystick *m_driveStick;			// joystick 1 (drivetrain controller)
	Joystick *m_armStick1;			// joystick 2 (drivetrain joystick 1)
	Joystick *m_armStick2;			// joystick 3 (drivetrain joystick 2)
	Joystick *m_manipulatorStick;	// joystick 4 (Stick for the gamepiece manipulator controls)
	
	//Jaguar *m_frontLeftMotor, *m_frontRightMotor;
	Victor *m_frontLeftMotor, *m_frontRightMotor;
	//Jaguar *m_rearLeftMotor, *m_rearRightMotor;
	Victor *m_rearLeftMotor, *m_rearRightMotor;
	
	Victor *m_lowerArm1, *m_lowerArm2, *m_wrist, *m_grabber;
	
	Servo *m_cameraPitch, *m_cameraYaw;
	
	Encoder *m_leftFrontEncoder, *m_leftRearEncoder, *m_rightFrontEncoder, *m_rightRearEncoder; 
	
	AnalogChannel *m_lowerArmPotentiometer, *m_wristPotentiometer;
	
	Gyro *m_gyro;
	Accelerometer *m_accelX,*m_accelY,*m_accelZ;
	
	DigitalOutput *m_leftBrake1, *m_leftBrake2, *m_rightBrake1, *m_rightBrake2;
	DigitalInput *m_armStop;
	
	int m_grabberActive;
	float m_grabberSpeed, m_armSpeed, m_wristSpeed;
	
	int ARM_POTENTIOMETER_MIN, WRIST_POTENTIOMETER_MIN, ARM_POTENTIOMETER_MAX, WRIST_POTENTIOMETER_MAX;
	
	// Local variables to count the number of periodic loops performed
	UINT32 m_autoPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;
	
public:
/**
 * Constructor for this "Team322Robot2011" Class.
 * 
 * The constructor creates all of the objects used for the different inputs and outputs of
 * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
 * providing named objects for each of the robot interfaces. 
 */
	Team322Robot2011(void) : zb(ZomBDashboard::GetInstance(TCP)), camera(AxisCamera::GetInstance())
	{
		printf("Team322Robot2011 Constructor Started\n");

		// Create the SpeedController objects for the drivetrain 
		m_frontLeftMotor = new Victor(4,1);
		m_rearLeftMotor = new Victor(4,2);
		m_frontRightMotor = new Victor(6,1);
		m_rearRightMotor = new Victor(6,2);
		
		// Create the SpeedController objects for the arm
		m_lowerArm1 = new Victor(4,3);
		m_lowerArm2 = new Victor(6,3);
		m_grabber = new Victor(4,4);
		m_wrist = new Victor(6,4);
		
		// Create the robot's drivetrain
		m_robotDrive = new RobotDrive(m_frontLeftMotor, m_rearLeftMotor, m_frontRightMotor, m_rearRightMotor);

		// Create an Encoder object for each transmission
		m_leftFrontEncoder = new Encoder(4,2,4,12,true, m_leftFrontEncoder->k4X);
		m_leftRearEncoder = new Encoder(4,1,4,11,true, m_leftRearEncoder->k4X);
		m_rightFrontEncoder = new Encoder(6,2,6,12,false, m_rightFrontEncoder->k4X);
		m_rightRearEncoder = new Encoder(6,1,6,11,false, m_rightRearEncoder->k4X);
		
		// Create Analog Objects for arm position
		m_lowerArmPotentiometer = new AnalogChannel(5);
		m_wristPotentiometer = new AnalogChannel(6);
		
		// Create a Gyro(scope) object on AnalogModule 1 & AnalogInput 1
		m_gyro = new Gyro(1,1);
		
		// Create the 3 Accelerometer objects for our 3 axis Accelerometer
		// on AnalogModule 1 & AnalogInputs 2, 3, and 4
		m_accelX = new Accelerometer(1,2);
		m_accelY = new Accelerometer(1,3);
		m_accelZ = new Accelerometer(1,4);
		
		// Acquire the Driver Station and Dashboard objects and initialize variables 
		m_ds = DriverStation::GetInstance();
		m_dsLCD = DriverStationLCD::GetInstance();
		m_priorPacketNumber = 0;
		m_dsPacketsReceivedInCurrentSecond = 0;
		
		// Define joysticks being used on the Drivers Station
		m_driveStick = new Joystick(1, 6, 8);
		m_armStick1 = new Joystick(2);
		m_armStick2 = new Joystick(3);
		m_manipulatorStick = new Joystick(4);

		// Define Servo's
		m_cameraPitch = new Servo(4,5);
		m_cameraYaw = new Servo(4,6);
		
		// Create the DigitalOutput objects for the brake/coast header control system
		m_leftBrake1 = new DigitalOutput(4,5);
		m_leftBrake2 = new DigitalOutput(4,6);
		m_rightBrake1 = new DigitalOutput(6,5);
		m_rightBrake2 = new DigitalOutput(6,6);

		// Create the DigitalInput object for the arm limit switch 
		m_armStop = new DigitalInput(4,7);
		
		m_grabberActive = 0;
		m_grabberSpeed = m_armSpeed = m_wristSpeed = 0.0;
		
		// Initialize counters to record the number of loops completed in autonomous and teleop modes
		m_autoPeriodicLoops = 0;
		m_disabledPeriodicLoops = 0;
		m_telePeriodicLoops = 0;
		
		// Initialize the potentiometer minimum and maximum values
		ARM_POTENTIOMETER_MIN = 635;
		//WRIST_POTENTIOMETER_MIN = 510;
		WRIST_POTENTIOMETER_MIN = 0;
		ARM_POTENTIOMETER_MAX = 980;
		//WRIST_POTENTIOMETER_MAX = 950;
		WRIST_POTENTIOMETER_MAX = 1000;
		
		// Camera Display Code
		camera.WriteResolution(AxisCamera::kResolution_320x240);
		camera.WriteCompression(20);
		camera.WriteBrightness(0);
		
		ClearLCD();
		ClearDashboard();
		printf("Team322Robot2011 Constructor Completed\n");
	}
	
	
	/********************************** Init Routines *************************************/

	void RobotInit(void) {
		// Actions which would be performed once (and only once) upon initialization of the
		// robot would be put here.
		
		// Set the inversion of the motors 
		m_robotDrive->SetInvertedMotor(m_robotDrive->kFrontLeftMotor,false);
		m_robotDrive->SetInvertedMotor(m_robotDrive->kFrontRightMotor,false);
		m_robotDrive->SetInvertedMotor(m_robotDrive->kRearLeftMotor,false);
		m_robotDrive->SetInvertedMotor(m_robotDrive->kRearRightMotor,false);
		
		//Set-up the Gyro
		m_gyro->SetSensitivity(2.5);
		
		// Start the Encoders
		m_leftFrontEncoder->Start();
		m_leftRearEncoder->Start();
		m_rightFrontEncoder->Start();
		m_rightRearEncoder->Start();
		
		// Setup the Accelerometers
		m_accelX->SetZero(1.5);
		m_accelX->SetSensitivity(0.3);
		m_accelY->SetZero(1.5);
		m_accelY->SetSensitivity(0.3);
		m_accelZ->SetZero(1.5);
		m_accelZ->SetSensitivity(0.3);
		
		UpdateLCD(0);
		UpdateDashboard();

		printf("RobotInit() completed.\n");
	}
	
	void DisabledInit(void) {
		m_leftFrontEncoder->Reset();	// Reset the left front encoder count
		m_leftRearEncoder->Reset();		// Reset the left rear encoder count
		m_rightFrontEncoder->Reset();	// Reset the right front encoder count
		m_rightRearEncoder->Reset();	// Reset the right rear encoder count
		
		m_gyro->Reset();				// Reset the gyro heading
		
		// Reset the Camera direction
		m_cameraPitch->Set(0.5);
		m_cameraYaw->Set(0.5);
		
		UpdateLCD(0);
		UpdateDashboard();
		
		m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
		// Move the cursor down a few, since we'll move it back up in periodic.
		printf("\x1b[2B");
	}

	void AutonomousInit(void) {
		m_leftFrontEncoder->Reset();	// Reset the left front encoder count
		m_leftRearEncoder->Reset();		// Reset the left rear encoder count
		m_rightFrontEncoder->Reset();	// Reset the right front encoder count
		m_rightRearEncoder->Reset();	// Reset the right rear encoder count
		
		// Reset the Camera direction
		m_cameraPitch->Set(0.5);
		m_cameraYaw->Set(0.5);
		
		m_autoPeriodicLoops = 0;		// Reset the loop counter for autonomous mode
	}

	void TeleopInit(void) {
		
		// Reset the Camera direction
		m_cameraPitch->Set(0.5);
		m_cameraYaw->Set(0.5);
		
		UpdateLCD(0);
		UpdateDashboard();
		
		m_telePeriodicLoops = 0;				// Reset the loop counter for teleop mode
		m_dsPacketsReceivedInCurrentSecond = 0;	// Reset the number of dsPackets in current second
	}

	/********************************** Periodic Routines *************************************/
	
	void DisabledPeriodic(void)  {
		static INT32 printSec = (INT32)GetClock() + 1;
		static const INT32 startSec = (INT32)GetClock();

		// feed the user watchdog at every period when disabled
		GetWatchdog().Feed();

		// while disabled, printout the duration of current disabled mode in seconds
		if (GetClock() > printSec) {
			// Move the cursor back to the previous line and clear it.
			printf("\x1b[1A\x1b[2K");
			printf("Disabled seconds: %d\r\n", printSec - startSec);
			printSec++;
		}
		
		UpdateLCD(0);
		UpdateDashboard();
		
		// increment the number of disabled periodic loops completed
		m_disabledPeriodicLoops++;
	}

	void AutonomousPeriodic(void) {
		// feed the user watchdog at every period when in autonomous
		GetWatchdog().Feed();
		
		if ((m_autoPeriodicLoops % (UINT32)GetLoopsPerSec()) == 0) {
			printf("Left Front Distance = %f", m_leftFrontEncoder->GetDistance());
			printf(" Right Front Distance = %f", m_rightFrontEncoder->GetDistance());
			printf("\r\n");
			printf("Current Angle = %f", m_gyro->GetAngle());
			printf("\r\n");
			printf("X-Axis Acceleration = %f", m_accelX->GetAcceleration());
			printf(" Y-Axis = %f", m_accelY->GetAcceleration());
			printf(" Z-Axis = %f", m_accelZ->GetAcceleration());
			printf("\r\n");
			printf("\r\n");
		}
		
		UpdateLCD(0);
		UpdateDashboard();
		
		m_autoPeriodicLoops++;
	}

	
	void TeleopPeriodic(void) {
		// feed the user watchdog at every period when in autonomous
		GetWatchdog().Feed();

		/*
		 * No longer needed since periodic loops are now synchronized with incoming packets.
		if (m_ds->GetPacketNumber() != m_priorPacketNumber) {
		*/
			/* 
			 * Code placed in here will be called only when a new packet of information
			 * has been received by the Driver Station.  Any code which needs new information
			 * from the DS should go in here
			 */
			 
			m_priorPacketNumber = m_ds->GetPacketNumber();
			m_dsPacketsReceivedInCurrentSecond++;					// increment DS packets received
			
			// put Driver Station-dependent code here
			
			// This is the Drivetrain code
			m_robotDrive->MecanumDrive_Cartesian(-(m_driveStick->GetRawAxis(1)), m_driveStick->GetRawAxis(4),
					m_driveStick->GetRawAxis(2), m_gyro->GetAngle());
			
			//m_robotDrive->MecanumDrive_Cartesian(-(m_driveStick->GetRawAxis(1)), m_manipulatorStick->GetX(),
			//		m_manipulatorStick->GetY(), m_gyro->GetAngle());
			
			/*
			if(m_driveStick->GetRawButton(1))
				{
					m_arm = m_armStick1->GetZ();
					ArmControl(m_arm);
				}
			if(m_driveStick->GetRawButton(2))
				{
					m_wrist->Set(m_manipulatorStick->GetZ());
				}
			if(m_driveStick->GetRawButton(3))
				{
					m_grabberSpeed = m_armStick2->GetZ();
					GrabberControl(1);
				}
			GrabberControl(0);
			*/
			
			// This is the lower arm control code
			m_armSpeed = m_armStick1->GetY();
			
			ArmControl(m_armSpeed);
			
			// This is the wrist control code
			m_wristSpeed = m_armStick2->GetY();
			
			WristControl(m_wristSpeed);
			
			// Grabber Control System
			m_grabberSpeed = m_armStick2->GetZ();
			
			if(m_armStick2->GetTrigger()) m_grabberActive = 1;
			else m_grabberActive = 0;
			GrabberControl(m_grabberActive);
			
			// Manual Brake Control System
			if(m_driveStick->GetRawButton(6)) BrakeControlSystem(1);
			else BrakeControlSystem(0);
			
			// Servo Controls
			m_cameraPitch->Set(((m_manipulatorStick->GetY()) / 2) + 0.5);
			m_cameraYaw->Set(((m_manipulatorStick->GetX()) / 2) + 0.5);
			
		/*
		}  // if (m_ds->GetPacketNumber()...
		*/
			if ((m_telePeriodicLoops % (UINT32)GetLoopsPerSec()) == 0)
			{
				printf("Left Front Distance = %f", m_leftFrontEncoder->GetDistance());
				printf("\r\n");
				printf("Right Front Distance = %f", m_rightFrontEncoder->GetDistance());
				printf("\r\n");
				printf("Current Angle = %f", m_gyro->GetAngle());
				printf("\r\n");
				printf("X-Axis Acceleration = %f", m_accelX->GetAcceleration());
				printf("\r\n");
				printf("Y-Axis Acceleration = %f", m_accelY->GetAcceleration());
				printf("\r\n");
				printf("Z-Axis Acceleration = %f", m_accelZ->GetAcceleration());
				printf("\r\n");
				printf("\r\n");
			}
		
			//UpdateLCD(0);
			UpdateDashboard();
			
		m_telePeriodicLoops++;			// increment the number of teleop periodic loops completed
	} // TeleopPeriodic(void)


/********************************** Continuous Routines *************************************/

	void DisabledContinuous(void) {
		if (((m_disabledPeriodicLoops / (UINT32)GetLoopsPerSec()) % 4 == 2) ||
			((m_disabledPeriodicLoops / (UINT32)GetLoopsPerSec()) % 4 == 3)) UpdateLCD(1);
		else UpdateLCD(0);
		//UpdateDashboard();
	}

	void AutonomousContinuous(void)	{
		if (((m_autoPeriodicLoops / (UINT32)GetLoopsPerSec()) % 4 == 2) ||
				((m_autoPeriodicLoops / (UINT32)GetLoopsPerSec()) % 4 == 3)) UpdateLCD(1);
		else UpdateLCD(0);
		//UpdateDashboard();
	}

	void TeleopContinuous(void) {
		if (((m_telePeriodicLoops / (UINT32)GetLoopsPerSec()) % 4 == 2) ||
				((m_telePeriodicLoops / (UINT32)GetLoopsPerSec()) % 4 == 3)) UpdateLCD(1);
		else UpdateLCD(0);
		//UpdateDashboard();
	}

	
/********************************** Miscellaneous Routines *************************************/
	
	void BrakeControlSystem(int brakeCondition)
	{	
		switch(brakeCondition)
		{
			case 0:
				m_leftBrake1->Set(1);
				m_rightBrake1->Set(1);
				m_leftBrake2->Set(1);
				m_rightBrake2->Set(1);
				break;
			
			case 1:
				m_leftBrake1->Set(0);
				m_rightBrake1->Set(0);
				m_leftBrake2->Set(0);
				m_rightBrake2->Set(0);
				break;
			
			default:
				m_leftBrake1->Set(0);
				m_rightBrake1->Set(0);
				m_leftBrake2->Set(0);
				m_rightBrake2->Set(0);
				printf("How in the world did you get here!?\r\n");
				return;
		}
	}
	
	void ArmControl(float armSpeed)
		{
			//m_lowerArm1->Set(armSpeed);
			//m_lowerArm2->Set(-(armSpeed));
			if (m_armStop->Get() == 1 && armSpeed != 0.0)
			{
				m_lowerArm1->Set(0.0);
				m_lowerArm2->Set(0.0);
			}
			else if(m_lowerArmPotentiometer->GetAverageValue() >= ARM_POTENTIOMETER_MAX && armSpeed < 0.0)
			{
				m_lowerArm1->Set(0.0);
				m_lowerArm2->Set(0.0);
			}
			else if(m_lowerArmPotentiometer->GetAverageValue() <= ARM_POTENTIOMETER_MIN && armSpeed > 0.0)
			{
				m_lowerArm1->Set(0.0);
				m_lowerArm2->Set(0.0);
			}
			else
			{
				m_lowerArm1->Set(armSpeed);
				m_lowerArm2->Set(-(armSpeed));
			}
		}
	
	void WristControl(float wristSpeed)
	{
		//m_wrist->Set(wristSpeed);
		if(m_wristPotentiometer->GetAverageValue() >= WRIST_POTENTIOMETER_MAX && wristSpeed < 0.0)
			{
				m_wrist->Set(0.0);
			}
		else if(m_wristPotentiometer->GetAverageValue() <= WRIST_POTENTIOMETER_MIN && wristSpeed > 0.0)
			{
				m_wrist->Set(0.0);
			}
		else
			{
				m_wrist->Set(wristSpeed);
			}
	}
	
	void GrabberControl(int mode)
		{
			switch(mode)
				{
				case 0:
					m_grabber->Set(0.0);
					break;
					
				case 1:
					m_grabber->Set(m_grabberSpeed);
					break;
					
				default:
					m_grabber->Set(0.0);
					break;				
				}
		}
	
	void ClearLCD(void)
	{		
		m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "");
		m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "");
		m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "");
		m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "");
		m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "");
		m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "");
		m_dsLCD->UpdateLCD();
	}
	
	void UpdateLCD(int type)
	{	
		switch(type)
		{
			case 0:
				m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Gyro Angle: %7.3f", m_gyro->GetAngle());
				m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "Arm Pot:  %i", m_lowerArmPotentiometer->GetAverageValue());
				m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "Wrist Pot:  %i", m_wristPotentiometer->GetAverageValue());
				m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "X-Axis Acc = %f", m_accelX->GetAcceleration());
				m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "Y-Axis Acc = %f", m_accelY->GetAcceleration());
				m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "Z-Axis Acc = %f", m_accelZ->GetAcceleration());
				m_dsLCD->UpdateLCD();
			break;

			case 1:
				m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Gyro Angle: %7.3f", m_gyro->GetAngle());
				m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "Arm Pot:  %i", m_lowerArmPotentiometer->GetAverageValue());
				m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "Wrist Pot:  %i", m_wristPotentiometer->GetAverageValue());
				m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "X-Axis Acc = %f", m_accelX->GetAcceleration());
				m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "Y-Axis Acc = %f", m_accelY->GetAcceleration());
				m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "Z-Axis Acc = %f", m_accelZ->GetAcceleration());
				m_dsLCD->UpdateLCD();
			break;
			
			default:
				m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Gyro Angle: %7.3f", m_gyro->GetAngle());
				m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "%f",m_leftRearEncoder->GetDistance());
				m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "%f",m_rightRearEncoder->GetDistance());
				m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "X-Axis Acc = %f", m_accelX->GetAcceleration());
				m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "Y-Axis Acc = %f", m_accelY->GetAcceleration());
				m_dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "Z-Axis Acc = %f", m_accelZ->GetAcceleration());
				m_dsLCD->UpdateLCD();
			break;
		}
	}
	
	void ClearDashboard(void)
	{
		return;
	}
	
	void UpdateDashboard(void)
	{
		if (zb.CanSend())
			{
				zb.Add("driveStick1Y", m_driveStick->GetRawAxis(2));
				zb.Add("driveStick1X", m_driveStick->GetRawAxis(1));
				zb.Add("driveStick2X", m_driveStick->GetRawAxis(4));
				zb.Add("leftFrontOutput", m_frontLeftMotor->Get());
				zb.Add("rightFrontOutput", m_frontRightMotor->Get());
				zb.Add("leftRearOutput", m_rearLeftMotor->Get());
				zb.Add("rightRearOutput", m_rearRightMotor->Get());
				zb.Add("gyro", fmod(m_gyro->GetAngle(), 360));
				zb.Add("xAccel", m_accelX->GetAcceleration());
				zb.Add("yAccel", m_accelY->GetAcceleration());
				zb.Add("zAccel", m_accelZ->GetAcceleration());
				zb.Add("brakes", m_driveStick->GetRawButton(6));
				zb.Add("armPotentiometer", m_lowerArmPotentiometer->GetAverageValue());
				zb.Add("wristPotentiometer", m_wristPotentiometer->GetAverageValue());
				zb.Send();
			}
	}
			
};

START_ROBOT_CLASS(Team322Robot2011);
