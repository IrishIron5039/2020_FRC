/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
Gamepad: 
Intake: getRawAxes(2)
Output: getRawAxes(3)
elevator up: getRawButton(6)
elevator down: getRawButton(5)
lock: getRawButton(10)
up position: getRawButton(4)
middle position: getRawButton(2)
down position: getRawButton(1)

Joystick: driving
Shifting gears (Toggle): getRawButton(1)
*/

#include <iostream>
#include <string>

#include <frc/IterativeRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "frc/Joystick.h"
#include "ctre/Phoenix.h"
#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/PIDController.h>
#include <frc/DriverStation.h>
#include <frc/WPILib.h> //Including everything you need for the robot code to work

class Robot : public frc::IterativeRobot {
public:
	frc::ADXRS450_Gyro *gyro = new frc::ADXRS450_Gyro();
	
	WPI_TalonSRX *left = new WPI_TalonSRX(0); 
	WPI_TalonSRX *leftFollower = new WPI_TalonSRX(1);
	WPI_TalonSRX *leftFollower2 = new WPI_TalonSRX(2);  
	WPI_TalonSRX *right = new WPI_TalonSRX(3); 
	WPI_TalonSRX *rightFollower = new WPI_TalonSRX(4); 
	WPI_TalonSRX *rightFollower2 = new WPI_TalonSRX(5);
	WPI_TalonSRX *elevator = new WPI_TalonSRX(6); 
	WPI_TalonSRX *elevator2 = new WPI_TalonSRX(7); 
	WPI_TalonSRX *intakeBalls = new WPI_TalonSRX(8); 
	WPI_TalonSRX *ballWrist = new WPI_TalonSRX(9); 
	WPI_TalonSRX *controlPanelSpin = new WPI_TalonSRX(10); 

	// Kiran
	//frc::Encoder elevatorEncoder{0, 1}; // This is hopefully initializing a rotary quadrature encoder
	//bool elevatorUp = false;

	frc::DoubleSolenoid changeWheelSpeed {3, 5}; //Pneumatics, connected to spot 1 and 5 to the powerboard next to the battery
	//frc::DoubleSolenoid endAffector {1, 6}; //Connected to spot 1 and 6
	//frc::DoubleSolenoid hatch {2, 3}; // Connected to spot 2 and 3

	frc::DifferentialDrive *drive = new frc::DifferentialDrive(*left, *right); //Setting drive to 1 joystick, adds the left and right joystick

	frc::Joystick *joy = new frc::Joystick(0);
	frc::Joystick *gamePad = new frc::Joystick(1);


	float angleElevator = 0; //Placeholder value
	bool wheelSpeedFast = true;
	int lowestElevatorPosition = elevator->GetSensorCollection().GetPulseWidthPosition() + 500;
	int highestElevatorPosition = lowestElevatorPosition + 4096*2.5 - 1000;

	void RobotInit() {

		cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
		cs::UsbCamera camera2 = frc::CameraServer::GetInstance()->StartAutomaticCapture();
		camera.SetResolution(320,240);
		camera2.SetResolution(320,240);

		left->SetInverted(true);
		right->SetInverted(true);
		leftFollower->SetInverted(true);
		rightFollower->SetInverted(true);
		leftFollower2->SetInverted(true);
		rightFollower2->SetInverted(true);
		
		

		leftFollower->Follow(*left);
		rightFollower->Follow(*right);
		leftFollower2->Follow(*left);
		rightFollower2->Follow(*right);
		
		//elevator2->Follow(*elevator);

		ballWrist->ConfigSelectedFeedbackSensor(FeedbackDevice::Analog, 0, 0);
		ballWrist->ConfigSetParameter(ParamEnum::eFeedbackNotContinuous, 1, 0x00,0x00, 0x00);
		ballWrist->SetSensorPhase(true);

		ballWrist->Config_kF(0, 0.0, 0);
		ballWrist->Config_kP(0, 1, 0);
		ballWrist->Config_kI(0, 0.0, 0);
		ballWrist->Config_kD(0, 0.0, 0);

		gyro->Calibrate();
		gyro->Reset();

		changeWheelSpeed.Set(frc::DoubleSolenoid::Value::kForward);
		//hatch.Set(frc::DoubleSolenoid::Value::kForward);
		//endAffector.Set(frc::DoubleSolenoid::Value::kForward);

		// Kiran
		//elevatorEncoder.SetDistancePerPulse(1./256.);
		//elevatorEncoder.Reset();

	}

	void AutonomousInit() {
		gyro->Reset();
	}



void AutonomousPeriodic() {
	//drive->ArcadeDrive(joy->GetY(), -joy->GetX(), true);
	//elevator->Set(1);
	/*if (gamePad->GetRawButton(1)) {
		angleElevator = 1000;//Placeholder
		elevator->Config_kP(0, 1, 0);
	}
	if (gamePad->GetRawButton(2)) {
		angleElevator = 0;//Placeholder
		elevator->Config_kP(0, 1, 0);
	}
	elevator->Set(ControlMode::Position, angleElevator);*/
}
	void TeleopInit() {
		gyro->Reset();
	}

	void TeleopPeriodic() {
		drive->ArcadeDrive(joy->GetY(), -joy->GetX(), true);
		if(gamePad->GetRawButton(5)){
			if(!(elevator->GetSensorCollection().GetPulseWidthPosition() < lowestElevatorPosition)){
				elevator->Set (-0.25);
			}
			else elevator->Set (0);
		}
		else if(gamePad->GetRawButton(6)){
			if(!(elevator->GetSensorCollection().GetPulseWidthPosition() > highestElevatorPosition)){
				elevator->Set (0.25);
			}
			else elevator->Set (0);
		}
		//elevator->Set (0.5 * (gamePad->GetRawButton(5)-gamePad->GetRawButton(6)));
		//std::cout << "GetPulseWidthPosition: " << elevator->GetSensorCollection().GetPulseWidthPosition() << "\n";
		std::cout << "Left: " << left->GetSensorCollection().GetPulseWidthPosition() << "\n";
		std::cout << "Right: " << right->GetSensorCollection().GetPulseWidthPosition() << "\n";
		
		if (joy->GetRawButton(1)) changeWheelSpeed.Set(frc::DoubleSolenoid::Value::kReverse);
		
		else changeWheelSpeed.Set(frc::DoubleSolenoid::Value::kForward);
		
		/*if(gamePad->GetRawButton(1)){
			elevatorUp = !elevatorUp;
		}

		// Kiran
		if(elevatorUp && elevatorEncoder.GetDistance < 2.75){
			elevator->Set(1);
		}
		else if(!elevatorUp && elevatorEncoder.GetDistance > 0.25){
			elevator->Set(-1);
		}
		*/
	}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

#ifndef RUNNING_FRC_TESTS
int main(){
  frc::StartRobot<Robot>(); //Runs the robot code
  return 1;
}
#endif
