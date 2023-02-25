// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

//************************************************************************************************    Include Libraries
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <math.h>

#include <ctre/Phoenix.h>

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

#include "AHRS.h"

#include <cameraserver/CameraServer.h>

//************************************************************************************************    Robot Code
class Robot : public frc::TimedRobot {
 public:
  //****************************************************************    Default robot functions
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  //****************************************************************    Angle optimization algorithm
  void AngleOptimize(double current, double desired){
    double current2 = current;

    while (abs(current2) >= 4096) {
		  if(current2<0){
			  current2 += 4096;
		  }
		  else {
			  current2 -= 4096;
		  }
		
	  }


	  if ((abs(current2) >= 2048)) {
		  if (current2 < 0) {
			  current2 = 2048 + fmod(current2, 2048);
		  }
		  else {
			  current2 = -2048 + fmod(current2, 2048);
		  }
	  }


	  direction = 1;
	  angle1 = (desired - current2);
  	angle2 = ((2048 - abs(desired)) + (2048 - abs(current2)));
    //angle2 = ((2048 - abs(fmod(desired,4096))) + (2048 - abs(current2)));

  	if (abs(angle1) <= abs(angle2)) {
  		shortest = angle1;
  		check1 = false;
  	}
  	else {
  		shortest = angle2;
  		check1 = true;
  	}
  
  double desiredOpposite;

  if(desired < 0){
    desiredOpposite = (2048 + desired);
  }else{
    desiredOpposite = (desired - 2048);
  }


	  angle1 = (desiredOpposite - current2);
	  angle2 = ((2048 - abs(desiredOpposite)) + (2048 - (abs(current2))));
	
  
	  if (abs(angle2) < abs(angle1)) {
		  shortest2 = angle2;
		  check2 = true;
	  }
	  else {
		  shortest2 = angle1;
		  check2 = false;
	  }


	  if (abs(shortest) <= abs(shortest2)) {
		  if (check1 == true) {
			  if (current < 0) {
				  current -= shortest;
			  }
			  else {
				  current += shortest;
			  }
		  }
		  else {
			  current += shortest;
		  }
	  }
	  else {
		  direction = -1;
		  if (check2 == true) {
			  if (current < 0) {
				  current -= shortest2;
			  }
			  else {
				  current += shortest2;
		  	}
		  } 
		  else {
			  current += shortest2;
	  	}
	  }
    outputAngle = current;
  }

  //****************************************************************    Drive common code
  void DriveCommon(double FWD, double STR, double RCW){
    yaw = ahrs->GetYaw();
    double yaw2 = yaw;
    if(yaw<0){
      yaw2 += 360;
    }

    yaw = (yaw * (M_PI/180));
    yaw2 = (yaw2 * (M_PI/180));

    double temp = ((FWD * cos(yaw2)) + (STR * sin(yaw)));
    STR = (((-FWD) * sin(yaw)) + (STR * cos(yaw2)));
    FWD = temp;

    //****************    Set wheel angles and speeds
    A = STR - RCW * (L / R);
	  B = STR + RCW * (L / R);
	  C = FWD - RCW * (W / R);
	  D = FWD + RCW * (W / R);

	  ws1 = sqrt((B * B) + (C * C));
	  wa1 = atan2(B, C) * 180 / M_PI;
    wa1 *= conversionFactor;

	  ws2 = sqrt((B * B) + (D * D));
	  wa2 = atan2(B, D) * 180 / M_PI;
    wa2 *= conversionFactor;

	  ws3 = sqrt((A * A) + (D * D));
	  wa3 = atan2(A, D) * 180 / M_PI;
    wa3 *= conversionFactor;

	  ws4 = sqrt((A * A) + (C * C));
	  wa4 = atan2(A, C) * 180 / M_PI;
    wa4 *= conversionFactor;

    //****************    Optimize wheel angles
    AngleOptimize(oldwa1,wa1);
    wa1=outputAngle;
    ws1*=direction;

    AngleOptimize(oldwa2,wa2);
    wa2=outputAngle;
    ws2*=direction;

    AngleOptimize(oldwa3,wa3);
    wa3=outputAngle;
    ws3*=direction;

    AngleOptimize(oldwa4,wa4);
    wa4=outputAngle;
    ws4*=direction;

    //****************    Set max speed
	  double max = ws1;
	  if (ws2 > max)max = ws2;
	  if (ws3 > max)max = ws3;
	  if (ws4 > max)max = ws4;
	  if (max > 1) { ws1 /= max; ws2 /= max; ws3 /= max; ws4 /= max; }

    //****************    Set angle motor position
    m_rra.Set(TalonFXControlMode::Position, wa4);
    m_fra.Set(TalonFXControlMode::Position, wa1);
    m_rla.Set(TalonFXControlMode::Position, wa3);
    m_fla.Set(TalonFXControlMode::Position, wa2);

    //****************    Set old angles
    oldwa1 = wa1;
    oldwa2 = wa2;
    oldwa3 = wa3;
    oldwa4 = wa4;
  }                             

  //****************************************************************    Autonomous drive loop
  void AutoDrive(double FWD, double STR, double RCW, double speed){
    DriveCommon(FWD,STR,RCW);
    m_frd.Set(TalonFXControlMode::Velocity, (-speed*ws1));
    m_fld.Set(TalonFXControlMode::Velocity, (-speed*ws2));
    m_rld.Set(TalonFXControlMode::Velocity, (-speed*ws3));
    m_rrd.Set(TalonFXControlMode::Velocity, (-speed*ws4));
  }

  //****************************************************************    Teleoperated drive loop
  void TeleDrive(double FWD, double STR, double RCW){
    DriveCommon(FWD,STR,RCW);
    double PercentOut = -0.3; // must be negative
    m_frd.Set(PercentOut*ws1);
    m_fld.Set(PercentOut*ws2);
    m_rld.Set(PercentOut*ws3);
    m_rrd.Set(PercentOut*ws4);
  }

  //****************************************************************    Arm control loop
  void GrabberArm(int in, int out, float up, float down){
    arm_extend.Set(0.2*(in-out));

    if((down>0) && (up==0)){
      arm_angle.Set(.3);
    }else if((up>0) && (down==0)){
      arm_angle.Set(-.3);
    }else{
      arm_angle.Set(0);
    }
  }

  //****************************************************************    Claw pneumatics loop
  void ClawPosition(int toggleState){
    p_solenoid.Toggle();
  }

//************************************************************************************************    Variables
  private:
  //****************************************************************    CONTROLLERS
    frc::XboxController m_driverController{0};
    
  //****************************************************************    Instancing CAN devices and sensors
    WPI_TalonFX m_fla{0};
    WPI_TalonFX m_fld{1};
    WPI_CANCoder m_flsensor{2};
    
    WPI_TalonFX m_fra{3};
    WPI_TalonFX m_frd{4};
    WPI_CANCoder m_frsensor{5};

    WPI_TalonFX m_rla{6};
    WPI_TalonFX m_rld{7};
    WPI_CANCoder m_rlsensor{8};

    WPI_TalonFX m_rra{9};
    WPI_TalonFX m_rrd{10};
    WPI_CANCoder m_rrsensor{11};

    WPI_VictorSPX arm_extend{12};
    WPI_TalonFX arm_angle{13};

    frc::DoubleSolenoid p_solenoid{14, frc::PneumaticsModuleType::REVPH, 4, 5};
    frc::Compressor p_compressor{14, frc::PneumaticsModuleType::REVPH};

    AHRS *ahrs;

  //****************************************************************    SWERVE DRIVE VARIABLES
    //****************    Conversion Factor
    double conversionFactor = 4096.0/ 360.0;

    //****************    Frame Ratios
    //wheelbase (from center of front wheel to center of back wheel)
    double L = 25.75;
    double W = 25.75;
    double R = sqrt((L * L) + (W * W));;

    //****************    Kinematics Variables
	  double A;
    double B;
    double C;
    double D;

    double ws1;
    double wa1;
    double oldwa1;

    double ws2;
    double wa2;
    double oldwa2;

    double ws3;
    double wa3;
    double oldwa3;

    double ws4;
    double wa4;
    double oldwa4;

    //****************    NAVX-MXP (Gyro board)
    double yaw;

    //****************    Swerve Drive Optimization
    double direction;

    double angle1;
  	double angle2;

  	double shortest;
    double shortest2;

  	bool check1;
	  bool check2;

    double outputAngle;

    double disp = 0;
  };