// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <ctre/Phoenix.h>
#include <iostream>
#include <frc/Timer.h>
#include <string>
#include <frc/Solenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <chrono>
#include <thread>
#include <math.h>
#include <cameraserver/CameraServer.h>
#include "AHRS.h"
#include <iostream>
//#include "frc/span"



class Robot : public frc::TimedRobot {
 public:
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

  //**************************************************************Drive Loop******************************************************
  void drive(double FWD, double STR, double RCW){
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
    std::cout<<"STR: "<<STR<<std::endl;


    //********Setting ratios for wheel distance********
    //wheelbase (from center of front wheel to center of back wheel) unit doesn't matter because these will only be used as ratios but works well with meters
    //IMPORTANT!!!    THESE TWO NUMBERS ARE THE LENGTH AND WIDTH, THE EXACT VALUE DOES NOT MATTER IF THE FRAME IS SQUARE!!!    >>> MUST CHANGE "L" & "W" WITH NON-SQUARE FRAME <<<
    L = 25.75;
	  W = 25.75;

	  R = sqrt((L * L) + (W * W));

	  A = STR - RCW * (L / R);
	  B = STR + RCW * (L / R);
	  C = FWD - RCW * (W / R);
	  D = FWD + RCW * (W / R);




    //********Set wheel angles and speeds********
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




    //*********Optimize wheel angles********
    angleOptimize(oldwa1,wa1);
    wa1=outputAngle;
    ws1*=direction;

    angleOptimize(oldwa2,wa2);
    wa2=outputAngle;
    ws2*=direction;

    angleOptimize(oldwa3,wa3);
    wa3=outputAngle;
    ws3*=direction;

    angleOptimize(oldwa4,wa4);
    wa4=outputAngle;
    ws4*=direction;



    //********Set max speed********
	  double max = ws1;
	  if (ws2 > max)max = ws2;
	  if (ws3 > max)max = ws3;
	  if (ws4 > max)max = ws4;
	  if (max > 1) { ws1 /= max; ws2 /= max; ws3 /= max; ws4 /= max; }




    //********Set angle motor position********
    m_rra.Set(TalonFXControlMode::Position, wa4);
    m_fra.Set(TalonFXControlMode::Position, wa1);
    m_rla.Set(TalonFXControlMode::Position, wa3);
    m_fla.Set(TalonFXControlMode::Position, wa2);




    //********Set old angles********
    oldwa1 = wa1;
    oldwa2 = wa2;
    oldwa3 = wa3;
    oldwa4 = wa4;




    //********Drive Speed!********
    double PercentOut = -0.3; // must be negative


    //********Set Drive Motors********
    m_frd.Set(PercentOut*ws1);
    m_fld.Set(PercentOut*ws2);
    m_rld.Set(PercentOut*ws3);
    m_rrd.Set(PercentOut*ws4);
  }                             

  //***************************************************Angle Optimization Algorithm**************************************************
  void angleOptimize(double current, double desired){
    double current2 = current;



    while (abs(current2) > 4096) {
		  if(current2<0){
			  current2 += 4096;
		  }
		  else {
			  current2 -= 4096;
		  }
		
	  }

	  if ((abs(current2) >= 2048)&&(abs(current2)!=4096)) {
		  if (current2 < 0) {
			  current2 = 2048 + fmod(current2, 2048);
		  }
		  else {
			  current2 = -2048 + fmod(current2, 2048);
		  }
	  }
	  else {
		  current2 = fmod(current2, 2048);
	  }



	  direction = 1;
	  angle1 = (desired - current2);
  	angle2 = ((2048 - abs(fmod(desired,4096))) + (2048 - abs(current2)));

  	if (abs(angle1) <= abs(angle2)) {
  		shortest = angle1;
  		check1 = false;
  	}
  	else {
  		shortest = angle2;
  		check1 = true;
  	}
  
	  angle1 = ((desired - 2048) - current2);
	  angle2 = ((2048 - (abs(fmod(desired,4096))-2048)) + (2048 - (abs(current2))));
	
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

  private:
    //********************************************************CONTROLLERS********************************************************
    frc::XboxController m_driverController{0};

    //****************************************************MOTORS AND ENCODERS****************************************************
    WPI_TalonFX m_fra{9};
    WPI_TalonFX m_frd{11};

    WPI_TalonFX m_rra{12};
    WPI_TalonFX m_rrd{10};

    WPI_TalonFX m_fla{0};
    WPI_TalonFX m_fld{5};

    WPI_TalonFX m_rla{15};
    WPI_TalonFX m_rld{18};


    WPI_CANCoder m_rrsensor{5};
    WPI_CANCoder m_frsensor{2};
    WPI_CANCoder m_rlsensor{1};
    WPI_CANCoder m_flsensor{3};

    //****************************************************SWERVE DRIVE VARIABLES****************************************************
    double conversionFactor = 4096.0/ 360.0;

    double L;
    double W;
    double R;

	  double A;
    double B;
    double C;
    double D;

    double ws1;
    double wa1;
    double ws2;
    double wa2;
    double ws3;
    double wa3;
    double ws4;
    double wa4;

    double oldwa1;
    double oldwa2;
    double oldwa3;
    double oldwa4;

    //****************************************************NAVX-MXP (Gyro board)***************************************
    AHRS *ahrs;
    double yaw;

    //*********************************************Swerve Drive Optimization******************************************
    double direction;
    double angle1;
  	double angle2;
  	double shortest;
  	bool check1;
    double shortest2;
	  bool check2;

    double outputAngle;

  };
