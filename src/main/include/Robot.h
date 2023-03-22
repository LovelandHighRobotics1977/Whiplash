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
#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

#include "AHRS.h"

#include <cameraserver/CameraServer.h>
#include "rev/CANSparkMax.h"
#include <frc/DigitalInput.h>
#include <frc/Encoder.h>

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
    /*double current2 = fmod(current, 4096.0);
    double desired2 = fmod(desired, 4096.0);

    double difference = desired2 - current2;

    if (difference > 2048.0) {
        difference -= 4096.0;
    }
    else if (difference < -2048.0) {
        difference += 4096.0;
    }
    
    //checking opposite angle
    double desiredOpposite;
    if (desired <= 0) {
        desiredOpposite = (2048 + desired);
    }
    else {
        desiredOpposite = (desired - 2048);
    }

    desired2 = fmod(desiredOpposite, 4096.0);

    double difference2 = desired2 - current2;

    if (difference2 > 2048.0) {
        difference2 -= 4096.0;
    }
    else if (difference2 < -2048.0) {
        difference2 += 4096.0;
    }

    if (abs(difference) <= abs(difference2)) {
        outputAngle = current + difference;
    }
    else {
        direction = -1;
        outputAngle = current + difference2;
    }*/
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
    m_frd.Set(TalonFXControlMode::Velocity, (speed*ws1));
    m_fld.Set(TalonFXControlMode::Velocity, (speed*ws2));
    m_rld.Set(TalonFXControlMode::Velocity, (speed*ws3));
    m_rrd.Set(TalonFXControlMode::Velocity, (speed*ws4));
  }

  void driveDistance(double distanceInInches, double speed, double orientationSet, int direction = 0){
    double baseSpeed = 12;
    double speedScalar = speed/100.0;
    double timeRequired = (distanceInInches/(baseSpeed*speedScalar));
    double orientation = 0;
    
    if(timeTest == false){
      timer.Reset();
      timer.Start();
      timeTest = true;
    }

    while(  timer.Get() <= ((units::time::second_t) timeRequired)){
      
      if(abs(ahrs->GetAngle()) < orientationSet-2){
        orientation = 0.1;
      }else if(abs(ahrs->GetAngle()) > orientationSet+2){
        orientation = -0.1;
      }else{
        orientation = 0;
      }

      switch(direction){
      case 0:
        AutoDrive(1,0,orientation,speed);
        break;
      case 1:
        AutoDrive(0,1,orientation,speed);
        break;
      case 2:
        AutoDrive(-1,0,orientation,speed);
        break;
      case 3:
        AutoDrive(0,-1,orientation,speed);
        break;
      }
    }

    AutoDrive(0,0,0,0);
    
    timeTest = false;
  }

  //****************************************************************    Teleoperated drive loop
  void TeleDrive(double FWD, double STR, double RCW, double throttle){
    DriveCommon(FWD,STR,RCW);
    double PercentOut = .75;
    m_frd.Set(PercentOut*ws1*throttle);
    m_fld.Set(PercentOut*ws2*throttle);
    m_rld.Set(PercentOut*ws3*throttle);
    m_rrd.Set(PercentOut*ws4*throttle);
  }

  //****************************************************************    Arm control loop
  void GrabberArm(int out, int in, float down, float up, bool retract){
    //std::cout<<(!armSwitch.Get())<<"    "<<(in==1)<<"    "<<(out == 0)<<std::endl;
    if(retract == false){
    if((!armSwitch.Get())&&(out == 0)&&(in == 1)){
      arm_extend.Set(1);
    }
    else if((out == 1)&&(in == 0)){
      arm_extend.Set(-1);
    }else {
      arm_extend.Set(0);
    }

    if((down>0) && (up==0)){
      arm_angle.Set(.3 * down);
    }else if((up>0) && (down==0)&&(!insideSwitch.Get())){
      arm_angle.Set(-.3 * up);
    }else{
      arm_angle.Set(0);
    }
    }

    if(retract == true){
      if(!armSwitch.Get()){
        arm_extend.Set(1);
      }else{
        arm_extend.Set(0);
      }

      if((!insideSwitch.Get())&&(armSwitch.Get())){
        arm_angle.Set(-.2);
      }else{
        arm_angle.Set(0);
      }

    }
  }

  void autoArm(int position, bool enabled){
    if(!enabled){
      positionSpeed = .3;
    }
    if(enabled){
    if(abs(arm_encoder.GetDistance()) < position-2){
      arm_angle.Set(positionSpeed);
    }else if(abs(arm_encoder.GetDistance()) > (position+2)){
      arm_angle.Set(-positionSpeed);
    }else if((abs(arm_encoder.GetDistance()) > (position-2))&&(abs(arm_encoder.GetDistance()) < (position+2))){
      positionSpeed = .05;
      arm_angle.Set(0);
    }
    }
  }


  bool score(){
    while(running){
    while(score1){
    if(!score1){
      positionSpeed = .4;
      score1 = true;
    }
    if(score1){
      if(abs(arm_encoder.GetDistance()) < 168){
        arm_angle.Set(positionSpeed);
      }else if(abs(arm_encoder.GetDistance()) > 172){
        arm_angle.Set(-positionSpeed);
      }else if((abs(arm_encoder.GetDistance()) > 168)&&(abs(arm_encoder.GetDistance()) < 172)){
        positionSpeed = .05;
        arm_angle.Set(0);
        score1 = false;
      }
    }
    }
    m_intake.Set(-1);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    m_intake.Set(0);
    //GrabberArm(0,0,0,0,1);
    running = false;
    }
    stage1 = true;
    return true;
  }
  //****************************************************************    Claw pneumatics loop
  void ClawPosition(int toggleState){
    if(toggleState){
      if(opened){
        p_solenoidA.Set(frc::DoubleSolenoid::Value::kReverse);
        opened=0;
      }else{
        p_solenoidA.Set(frc::DoubleSolenoid::Value::kForward);
        opened=1;
      }
    }
  }

  void intake(int in, int out){
    m_intake.Set(in-out);
  }

  void autoLevel(){
    if(derivativeState){
      timer.Reset();
      angleDERIVATIVE = ahrs->GetPitch();
      timer.Start();
      derivativeState = false;
    }else{
      angleDERIVATIVE2 = ahrs->GetPitch();
      derivative = ((angleDERIVATIVE2 - angleDERIVATIVE)/((double)timer.Get()));
      derivativeState = true;
   }

    double angle = ahrs->GetPitch();
    double kp = 7.5;
    int kd = abs(derivative);
    ki += angle;


    if(testing == true){
      AutoDrive(-1,0,.01,250);
    }

    if((angle >= 10)&&((angle <= 13))){
      testing = false;
      engaged = true;
    }

    if(engaged == true){
      speed = (abs(angle*kp) - kd + (.002*ki));
      if(speed >= 200){
        speed = 200;
      }

      if(angle > 0){
        AutoDrive(-1,0,.01,speed);
        ki = 0;
      }

      if(angle < 0){
        AutoDrive(1,0,.01,speed);
      }
      if((angle < -.2)&&(angle > .2)){
        AutoDrive(0,0,0,0);
      }
    }
  }

//************************************************************************************************    Variables
  private:
  //****************************************************************    CONTROLLERS
    frc::XboxController m_driverController{1};
    frc::Joystick m_Joystick{0};
    
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

    rev::CANSparkMax arm_extend{20, rev::CANSparkMax::MotorType::kBrushless};
    //rev::CANSparkMax m_intake{21, rev::CANSparkMax::MotorType::kBrushless};
    WPI_TalonFX arm_angle{13};
    WPI_TalonFX m_intake{15};

    frc::DoubleSolenoid p_solenoidA{14, frc::PneumaticsModuleType::CTREPCM, 4, 5};
    //frc::Solenoid downSolenoid{frc::PneumaticsModuleType::CTREPCM, 4};
    //frc::Solenoid upSolenoid{frc::PneumaticsModuleType::CTREPCM, 5};
    frc::Compressor p_compressor{14, frc::PneumaticsModuleType::CTREPCM};

    AHRS *ahrs;

    frc::Timer timer;
    frc::Encoder arm_encoder{7, 8};

  //****************************************************************    SWERVE DRIVE VARIABLES
    //****************    Conversion Factor
    double conversionFactor = 4096.0/ 360.0;

    //****************    Frame Ratios
    //wheelbase (from center of front wheel to center of back wheel)
    double L = 25.75;
    double W = 25.75;
    double R = sqrt((L * L) + (W * W));

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
    double positionSpeed = .3;

    //****************    Pneumatics
    bool opened;
    int resetTimer = 0;
    bool timeTest = false;
    bool testing = true;
    bool engaged = false;
    bool delay = false;

    int additional = 0;

    bool score1 = true;
    bool running = true;
    bool stage0 = false;
    bool stage1 = false;
    bool stage2 = false;
    bool derivativeState = true;
    double angleDERIVATIVE;
    double angleDERIVATIVE2;
    double derivative;
    double ki = 0;
    double speed = 0;

    frc::DigitalInput insideSwitch {0};
    frc::DigitalInput armSwitch {1};
    
  };