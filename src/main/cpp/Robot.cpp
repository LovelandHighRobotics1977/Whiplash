// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {
  m_rra.SetSensorPhase(true);
  m_fra.SetSensorPhase(true);
  m_rla.SetSensorPhase(true);
  m_fla.SetSensorPhase(true);
  m_rra.SetNeutralMode(NeutralMode::Brake);
  m_fra.SetNeutralMode(NeutralMode::Brake);
  m_rla.SetNeutralMode(NeutralMode::Brake);
  m_fra.SetNeutralMode(NeutralMode::Brake);




  //********************************Configure Rear Right Angle Motor********************************
  m_rra.ConfigFactoryDefault();
  m_rra.ConfigRemoteFeedbackFilter(5, RemoteSensorSource(13), 0, 0);
  m_rra.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0); // PIDLoop=0, timeoutMs=0

  m_rra.Config_kP(0, 1.7);
  m_rra.Config_kI(0, .0016);
  m_rra.Config_kD(0, 160);
  m_rra.Config_kF(0, 0);
  m_rra.Config_IntegralZone(0, 20);

  m_rra.ConfigNominalOutputForward(0);
	m_rra.ConfigNominalOutputReverse(0);
	m_rra.ConfigPeakOutputForward(1);
	m_rra.ConfigPeakOutputReverse(-1);




  //********************************Configure Front Right Angle Motor*******************************
  m_fra.ConfigFactoryDefault();
  m_fra.ConfigRemoteFeedbackFilter(2, RemoteSensorSource(13), 0, 0);
  m_fra.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0); // PIDLoop=0, timeoutMs=0

  m_fra.Config_kP(0, 1.7);
  m_fra.Config_kI(0, .0016);
  m_fra.Config_kD(0, 160);
  m_fra.Config_kF(0, 0);
  m_fra.Config_IntegralZone(0, 20);

  m_fra.ConfigNominalOutputForward(0);
	m_fra.ConfigNominalOutputReverse(0);
	m_fra.ConfigPeakOutputForward(1);
	m_fra.ConfigPeakOutputReverse(-1);




  //********************************Configure Rear Left Angle Motor*********************************
  m_rla.ConfigFactoryDefault();
  m_rla.ConfigRemoteFeedbackFilter(1, RemoteSensorSource(13), 0, 0);
  m_rla.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0); // PIDLoop=0, timeoutMs=0

  m_rla.Config_kP(0, 1.7);
  m_rla.Config_kI(0, .0016);
  m_rla.Config_kD(0, 160);
  m_rla.Config_kF(0, 0);
  m_rla.Config_IntegralZone(0, 20);

  m_rla.ConfigNominalOutputForward(0);
	m_rla.ConfigNominalOutputReverse(0);
	m_rla.ConfigPeakOutputForward(1);
	m_rla.ConfigPeakOutputReverse(-1);




  //********************************Configure Front Left Angle Motor********************************
  m_fla.ConfigFactoryDefault();
  m_fla.ConfigRemoteFeedbackFilter(3, RemoteSensorSource(13), 0, 0);
  m_fla.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0); // PIDLoop=0, timeoutMs=0

  m_fla.Config_kP(0, 1.7);
  m_fla.Config_kI(0, .0016);
  m_fla.Config_kD(0, 160);
  m_fla.Config_kF(0, 0);
  m_fla.Config_IntegralZone(0, 20);

  m_fla.ConfigNominalOutputForward(0);
	m_fla.ConfigNominalOutputReverse(0);
	m_fla.ConfigPeakOutputForward(1);
	m_fla.ConfigPeakOutputReverse(-1);




  //********************************Configure Cancoder Magnet Offsets**************************
  m_flsensor.ConfigMagnetOffset(100);
  m_frsensor.ConfigMagnetOffset(110);
  m_rlsensor.ConfigMagnetOffset(250);
  m_rrsensor.ConfigMagnetOffset(148);
  m_flsensor.SetPositionToAbsolute();
  m_frsensor.SetPositionToAbsolute();
  m_rlsensor.SetPositionToAbsolute();
  m_rrsensor.SetPositionToAbsolute();




  //********************************Configure the gyro board********************************
  //ahrs = new AHRS(frc::SPI::Port::kMXP);    //if we use SPI to connect the gyro board, use this instead.        >>> SPI uses 6 wires, I2C uses 4 <<<
  ahrs = new AHRS(frc::I2C::Port::kMXP);

}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  //timer.Reset();
  //timer.Start();
}
void Robot::TeleopPeriodic() {
  double forward = -m_driverController.GetRightY();
  double strafe =  m_driverController.GetRightX();
  double rotate = m_driverController.GetLeftX();

  //********Controller Deadzones********
  if((forward<.2)&&(forward>-.2)){forward = 0;}
  if((strafe<.2)&&(strafe>-.2)){strafe = 0;}
  if((rotate<.2)&&(rotate>-.2)){rotate = 0;}
  std::cout<<"FWD0 "<< forward <<" STR0 "<< strafe <<" RCW0 "<< rotate<< std::endl;
  drive(forward, strafe, rotate);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
