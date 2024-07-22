// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

//Driver Library
#include "rev/CanSparkMax.h"

#include <frc/DigitalInput.h>
#include "frc/filter/SlewRateLimiter.h"

#include "frc/filter/MedianFilter.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  void Periodic() override;

  void  spitSpeaker(double SpitSpeed_Parameter);
  void  spitAmp(double SpitSpeed_Parameter);
  void  catchNote(double CatchSpeed_Parameter);
  void  stop();
  void  keep();
  bool  noteDetect();
  bool  reachedMaxSpeed();

 private:
  //Creating the drivers Objects
  rev::CANSparkMax intakeMotor{13,rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkRelativeEncoder intakeEncoder = intakeMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

  frc::DigitalInput intakeNoteSensor{8};

  //Catch SPEED
  double CatchSpeed;

  //Spit SPEED
  double SpitSpeakerSpeed;
  double SpitAmpSpeed;
  
  frc::SlewRateLimiter<units::dimensionless::scalar> IntakeFilter{0.1 / 20_ms};
};
