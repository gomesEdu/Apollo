// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

//Driver Library
#include "rev/CanSparkMax.h"

//MotorGroup Library
#include "frc/motorcontrol/MotorControllerGroup.h"

#include "frc/filter/SlewRateLimiter.h"

#include "frc/filter/MedianFilter.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void  shoot(double speed);
  void  stop();

 private:
  //Motors and Encoders
  rev::CANSparkMax shooterLeftMotor{11,rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkRelativeEncoder shooterLeftEncoder = shooterLeftMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::CANSparkMax shooterRightMotor{12,rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkRelativeEncoder shooterRightEncoder = shooterRightMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

  //Speed for Speaker
  double SpeakerSpeed;
  double CatchtSpeed;
  double AmplifierSpeed;
  
  frc::SlewRateLimiter<units::dimensionless::scalar> ShooterFilter{0.1 / 20_ms};
};
