
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


class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem();

  void Periodic() override;

  //Group Movements
  void moveGroupUp();
  void moveGroupDown();
  void stopGroup();
  void holdGroupUp();
  void holdGroupDown();

  //Right Movements
  void moveRightUp();
  void moveRightDown();
  void stopRight();
  void holdRightDown();

  //Left Movements
  void moveLeftUp();
  void moveLeftDown();
  void stopLeft();
  void holdLeftDown();

  //Encoders and positioning
  bool rightReachedUp();
  bool rightReachedDown();
  bool leftReachedUp();
  bool leftReachedDown();
  bool groupReachedUp();
  bool groupReachedDown();

  //rESET ENCODERS
  void resetLeftEncoder();
  void resetRightEncoder();

 private:
  //rev::CANSparkMax climberLeftMotor{15,rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax climberLeftMotor{7,rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax climberRightMotor{16,rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkRelativeEncoder climberLeftEncoder = climberLeftMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkRelativeEncoder climberRightEncoder = climberRightMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

  //frc::MotorControllerGroup climberMotorGroup{climberLeftMotor, climberRightMotor};

  
  frc::SlewRateLimiter<units::dimensionless::scalar> ClimberFilter{0.2 / 20_ms};

};
