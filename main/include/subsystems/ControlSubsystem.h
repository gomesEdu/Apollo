// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/XboxController.h>
#include <frc2/command/button/JoystickButton.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "Constants.h"

#include <frc/Joystick.h>

class ControlSubsystem : public frc2::SubsystemBase {
 public:
  ControlSubsystem();

  //Get the Left X axis of  XboxController 
  double GetDriveX();

  //Get the Left Y axis of  XboxController 
  double GetDriveY();

  //Get the Right X axis of  XboxController
  double GetRotX();

  //Method which returns the field Oriented condition , in function with right and left bumpers 
  bool GetFieldOriented();

  double GetSpeed();



  //Field Oriented Condition
  bool kField;

  //Speed Control
  double kSpeed;
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::XboxController xbox{OperatorConstants::kDriverControllerPort};
  frc::Joystick joy1{0};
  frc::Joystick joy2{1};


 // frc2::JoystickButton aButton{&xbox, frc::XboxController::Button::kA};
};
