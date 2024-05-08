// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"
#include "iostream"
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>

ShooterSubsystem::ShooterSubsystem(){
shooterLeftMotor.SetSmartCurrentLimit(0,35);
shooterLeftMotor.BurnFlash();
shooterLeftMotor.SetInverted(false);
shooterRightMotor.SetSmartCurrentLimit(0,35);
shooterRightMotor.BurnFlash();
shooterRightMotor.SetInverted(true);
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {
}

void ShooterSubsystem::shoot(double speed){
      shooterLeftMotor.Set(speed);
      shooterRightMotor.Set(speed);
}

void ShooterSubsystem::stop(){
      shooterLeftMotor.Set(0);
      shooterRightMotor.Set(0);
}
