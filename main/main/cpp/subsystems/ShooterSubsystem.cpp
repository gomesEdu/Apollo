// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc/shuffleboard/Shuffleboard.h>

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
      frc::SmartDashboard::PutBoolean("Note SHOOTER", !shooterNoteSensor.Get());
      frc::SmartDashboard::PutNumber("Sp LsHOOTER", shooterLeftEncoder.GetVelocity());
      frc::SmartDashboard::PutNumber("Sp RsHOOTER", shooterRightEncoder.GetVelocity());
}

void ShooterSubsystem::shoot(double speed){
      shooterLeftMotor.Set(speed);
      shooterRightMotor.Set(speed);
      //std::cout<<"\nDIREITO"<<shooterLeftEncoder.GetVelocity();//max 5700
      //std::cout<<"\nESQUERDO"<<shooterLeftEncoder.GetVelocity();
}

void ShooterSubsystem::stop(){
      shooterLeftMotor.Set(0);
      shooterRightMotor.Set(0);
}

bool  ShooterSubsystem::reachedSpeakerShootSpeed(){
      if(shooterLeftEncoder.GetVelocity()>=5550 && shooterRightEncoder.GetVelocity()>=5550){
            return true;
      }
      else{
            return false;
      }
}

bool  ShooterSubsystem::reachedZeroSpeed(){
      if(shooterLeftEncoder.GetVelocity()<=100 && shooterRightEncoder.GetVelocity()<=100){
            return true;
      }
      else{
            return false;
      }
}

bool ShooterSubsystem::noteDetect(){
   if(!shooterNoteSensor.Get()==true){//Detected note at SHOOTER
      return true;
   }
   else{//NOT detected note at SHOOTER
      return false;
   }
}