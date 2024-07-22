// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"
#include <frc/shuffleboard/Shuffleboard.h>

#include<iostream>


IntakeSubsystem::IntakeSubsystem()
{
intakeMotor.SetSmartCurrentLimit(0,35);
intakeMotor.BurnFlash();
intakeMotor.SetInverted(false);
}

void IntakeSubsystem::Periodic() {
   frc::SmartDashboard::PutBoolean("Note INTAKE", intakeNoteSensor.Get());
}


void IntakeSubsystem::catchNote(double catch_Speed){
   CatchSpeed = catch_Speed;
   intakeMotor.Set(-IntakeFilter.Calculate(CatchSpeed));
}

void IntakeSubsystem::spitAmp(double spit_AmpSpeed){
   SpitAmpSpeed = spit_AmpSpeed;
   intakeMotor.Set(IntakeFilter.Calculate(SpitAmpSpeed));
}

void IntakeSubsystem::spitSpeaker(double spit_SpeakerSpeed){
   SpitSpeakerSpeed = spit_SpeakerSpeed;
   intakeMotor.Set(IntakeFilter.Calculate(SpitSpeakerSpeed));
}

void IntakeSubsystem::stop(){
   intakeMotor.Set(0.0);
}

void IntakeSubsystem::keep(){
   intakeMotor.Set(0.005);
}

bool IntakeSubsystem::noteDetect(){
   //std::cout<<"Sensor da Nota= "<<intakeNoteSensor.Get();
   if(intakeNoteSensor.Get()==true){
      return true;
   }
   else{
      return false;
   }
} 

 bool  IntakeSubsystem::reachedMaxSpeed(){
   if(intakeEncoder.GetVelocity()>=5000){
      return true;
   }
   else{
      return false;
   }
 }
