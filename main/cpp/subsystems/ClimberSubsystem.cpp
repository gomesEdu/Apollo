#include "subsystems/ClimberSubsystem.h"
#include "Constants.h"
#include <iostream>


ClimberSubsystem::ClimberSubsystem()
{
climberLeftMotor.SetSmartCurrentLimit(0,35);
climberLeftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
climberLeftMotor.SetInverted(true);
climberLeftMotor.BurnFlash();

climberRightMotor.SetSmartCurrentLimit(0,35);
climberRightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
climberRightMotor.SetInverted(true);
climberRightMotor.BurnFlash();

ClimberSubsystem::resetRightEncoder();
ClimberSubsystem::resetLeftEncoder();
}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}

void ClimberSubsystem::moveGroupUp(){
   climberLeftMotor.Set(0.5);
   climberRightMotor.Set(0.5);
}
void ClimberSubsystem::moveGroupDown(){
   climberLeftMotor.Set(-0.5);
   climberRightMotor.Set(-0.5);
}
void ClimberSubsystem::stopGroup(){
   climberLeftMotor.Set(0.0);
   climberRightMotor.Set(0.0);
}
void ClimberSubsystem::holdGroupUp(){
   //climberMotorGroup.Set(-0.02);
}

bool ClimberSubsystem::groupReachedUp(){
   int leftPos = climberLeftEncoder.GetPosition();
   //int leftPos = 5;
   int rightPos = climberRightEncoder.GetPosition();
   //std::cout<<"\n\nLEFT CLIMBER = "<<leftPos<<"   RIGHT CLIMBER= "<<rightPos;
   if(leftPos>=44 || rightPos>=44){
      return true;
   }
   else{
      return false;
   }
}

bool ClimberSubsystem::groupReachedDown(){
   int leftPos = climberLeftEncoder.GetPosition();
   //int leftPos = 5;
   int rightPos = climberRightEncoder.GetPosition();
   //std::cout<<"\n\nLEFT CLIMBER = "<<leftPos<<"   RIGHT CLIMBER= "<<rightPos;
   if(leftPos<=0 || rightPos<=0){
      return true;
   }
   else{
      return false;
   }
}

void ClimberSubsystem::resetLeftEncoder(){
   climberLeftEncoder.SetPosition(0);
}
void ClimberSubsystem::resetRightEncoder(){
   climberRightEncoder.SetPosition(0);
}

void ClimberSubsystem::moveRightUp(){
   climberRightMotor.Set(0.5);
}
void ClimberSubsystem::moveRightDown(){
   climberRightMotor.Set(-0.5);
}
void ClimberSubsystem::stopRight(){
   climberRightMotor.Set(0.0);
}
void ClimberSubsystem::holdRightDown(){
   climberRightMotor.Set(-0.05);
}
bool ClimberSubsystem::rightReachedUp(){
   int rightPos = climberRightEncoder.GetPosition();
   //std::cout<<"\nRIGHT CLIMBER= "<<rightPos;
   if(rightPos>=44){
      return true;
   }
   else{
      return false;
   }
}
bool ClimberSubsystem::rightReachedDown(){
   int rightPos = climberRightEncoder.GetPosition();
   //std::cout<<"\nRIGHT CLIMBER= "<<rightPos;
   if(rightPos<=0){
      return true;
   }
   else{
      return false;
   }
}

void ClimberSubsystem::moveLeftUp(){
   climberLeftMotor.Set(0.5);
}
void ClimberSubsystem::moveLeftDown(){
   climberLeftMotor.Set(-0.5);
}
void ClimberSubsystem::stopLeft(){
   climberLeftMotor.Set(0.0);
}
void ClimberSubsystem::holdLeftDown(){
   climberLeftMotor.Set(-0.05);
}
bool ClimberSubsystem::leftReachedUp(){
   int leftPos = climberLeftEncoder.GetPosition();
   //std::cout<<"\nLEFT CLIMBER= "<<leftPos;
   if(leftPos>=45.2){
      return true;
   }
   else{
      return false;
   }
}
bool ClimberSubsystem::leftReachedDown(){
   int leftPos = climberLeftEncoder.GetPosition();
   //std::cout<<"\nLEFT CLIMBER= "<<leftPos;
   if(leftPos<=0){
      return true;
   }
   else{
      return false;
   }
}