// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ControlSubsystem.h"
#include <iostream>

ControlSubsystem::ControlSubsystem() = default;

// This method will be called once per scheduler run
void ControlSubsystem::Periodic() {frc::SmartDashboard::PutBoolean("FieldOriented", GetFieldOriented());
                                    //frc::SmartDashboard::PutValue("Speed", GetSpeed());
    if(joy1.GetRawButton(9)==true || joy1.GetRawButton(10)==true){
        kSpeed = 0.25;
        //std::cout<<"SPEED = 0.5"<<std::endl;//me
    }else if(joy1.GetRawButton(9)==false && joy1.GetRawButton(10)==false){
        kSpeed = 0.7;
        //std::cout<<"SPEED = 1"<<std::endl;//me
    }
    }
                                    

double ControlSubsystem::GetDriveX(){ return -joy1.GetRawAxis(1)*kSpeed; }

double ControlSubsystem::GetDriveY(){ return joy1.GetRawAxis(0)*kSpeed; }

double ControlSubsystem::GetRotX(){ return -joy1.GetRawAxis(4)*kSpeed; }

bool ControlSubsystem::GetFieldOriented(){ 
    
    if(joy1.GetRawButton(7)){
        kField = false;
    }
    else if(joy1.GetRawButton(8)){
        kField = true;
    }
    return kField;
}

double ControlSubsystem::GetSpeed(){
    /*if(joy1.GetRawButton(3)==true){
        kSpeed = 0.5;
    }else if(joy1.GetRawButton(3)==false){
        kSpeed = 1.0;
    }
    return kSpeed;*/
}