// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"



class LimeLightSubsystem : public frc2::SubsystemBase {
 public:
  LimeLightSubsystem(std::string LimeName_Parameter,std::shared_ptr<nt::NetworkTable> NetWorkTable_parameter, double Tx_Parameter
  , double Ty_Parameter, double Ta_Parameter, double TSkew_Parameter );

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  //Default LimeLight Method to catch Tx Image Horizontal (TX)
  double GetXhorizontal();

  //Default LimeLight Method to catch Ty Image Vertical (TY)
  double GetYvertical();

  //Default LimeLight Method to catch the Image Area (TA)
  double GetAarea();

  //Default LimeLight Method to catch the Error (TSkew)
  double GetError();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //The mainly actor NTtable Object
  std::shared_ptr<nt::NetworkTable> NetWorkTable;

  
  //Lime Target Values to be Initalized

  //Lime Target  Horizontal
  double Tx;
  //Lime Target Vertical
  double Ty;
  //Lime Target Area
  double Ta;
  //Lime Target Error
  double TSkew;

  //Lime's String Name
  std::string LimeLightName;
};
