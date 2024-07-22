// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>


//The subsystem library which we're going to use
#include <subsystems/SwerveSubsystem.h>

//Function library - C++
#include <functional>

//Default C++
#include <iostream>
#include <algorithm>

//Filter
#include <frc/filter/SlewRateLimiter.h>

//ChassisSpeed library
#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/kinematics/SwerveModuleState.h>

//control librarie
#include "subsystems/ControlSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SwerveCommand
    : public frc2::CommandHelper<frc2::Command, SwerveCommand> {
 public:

  //The default class ctor, the parameter classes is the function for spds and FieldOriented
  explicit SwerveCommand(SwerveSubsystem*swerve_SubsystemParameter, 
                                          ControlSubsystem*control_SubsystemParameter,
                                          bool FieldOriented_Parameter);
  
 

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private :

  //The subsystem wich we're going to use
  SwerveSubsystem* swerve_Subsystem;

  ControlSubsystem* control;

  bool FieldOriented;

//The limiter for the max X axis speed
frc::SlewRateLimiter <units::scalar> xAxisLimiter 
{DriveConstants::kTeleDriveMaxAccelerationUnitsPerSecond/1.0_s};

   //The limiter for the max Y axis speed
frc::SlewRateLimiter <units::scalar> yAxisLimiter
{DriveConstants::kTeleDriveMaxAccelerationUnitsPerSecond/1.0_s};

    //The limiter for the max rotation speed
frc::SlewRateLimiter <units::scalar> rAxisLimiter 
{DriveConstants::kTeleDriveMaxAngularAccelerationUnitsPerSecond/1.0_s};

};