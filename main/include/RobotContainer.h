// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include<frc/Joystick.h>
#include "Constants.h"

#include "subsystems/SwerveSubsystem.h"

#include "subsystems/IntakeSubsystem.h"

#include "subsystems/ArmSubsystem.h"

#include "subsystems/ShooterSubsystem.h"

#include "subsystems/ClimberSubsystem.h"

#include "frc2/command/Command.h"

#include "frc2/command/button/JoystickButton.h"

#include <frc/XboxController.h>

#include "subsystems/ControlSubsystem.h"

//Auto Commands Library;
#include "commands/SwerveAutonomousCommand.h"

//Command Library
#include "commands/SwerveCommand.h"

//COMANDOS NOVOS - 25/04/2024
#include "commands/IntakeCommand/ArmToGroundCommand.h"
#include "commands/IntakeCommand/ArmToShooterCommand.h"
#include "commands/ClimberCommand/climberLeftDownCommand.h"
#include "commands/ClimberCommand/climberRightDownCommand.h"
#include "commands/ClimberCommand/climberLeftuPCommand.h"
#include "commands/ClimberCommand/climberRightUpCommand.h"


#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>

#include <frc/Timer.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();
  double armIntakeEncoder_Gvar;

  frc2::CommandPtr GetAutonomousCommand();


    void AutonomousInit();
    void TeleopInit();
    void SelectAuto();


 private:

  // Replace with CommandPS4Controller or CommandJoystick if needed
  double Catch_Speed;
  double Spit_Speed;

 
  // Xbox Controller
  frc2::CommandXboxController m_TopController{OperatorConstants::kTopControllerPort};
  frc2::CommandXboxController m_DriverController{OperatorConstants::kDriverControllerPort};

  // The robot's subsystems are defined here...

  //XboxSubsystem Controller
  ControlSubsystem Control;  

  //SwerveSubsystem Object  
  SwerveSubsystem swerve_Subsystem;

  //intakeSubsystem Object  
  IntakeSubsystem m_intakeSubsystem;

  //Auto Command Library

  //ShooterSubsystem Object
  ShooterSubsystem m_shooterSubsystem;

  //armSubsystem Object
  ArmSubsystem m_armSubSystem;

  //armSubsystem Object
  ClimberSubsystem m_climberSubsystem;

  //sWERVE
  SwerveCommand Command{&swerve_Subsystem, &Control, Control.GetFieldOriented()};

  void ConfigureBindings();

  int autoValue = 0;
};
