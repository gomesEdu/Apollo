#pragma once

#include<frc/Joystick.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>

#include <frc2/command/button/CommandXboxController.h>
#include "frc2/command/button/JoystickButton.h"
#include "frc2/command/Command.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>


#include "Constants.h"

//Subsystems
#include "subsystems/ArmSubsystem.h"
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/ControlSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LedRgbSubsystem.h"
#include "subsystems/LimeLightSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/SwerveSubsystem.h"

//Auto Commands Library;
#include "commands/SwerveAutonomousCommand.h"

//Commands
#include "commands/SwerveCommand.h"
#include "commands/IntakeCommand/ArmToGroundCommand.h"
#include "commands/IntakeCommand/ArmToShooterCommand.h"
#include "commands/ClimberCommand/climberLeftDownCommand.h"
#include "commands/ClimberCommand/climberRightDownCommand.h"
#include "commands/ClimberCommand/climberLeftuPCommand.h"
#include "commands/ClimberCommand/climberRightUpCommand.h"

class RobotContainer {
 public:
  RobotContainer();
  //double armIntakeEncoder_Gvar;

  frc2::CommandPtr GetAutonomousCommand();
    void AutonomousInit();
    void TeleopInit();
    void SelectAuto();

 private:
  double Catch_Speed;
  double Spit_Speed;

  // Xbox Controller
  frc2::CommandXboxController m_TopController{OperatorConstants::kTopControllerPort};
  frc2::CommandXboxController m_DriverController{OperatorConstants::kDriverControllerPort};

  //Robot SUBSYSTEMS Objects
  ArmSubsystem m_armSubSystem;
  ClimberSubsystem m_climberSubsystem;
  ControlSubsystem Control;
  IntakeSubsystem m_intakeSubsystem;
  LedRgbSubsystem m_ledRgbSubsystem;
  //LimeLightSubsystem m_limeLightSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  SwerveSubsystem swerve_Subsystem;
  SwerveCommand Command{&swerve_Subsystem, &Control, Control.GetFieldOriented()};
  //IntakeCatchCommand autoIntakeCatchCommand{m_intakeSubsystem};

  void ConfigureBindings();

  int autoValue = 0;
};
