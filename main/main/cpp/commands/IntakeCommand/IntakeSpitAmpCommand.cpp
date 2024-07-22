// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCommand/IntakeSpitAmpCommand.h"
#include "subsystems/ShooterSubsystem.h"
#include "Constants.h" //IntakeConstants

IntakeSpitAmpCommand::IntakeSpitAmpCommand(IntakeSubsystem& intakeSubsystem,ShooterSubsystem& shooterSubsystem):m_intakeSubsystem(&intakeSubsystem){}

// Called when the command is initially scheduled.
void IntakeSpitAmpCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IntakeSpitAmpCommand::Execute() {
  m_intakeSubsystem->spitAmp(IntakeConstants::intake_SpitAmpSpeedConstant);
}

// Called once the command ends or is interrupted.
void IntakeSpitAmpCommand::End(bool interrupted) {
  m_intakeSubsystem->stop();
}

// Returns true when the command should end.
bool IntakeSpitAmpCommand::IsFinished() {
  return false;
}
