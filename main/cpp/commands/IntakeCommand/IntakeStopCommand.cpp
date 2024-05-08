// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCommand/IntakeStopCommand.h"

IntakeStopCommand::IntakeStopCommand(IntakeSubsystem& intakeSubsystem):m_intakeSubsystem(&intakeSubsystem){}

// Called when the command is initially scheduled.
void IntakeStopCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IntakeStopCommand::Execute() {
  m_intakeSubsystem->stop();
}

// Called once the command ends or is interrupted.
void IntakeStopCommand::End(bool interrupted) {
  m_intakeSubsystem->stop();
}

// Returns true when the command should end.
bool IntakeStopCommand::IsFinished() {
  return false;
}
