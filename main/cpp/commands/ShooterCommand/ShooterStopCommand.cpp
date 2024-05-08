// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShooterCommand/ShooterStopCommand.h"

ShooterStopCommand::ShooterStopCommand(ShooterSubsystem& shooterSubsystem):m_shooterSubsystem(&shooterSubsystem){}

// Called when the command is initially scheduled.
void ShooterStopCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShooterStopCommand::Execute() {
  m_shooterSubsystem->stop();
}

// Called once the command ends or is interrupted.
void ShooterStopCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool ShooterStopCommand::IsFinished() {
  return false;
}
