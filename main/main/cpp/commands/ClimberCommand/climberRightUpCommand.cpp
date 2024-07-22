// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimberCommand/climberRightUpCommand.h"

climberRightUpCommand::climberRightUpCommand(ClimberSubsystem& climberSubsystem) : m_climberSubsystem(&climberSubsystem){}

void climberRightUpCommand::Initialize() {}

void climberRightUpCommand::Execute() {
  m_climberSubsystem->moveRightUp();
}

void climberRightUpCommand::End(bool interrupted) {
  m_climberSubsystem->stopRight();
}

bool climberRightUpCommand::IsFinished() {
  return m_climberSubsystem->rightReachedUp();
}