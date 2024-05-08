// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimberCommand/climberLeftDownCommand.h"

climberLeftDownCommand::climberLeftDownCommand(ClimberSubsystem& climberSubsystem) : m_climberSubsystem(&climberSubsystem){}

void climberLeftDownCommand::Initialize() {}

void climberLeftDownCommand::Execute() {
  m_climberSubsystem->moveLeftDown();
}

void climberLeftDownCommand::End(bool interrupted) {
  m_climberSubsystem->stopLeft();
}

bool climberLeftDownCommand::IsFinished() {
  return m_climberSubsystem->leftReachedDown();
}