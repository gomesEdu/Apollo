// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimberCommand/climberLeftUpCommand.h"

climberLeftUpCommand::climberLeftUpCommand(ClimberSubsystem& climberSubsystem) : m_climberSubsystem(&climberSubsystem){}

void climberLeftUpCommand::Initialize() {}

void climberLeftUpCommand::Execute() {
  m_climberSubsystem->moveLeftUp();
}

void climberLeftUpCommand::End(bool interrupted) {
  m_climberSubsystem->stopLeft();
}

bool climberLeftUpCommand::IsFinished() {
  return m_climberSubsystem->leftReachedUp();
}