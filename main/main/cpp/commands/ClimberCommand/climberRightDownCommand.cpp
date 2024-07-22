// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimberCommand/climberRightDownCommand.h"

climberRightDownCommand::climberRightDownCommand(ClimberSubsystem& climberSubsystem) : m_climberSubsystem(&climberSubsystem){}

void climberRightDownCommand::Initialize() {}

void climberRightDownCommand::Execute() {
  m_climberSubsystem->moveRightDown();
}

void climberRightDownCommand::End(bool interrupted) {
  m_climberSubsystem->stopRight();
}

bool climberRightDownCommand::IsFinished() {
  return m_climberSubsystem->rightReachedDown();
}