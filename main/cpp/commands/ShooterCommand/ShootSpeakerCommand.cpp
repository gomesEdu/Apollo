// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShooterCommand/ShootSpeakerCommand.h"
#include "Constants.h"

ShootSpeakerCommand::ShootSpeakerCommand(ShooterSubsystem& shooterSubsystem):m_shooterSubsystem(&shooterSubsystem){}

void ShootSpeakerCommand::Initialize() {}

void ShootSpeakerCommand::Execute() {
  m_shooterSubsystem->shoot(ShooterConstants::speaker_ShootSpeedConstant);
}

void ShootSpeakerCommand::End(bool interrupted) {
  m_shooterSubsystem->stop();
}

bool ShootSpeakerCommand::IsFinished() {
  return false;
}
