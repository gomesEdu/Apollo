// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShooterCommand/ShootAmpCommand.h"
#include "Constants.h"

ShootAmpCommand::ShootAmpCommand(ShooterSubsystem& shooterSubsystem):m_shooterSubsystem(&shooterSubsystem){}


// Called when the command is initially scheduled.
void ShootAmpCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShootAmpCommand::Execute() {
  m_shooterSubsystem->shoot(ShooterConstants::amplifier_ShootSpeedConstant);
}

// Called once the command ends or is interrupted.
void ShootAmpCommand::End(bool interrupted) {
  m_shooterSubsystem->stop();
}

// Returns true when the command should end.
bool ShootAmpCommand::IsFinished() {
  return false;
}
