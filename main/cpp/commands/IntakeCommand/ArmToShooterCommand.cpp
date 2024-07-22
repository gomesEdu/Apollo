// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCommand/ArmToShooterCommand.h"

ArmToShooterCommand::ArmToShooterCommand(ArmSubsystem& armSubsystem) : m_armSubSystem(&armSubsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ArmToShooterCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ArmToShooterCommand::Execute() {
  m_armSubSystem->moveUp();
}

// Called once the command ends or is interrupted.
void ArmToShooterCommand::End(bool interrupted) {
  m_armSubSystem->holdShooterPosition();
}

// Returns true when the command should end.
bool ArmToShooterCommand::IsFinished() {
  return m_armSubSystem->reachedShooter();
}