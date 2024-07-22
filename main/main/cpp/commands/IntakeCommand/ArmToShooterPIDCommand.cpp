// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCommand/ArmToShooterPIDCommand.h"
#include "Constants.h"

ArmToShooterPIDCommand::ArmToShooterPIDCommand(ArmSubsystem& armSubsystem) : m_armSubSystem(&armSubsystem) {
}

// Called when the command is initially scheduled.
void ArmToShooterPIDCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ArmToShooterPIDCommand::Execute() {
  if(m_armSubSystem->reachedShooterPIDLimit()==false){
    m_armSubSystem->moveToShooterPID();
  }else if(m_armSubSystem->reachedShooterPIDLimit()==true){
    m_armSubSystem->moveUp();
  }
}

void ArmToShooterPIDCommand::End(bool interrupted) {
  m_armSubSystem->stop();
}

// Returns true when the command should end.
bool ArmToShooterPIDCommand::IsFinished() {
  return m_armSubSystem->reachedShooter();
}