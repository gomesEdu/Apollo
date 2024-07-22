// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShooterCommand/SourceCatchCommand.h"
#include "Constants.h"


SourceCatchCommand::SourceCatchCommand(ShooterSubsystem& shooterSubsystem, IntakeSubsystem& intakeSubsystem):m_shooterSubsystem(&shooterSubsystem), m_intakeSubsystem(&intakeSubsystem){}

void SourceCatchCommand::Initialize() {}

void SourceCatchCommand::Execute() {
  m_shooterSubsystem->shoot(ShooterConstants::source_CatchSpeedConstant);
}

void SourceCatchCommand::End(bool interrupted) {
  m_shooterSubsystem->stop();
}

bool SourceCatchCommand::IsFinished() {
  if(m_intakeSubsystem->noteDetect()==true){
    return true;
  }
  else{
    return false;
  }
}
