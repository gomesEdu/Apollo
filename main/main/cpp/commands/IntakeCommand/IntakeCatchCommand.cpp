// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCommand/IntakeCatchCommand.h"
#include "Constants.h" //IntakeConstants

IntakeCatchCommand::IntakeCatchCommand(IntakeSubsystem& intakeSubsystem): m_intakeSubsystem(&intakeSubsystem) {}

// Called when the command is initially scheduled.
void IntakeCatchCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IntakeCatchCommand::Execute() {
  m_intakeSubsystem->catchNote(IntakeConstants::intake_CatchSpeedConstant);  
}

// Called once the command ends or is interrupted.
void IntakeCatchCommand::End(bool interrupted) {
  m_intakeSubsystem->stop();
}

// Returns true when the command should end.
bool IntakeCatchCommand::IsFinished() {
  if(m_intakeSubsystem->noteDetect()==true){
    return true;
  }
  else{
    return false;
  }
  
}
