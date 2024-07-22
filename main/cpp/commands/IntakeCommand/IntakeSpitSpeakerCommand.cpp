// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCommand/IntakeSpitSpeakerCommand.h"
#include "subsystems/ShooterSubsystem.h"
#include "Constants.h" //IntakeConstants

IntakeSpitSpeakerCommand::IntakeSpitSpeakerCommand(IntakeSubsystem& intakeSubsystem,ShooterSubsystem& shooterSubsystem):m_intakeSubsystem(&intakeSubsystem),m_shooterSubsystem(&shooterSubsystem){}

// Called when the command is initially scheduled.
void IntakeSpitSpeakerCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IntakeSpitSpeakerCommand::Execute() {
  if(m_shooterSubsystem->reachedSpeakerShootSpeed()==true){
  m_intakeSubsystem->spitSpeaker(IntakeConstants::intake_SpitSpeakerSpeedConstant);
  }
}

// Called once the command ends or is interrupted.
void IntakeSpitSpeakerCommand::End(bool interrupted) {
  m_intakeSubsystem->stop();
}

// Returns true when the command should end.
bool IntakeSpitSpeakerCommand::IsFinished() {
  if(m_intakeSubsystem->noteDetect()==false){
    return true;
  }
  else{
    return false;
  }
}
