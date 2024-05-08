// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCommand/IntakeSpitSpeakerCommand.h"
#include "Constants.h" //IntakeConstants

IntakeSpitSpeakerCommand::IntakeSpitSpeakerCommand(IntakeSubsystem& intakeSubsystem):m_intakeSubsystem(&intakeSubsystem){}

// Called when the command is initially scheduled.
void IntakeSpitSpeakerCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IntakeSpitSpeakerCommand::Execute() {
  m_intakeSubsystem->spitSpeaker(IntakeConstants::intake_SpitSpeakerSpeedConstant);
}

// Called once the command ends or is interrupted.
void IntakeSpitSpeakerCommand::End(bool interrupted) {
  m_intakeSubsystem->stop();
}

// Returns true when the command should end.
bool IntakeSpitSpeakerCommand::IsFinished() {
  return false;
}
