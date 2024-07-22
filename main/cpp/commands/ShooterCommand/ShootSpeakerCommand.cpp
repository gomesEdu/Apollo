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
  if(m_shooterSubsystem->noteDetect()==false){
    return true;
  }
  else{
    return false;
  }
}