#include "commands/IntakeCommand/ArmToGroundPIDCommand.h"
#include "Constants.h"
#include <iostream>
ArmToGroundPIDCommand::ArmToGroundPIDCommand(ArmSubsystem& armSubsystem) : m_armSubSystem(&armSubsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ArmToGroundPIDCommand::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void ArmToGroundPIDCommand::Execute() {
  if(m_armSubSystem->reachedGroundPIDLimit()==false){
    m_armSubSystem->moveToGroundnPID();
  }else if(m_armSubSystem->reachedGroundPIDLimit()==true){
    m_armSubSystem->moveDown();
  }
}

// Called once the command ends or is interrupted.
void ArmToGroundPIDCommand::End(bool interrupted) {
  m_armSubSystem->stop();
}

// Returns true when the command should end.
bool ArmToGroundPIDCommand::IsFinished() {
  std::cout<<"\n Ground FINISHED";
  return m_armSubSystem->reachedGround();
}
