#include "commands/IntakeCommand/ArmToGroundCommand.h"

ArmToGroundCommand::ArmToGroundCommand(ArmSubsystem& armSubsystem) : m_armSubSystem(&armSubsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ArmToGroundCommand::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void ArmToGroundCommand::Execute() {
  m_armSubSystem->moveDown();
}

// Called once the command ends or is interrupted.
void ArmToGroundCommand::End(bool interrupted) {
  m_armSubSystem->stop();
}

// Returns true when the command should end.
bool ArmToGroundCommand::IsFinished() {
  return m_armSubSystem->reachedGround();
}
