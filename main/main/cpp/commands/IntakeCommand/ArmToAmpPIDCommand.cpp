#include "commands/IntakeCommand/ArmToAmpPIDCommand.h"
#include "Constants.h"

ArmToAmpPIDCommand::ArmToAmpPIDCommand(ArmSubsystem& armSubsystem) : m_armSubSystem(&armSubsystem) {
}

// Called when the command is initially scheduled.
void ArmToAmpPIDCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ArmToAmpPIDCommand::Execute() {
 // m_armSubSystem->moveToPositionPID(ArmConstants::armAtAmpPosition);
  m_armSubSystem->moveToAmpPID();
}

void ArmToAmpPIDCommand::End(bool interrupted) {
}

// Returns true when the command should end.
bool ArmToAmpPIDCommand::IsFinished() {
  return false;
}