#pragma once

#include "subsystems/ShooterSubsystem.h"
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>


class ShootSpeakerCommand
  : public frc2::CommandHelper<frc2::Command, ShootSpeakerCommand> {
 public:
  explicit ShootSpeakerCommand(ShooterSubsystem& ShooterSubsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  ShooterSubsystem*m_shooterSubsystem;
};
