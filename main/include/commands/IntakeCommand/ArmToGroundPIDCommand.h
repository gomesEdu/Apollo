// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ArmSubsystem.h"

class ArmToGroundPIDCommand : public frc2::CommandHelper<frc2::Command, ArmToGroundPIDCommand> {
 public:
  explicit ArmToGroundPIDCommand(ArmSubsystem& armSubsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  ArmSubsystem* m_armSubSystem;
};
