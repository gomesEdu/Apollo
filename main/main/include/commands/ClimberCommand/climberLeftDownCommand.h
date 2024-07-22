// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ClimberSubsystem.h"

class climberLeftDownCommand
    : public frc2::CommandHelper<frc2::Command, climberLeftDownCommand> {
 public:
  explicit climberLeftDownCommand(ClimberSubsystem& climberSubsystem);
  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  ClimberSubsystem* m_climberSubsystem;
};
