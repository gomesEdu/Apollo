// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/ShooterSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>


class SourceCatchCommand
    : public frc2::CommandHelper<frc2::Command, SourceCatchCommand> {
 public:
  explicit SourceCatchCommand(ShooterSubsystem& ShooterSubsystem, IntakeSubsystem& IntakeSubsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  ShooterSubsystem*m_shooterSubsystem;
  IntakeSubsystem*m_intakeSubsystem;
};

