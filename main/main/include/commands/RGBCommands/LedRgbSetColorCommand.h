#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/LedRgbSubsystem.h"

class LedRgbSetColorCommand : public frc2::CommandHelper<frc2::Command, LedRgbSetColorCommand>{
  public:
    explicit LedRgbSetColorCommand(LedRgbSubsystem& LedRgbSubsystem, int8_t color);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

    private:
      LedRgbSubsystem*m_ledRgbSubsystem;
      int8_t m_color;
};