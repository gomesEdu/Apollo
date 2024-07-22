#include "commands/RGBCommands/LedRgbSetColorCommand.h"

LedRgbSetColorCommand::LedRgbSetColorCommand(LedRgbSubsystem& ledRgbSubsystem, int8_t color)
  //: m_ledRgbSubsystem(&ledRgbSubsystem){}
  : m_ledRgbSubsystem(&ledRgbSubsystem), m_color(color){}
  
void LedRgbSetColorCommand::Initialize() {}

void LedRgbSetColorCommand::Execute() {
  m_ledRgbSubsystem->SetColor(m_color);
}

void LedRgbSetColorCommand::End(bool interrupted) {}

bool LedRgbSetColorCommand::IsFinished() {
  return true;
}