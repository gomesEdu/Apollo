#include "subsystems/LedRgbSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

LedRgbSubsystem::LedRgbSubsystem(){
    //m_led.SetLength(ledLength);
   // m_led.SetData(m_ledBuffer);
    //m_led.Start();
    LedRgbSubsystem::Init();

}

void LedRgbSubsystem::Init(){
    m_led.SetLength(ledLength);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
}

void LedRgbSubsystem::Periodic(){}

void LedRgbSubsystem::SetColor(int8_t color){
    int8_t rgbColor[3] = {0,0,0}; //RED, GREEN, BLUE - 0 to 255
    frc::SmartDashboard::PutNumber("COR", color);

    switch(color){
        case ledRgbConstants::RED:
            rgbColor[ledRgbConstants::RED]=255;
            rgbColor[ledRgbConstants::GREEN]=0;
            rgbColor[ledRgbConstants::BLUE]=0;
            break;
        case ledRgbConstants::GREEN:
            rgbColor[ledRgbConstants::RED]=0;
            rgbColor[ledRgbConstants::GREEN]=255;
            rgbColor[ledRgbConstants::BLUE]=0;
            break;
        case ledRgbConstants::BLUE:
            rgbColor[ledRgbConstants::RED]=0;
            rgbColor[ledRgbConstants::GREEN]=0;
            rgbColor[ledRgbConstants::BLUE]=255;
            break;
        case ledRgbConstants::MAGENTA:
            rgbColor[ledRgbConstants::RED]=240;
            rgbColor[ledRgbConstants::GREEN]=20;
            rgbColor[ledRgbConstants::BLUE]=120;
            break;
        case ledRgbConstants::PINK:
            rgbColor[ledRgbConstants::RED]=255;
            rgbColor[ledRgbConstants::GREEN]=0;
            rgbColor[ledRgbConstants::BLUE]=110;
            break;
        case ledRgbConstants::OFF:
            rgbColor[ledRgbConstants::RED]=0;
            rgbColor[ledRgbConstants::GREEN]=0;
            rgbColor[ledRgbConstants::BLUE]=0;
            break;
        case ledRgbConstants::ORANGE:
            rgbColor[ledRgbConstants::RED]=255;
            rgbColor[ledRgbConstants::GREEN]=40;
            rgbColor[ledRgbConstants::BLUE]=0;
            break;
    }

    for(int i=0; i<ledLength; i++){
        m_ledBuffer[i].SetRGB(rgbColor[ledRgbConstants::RED],
                             rgbColor[ledRgbConstants::GREEN],
                             rgbColor[ledRgbConstants::BLUE]); 
    }
    m_led.SetData(m_ledBuffer);
}