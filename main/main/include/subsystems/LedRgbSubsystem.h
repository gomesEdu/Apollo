#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include <frc/AddressableLED.h>

class LedRgbSubsystem : 
    public frc2::SubsystemBase{
        public:
            LedRgbSubsystem();
        
        void SetColor(int8_t color);

        void Periodic() override;

        void Init();

    private:
        static constexpr int8_t ledLength = 117; //Quantidade de LEDs instalados no Apollo
        frc::AddressableLED m_led{8}; //Porta PWM8
        std::array<frc::AddressableLED::LEDData, ledLength>
            m_ledBuffer;
            int8_t firstPixelHue = 0;
    };