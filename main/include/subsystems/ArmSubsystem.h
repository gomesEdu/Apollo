#pragma once

//Driver Library
#include "rev/CanSparkMax.h"

//MotorGroup Library
#include "frc/motorcontrol/MotorControllerGroup.h"

#include "frc/filter/SlewRateLimiter.h"

#include "frc/filter/MedianFilter.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/DigitalInput.h>

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  void Periodic() override;

  //---------------------------------------------------
  //-----------Edu 9461 24/04/24-----------------------
  void moveUp();//Move o Braço do Intake em direção ao shooter, para cima
  void moveDown();//Move o Braço do Intake em direção ao chão, para baixo
  void stop();//Desliga o Braço do Intake
  void holdShooterPosition();
  void holdGroundPosition();
  void resetEncoder();//Seta o encoder em 0 voltas
  bool reachedShooter();
  bool reachedGround();
  bool reachedAmplifier();
  //---------------------------------------------------

 private:
  rev::CANSparkMax armMotor{14,rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkRelativeEncoder armEncoder = armMotor
                  .GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

   double upwardSpeed;
   double downwardSpeed;

  frc::SlewRateLimiter<units::dimensionless::scalar> ArmFilter{0.1 / 20_ms};
};
