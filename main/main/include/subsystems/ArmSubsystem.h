#pragma once

//Driver Library
#include "rev/CanSparkMax.h"

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
  bool reachedGroundPIDLimit();
  bool reachedShooterPIDLimit();
  void moveToGroundnPID();
  void moveToShooterPID();
  void moveToAmpPID();
  //---------------------------------------------------

 private:
  rev::CANSparkMax armMotor{14,rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkRelativeEncoder armEncoder = armMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkPIDController armPidController = armMotor.GetPIDController();

   double upwardSpeed;
   double downwardSpeed;

  frc::SlewRateLimiter<units::dimensionless::scalar> ArmFilter{0.1 / 20_ms};

  

  // PID coefficients
  //double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;
  double kP = 0.1, kI = 5e-5, kD = 0.1, kIz = 1, kFF = 0, kMaxOutput = 1, kMinOutput = -1;
};
