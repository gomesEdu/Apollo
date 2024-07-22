// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

//KRAKEN
SwerveModule::SwerveModule(int driveMotorId, int turningMotorId, bool driveMotorReversed, bool turningMotorReversed,
                           int absoluteEncoderId, double absoluteEncoderOffset, bool absoluteEncoderReversed) : driveMotor(driveMotorId,"rio"),
                                                                                                                turningMotor(turningMotorId, rev::CANSparkMax::MotorType::kBrushless),
                                                                                                                absoluteEncoder(absoluteEncoderId),
                                                                                                                turningEncoder(turningMotor.GetEncoder()),
                                                                                                                absoluteEncoderOffsetRad(absoluteEncoderOffset)
{


  turningPidController.EnableContinuousInput(0, 2*std::numbers::pi);
  toApplyCancoder.MagnetSensor.AbsoluteSensorRange = 0;

  // set units of the CANCoder to radians, with velocity being radians per second
  //config.sensorCoefficient = (2 * std::numbers::pi / 4096.0);//PHOENIX 5
	//config.unitString = "rad";//PHOENIX 5
	//config.sensorTimeBase = SensorTimeBase::PerSecond;//PHOENIX 5
  //absoluteEncoder.ConfigAllSettings(config); //PHOENIX 5
  //absoluteEncoder.ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360);//PHOENIX 5


  

  driveMotor.SetInverted(driveMotorReversed);
  turningMotor.SetInverted(turningMotorReversed);

  
  //KRAKEN
   driveMotor.SetInverted(driveMotorReversed);
   driveMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  

  //Rev 
  turningMotor.SetSmartCurrentLimit(0, 35);
  turningMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  turningMotor.BurnFlash();

  
  /*Rev
   driveEncoder.SetPositionConversionFactor(ConstantsMod::kDriveEncoderRot2Meter);
   driveEncoder.SetVelocityConversionFactor(ConstantsMod::kDriveEncoderRPM2MeterPerSec);
   driveMotor.BurnFlash();*/


  ResetEncoders();
  ConfigureKraken(driveMotor.GetConfigurator());
  absoluteEncoder.GetConfigurator().Apply(toApplyCancoder);
}

double SwerveModule::GetDriveVelocity()
{
  return driveMotor.GetVelocity().GetValueAsDouble() *60;
}

double SwerveModule::getTurningVelocity()
{
  return turningEncoder.GetVelocity();
}

double SwerveModule::GetDrivePosition()
{
  return driveMotor.GetPosition().GetValueAsDouble();
  
}


double SwerveModule::GetTurningPosition()
{
  return turningEncoder.GetPosition();
}

double SwerveModule::GetAbsoluteEncoderRad()
{

  double angle = absoluteEncoder.GetAbsolutePosition().GetValueAsDouble()*(2*std::numbers::pi) - absoluteEncoderOffsetRad;
  // double angle = absoluteEncoder.GetAbsolutePosition().GetValueAsDouble()*(2*std::numbers::pi);
  
  if (angle < 0)
  {
    angle += 2*std::numbers::pi;
  }

  if (angle > 2*std::numbers::pi){
    angle -= 2*std::numbers::pi;
  }
    
  return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  
  
}

void SwerveModule::ResetEncoders()
{
  units::angle::turn_t zeroturns = 0_tr;
  driveMotor.SetPosition(zeroturns);
  turningEncoder.SetPosition(GetAbsoluteEncoderRad());
}

frc::SwerveModuleState SwerveModule::GetState()
{
  //  return {units::meters_per_second_t{GetDriveVelocity()},
  //          frc::Rotation2d(units::radian_t{GetTurningPosition()})};

      return {units::meters_per_second_t{GetDriveVelocity()},
            frc::Rotation2d(units::radian_t(GetAbsoluteEncoderRad()))};

}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& State)
{
  if (units::math::abs(State.speed) < units::meters_per_second_t{0.002})
  {
    Stop();
    return;
  }

  auto state = frc::SwerveModuleState::Optimize(State, GetState().angle);

    auto speed_motor = 0_mps;
    if (state.speed > DriveConstants::kMpsPhysicalMaxSpeedMetersPerSecond)
    {
      speed_motor =   DriveConstants::kMpsPhysicalMaxSpeedMetersPerSecond;
    }
    else
    {
      speed_motor = state.speed;
    }
      
    driveMotor.Set(static_cast<double>(speed_motor));
    turningMotor.Set(turningPidController.Calculate(GetAbsoluteEncoderRad(), static_cast<double>(state.angle.Radians())));
    
}

  frc::SwerveModulePosition SwerveModule::GetPosition(){

    return {units::meter_t{driveMotor.GetPosition().GetValueAsDouble()},

    frc::Rotation2d{units::radian_t{GetAbsoluteEncoderRad()}}};

  }

void SwerveModule::Stop()
{
  driveMotor.Set(0);
  turningMotor.Set(0);
}

// This method will be called once per scheduler run
void SwerveModule::Periodic() {
}


void SwerveModule::ConfigureKraken(ctre::phoenix6::configs::TalonFXConfigurator &cfg){
  ctre::phoenix6::configs::TalonFXConfiguration toApply{};

  m_currentLimits.SupplyCurrentLimit = 40;

  toApply.MotorOutput.NeutralMode = 1;

  ctre::phoenix6::configs::MotionMagicConfigs &MotionDriveConfig = toApply.MotionMagic;

  MotionDriveConfig.MotionMagicCruiseVelocity = 100;
  MotionDriveConfig.MotionMagicAcceleration = 200;
  MotionDriveConfig.MotionMagicJerk = 50;
  MotionDriveConfig.MotionMagicExpo_kA = 0.0001;

  // ctre::phoenix6::configs::MotionMagicConfigs &mm_Drive = toApply.MotionMagic;

  // mm_Drive.MotionMagicCruiseVelocity = 90;
  // mm_Drive.MotionMagicAcceleration = 250;
  // mm_Drive.MotionMagicJerk = 200;

  ctre::phoenix6::configs::Slot0Configs &slot0_Drive = toApply.Slot0;


  slot0_Drive.kP = 0.10;
  slot0_Drive.kI = 0.0001;
  slot0_Drive.kD = 0.00001;
  slot0_Drive.kV = 0.12;
  slot0_Drive.kS = 0.05; // Approximately 0.25V to get the mechanism moving
  
  toApply.CurrentLimits = m_currentLimits;

  cfg.Apply(toApply);
}