// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

/*SwerveModule::SwerveModule(int driveMotorId, int turningMotorId, bool driveMotorReversed, bool turningMotorReversed,
                           int absoluteEncoderId, double absoluteEncoderOffset, bool absoluteEncoderReversed) : driveMotor(driveMotorId, rev::CANSparkMax::MotorType::kBrushless),
                                                                                                                turningMotor(turningMotorId, rev::CANSparkMax::MotorType::kBrushless),
                                                                                                                absoluteEncoder(absoluteEncoderId),
                                                                                                                turningEncoder(turningMotor.GetEncoder()),
                                                                                                                driveEncoder(driveMotor.GetEncoder()),
                                                                                                                absoluteEncoderOffsetRad(absoluteEncoderOffset)
*/

//KRAKEN
SwerveModule::SwerveModule(int driveMotorId, int turningMotorId, bool driveMotorReversed, bool turningMotorReversed,
                           int absoluteEncoderId, double absoluteEncoderOffset, bool absoluteEncoderReversed) : driveMotor(driveMotorId,"rio"),
                                                                                                                turningMotor(turningMotorId, rev::CANSparkMax::MotorType::kBrushless),
                                                                                                                absoluteEncoder(absoluteEncoderId),
                                                                                                                turningEncoder(turningMotor.GetEncoder()),
                                                                                                                absoluteEncoderOffsetRad(absoluteEncoderOffset)
{


  turningPidController.EnableContinuousInput(0, 2*std::numbers::pi);

  // set units of the CANCoder to radians, with velocity being radians per second
  config.sensorCoefficient = (2 * std::numbers::pi / 4096.0);
	config.unitString = "rad";
	config.sensorTimeBase = SensorTimeBase::PerSecond;
  absoluteEncoder.ConfigAllSettings(config);
  absoluteEncoder.ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360);
  

  driveMotor.SetInverted(driveMotorReversed);
  turningMotor.SetInverted(turningMotorReversed);

  
  //KRAKEN
   driveMotor.SetInverted(driveMotorReversed);
  

  //Rev 
  turningMotor.SetSmartCurrentLimit(0, 35);
  turningMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  turningMotor.BurnFlash();

  
  /*Rev
   driveEncoder.SetPositionConversionFactor(ConstantsMod::kDriveEncoderRot2Meter);
   driveEncoder.SetVelocityConversionFactor(ConstantsMod::kDriveEncoderRPM2MeterPerSec);
   driveMotor.BurnFlash();*/


  ResetEncoders();
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

  double angle = absoluteEncoder.GetAbsolutePosition() - absoluteEncoderOffsetRad;
  
  if (angle < 0)
  {
    angle += 2*std::numbers::pi;
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