// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

//Constants
#include "Constants.h"

//ctre Library Kraken
#include "ctre/Phoenix.h"
#include "ctre/phoenix/motorcontrol/can/TalonFX.h"
#include <ctre/phoenix6/TalonFX.hpp>

//Driver Library
#include "rev/CanSparkMax.h"

//CanCoder Library
#include "ctre/Phoenix.h"

//Units
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <numbers>

//Wplib Default
#include <wpi/DataLog.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>

//Swerve Library(State and Position)
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>

//PID
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>

//Shuffleboard
#include <frc/smartdashboard/SmartDashboard.h>

class SwerveModule : public frc2::SubsystemBase {
 public:
 /**
   * Allocates a PIDController with the given constants for Kp, Ki, and Kd.
   *
   * @param driveMotorId            Id do motor de deslocamento.
   * @param turningMotorId          Id do motor de rotação.
   * @param driveMotorReversed      Parâmetro para reversão do motor de deslocamento.
   * @param turningMotorReversed    Parâmetro para reversão do motor de rotação.
   * @param absoluteEncoderId       Id do encoder absoluto.
   * @param absoluteEncoderOffset   Diferença entre os valores dos encoders absoluto e relativo ao iniciar.
   * @param absoluteEncoderReversed Parâmetro para cosiderar o encoder absoluto como reverso.
   */
  SwerveModule(int driveMotorId, int turningMotorId, bool driveMotorReversed, bool turningMotorReversed,
  int absoluteEncoderId, double absoluteEncoderOffset, bool absoluteEncoderReversed);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void   Periodic() override;   

  double GetDrivePosition();

  double GetTurningPosition();

  double GetDriveVelocity();

  double getTurningVelocity();

  double GetAbsoluteEncoderRad();

  void ResetEncoders();

  frc::SwerveModuleState GetState();

  void SetDesiredState(const frc::SwerveModuleState& state);

  frc::SwerveModulePosition GetPosition();

  void Stop();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //All the objects here is the system base for one module work nice
  //Motors,Relative Encoders, Absolute Encoders , Turning PID and OffSetAngle

    //DriveRev   Motor
  // rev::CANSparkMax driveMotor;

   //KRAKEN
   ctre::phoenix6::hardware::TalonFX driveMotor;

    //TurningRev Motor
    rev::CANSparkMax turningMotor;

    //TurningRev RelativeEncoder 
    rev::SparkMaxRelativeEncoder turningEncoder;

    //Absolute CTRE Encoder 
    ctre::phoenix::sensors::CANCoder absoluteEncoder;

    // set units of the CANCoder to radians, with velocity being radians per second
     CANCoderConfiguration config;
			  
    //Turning PID for Teleoperated    
    frc::PIDController turningPidController{
      PIDModConstants::kPTurning, PIDModConstants::kITurning, PIDModConstants::kDTurning};

    //Reverse Condition for CTRE Absolute Encoder
    bool absoluteEncoderReversed;

    //The OffSet Angle for each module
    double absoluteEncoderOffsetRad;

    //Auto Constants...
    static constexpr auto kModuleMaxAngularVelocity =
      units::radians_per_second_t{std::numbers::pi};
    static constexpr auto kModuleMaxAngularAcceleration =
      units::radians_per_second_squared_t{std::numbers::pi * 2.0};

      
};
