// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "string"
#include <units/angle.h>
#include <units/length.h>
#include "numbers"
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/acceleration.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

#include <frc/trajectory/TrapezoidProfile.h>

#include <frc/kinematics/SwerveDriveKinematics.h>

using namespace units;
using namespace units::angle;
using namespace units::length;
/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace BooleansConsts{
   constexpr bool kFrontLeftTurningEncoderReversed = true;
   constexpr bool kBackLeftTurningEncoderReversed = true;
   constexpr bool kFrontRightTurningEncoderReversed = true;
   constexpr bool kBackRightTurningEncoderReversed = true;

   constexpr bool kFrontLeftDriveEncoderReversed = false;
   constexpr bool kBackLeftDriveEncoderReversed = false;
   constexpr bool kFrontRightDriveEncoderReversed = false;
   constexpr bool kBackRightDriveEncoderReversed = false;

   constexpr bool kFrontLeftAbsoluteEncoderReversed = false;
   constexpr bool kBackLeftAbsoluteEncoderReversed = false;
   constexpr bool kFrontRightAbsoluteEncoderReversed = false;
   constexpr bool kBackRightAbsoluteEncoderReversed = false;
}

namespace DriveConstants{

//MECACHRONOS EDIT - 20/01/24
  constexpr int kFrontRightDriveMotor = 7;
  constexpr int kFrontLeftDriveMotor = 3;
  constexpr int kBackRightDriveMotor = 5;
  constexpr int kBackLeftDriveMotor = 1;

  constexpr int kFrontRightTurningMotor = 8;
  constexpr int kFrontLeftTurningMotor = 4;
  constexpr int kBackRightTurningMotor = 6;
  constexpr int kBackLeftTurningMotor = 2;
  //=================================

    //Max angle and drive acceleration for Teleoperated
  constexpr double kTeleDriveMaxAccelerationUnitsPerSecond = 4.0;
  constexpr double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 6.0; 

   //The variable to put in SetDesiredState in SwerveSubsystem.cpp in mps
  constexpr  auto  kMpsPhysicalMaxSpeedMetersPerSecond = 2_mps;

  //The Variable to put in limiters in SwerveCommand.cpp, in double
  constexpr  double  kDoublePhysicalMaxSpeedMetersPerSecond = 1.0;

  constexpr double kTeleDriveMaxSpeedMetersPerSecond = kDoublePhysicalMaxSpeedMetersPerSecond;

  //Perfect one 
  // constexpr double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2.5 * std::numbers::pi;
    constexpr double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 1.5 * std::numbers::pi;
  constexpr double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
  kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

  //Translation and kinematics object 

    constexpr units::meter_t kTrackWidth =
      0.7_m;  // Distance between centers of right and left wheels on robot
    constexpr units::meter_t kWheelBase =
      0.5_m;  // Distance between centers of front and back wheels on robot

    
     static frc::SwerveDriveKinematics<4>  kDriveKinematics{
    frc::Translation2d{kWheelBase / 2, kTrackWidth / 2},
    frc::Translation2d{kWheelBase / 2, -kTrackWidth / 2},
    frc::Translation2d{-kWheelBase / 2, kTrackWidth / 2},
    frc::Translation2d{-kWheelBase/ 2, -kTrackWidth / 2}};


}//namespace DriveConstants

namespace CanSensorConstants{

  constexpr int kFrontRightTurningSensor = 23;
  constexpr int kFrontLeftTurningSensor = 24;
  constexpr int kBackRightTurningSensor = 22;
  constexpr int kBackLeftTurningSensor = 21;

}//namespace CansSensorConstants

namespace OperatorConstants
{
    
    constexpr int kDriverControllerPort = 0;
    constexpr int kTopControllerPort = 1;

} // namespace OperatorConstants

namespace RLGear
{
    // angle gear relation set
    constexpr double ANGLE_GEAR_RATIO = 12.8 / 1.0;
    // drive gear relation set
    constexpr double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
}

namespace WheelUnits
{
    constexpr meter_t WHEEL_DIAMETER = 3.94_in;
    constexpr meter_t WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * std::numbers::pi;
}

namespace DefAngModule
{

//MECACRONICO
    /* Module OFFSET Specific Constants */

    //******ANGULOS EM RADIANOS!!!!!!*******
    /* Front Right Module */
    constexpr double FR_DEGREE_OFFSET = 2.208554;
    /* Rear Right Module */
    constexpr double RR_DEGREE_OFFSET = 0.283253;
    //constexpr double RR_DEGREE_OFFSET = 1.554;
;
    /* Front Left Module */
    constexpr double FL_DEGREE_OFFSET = -0.0981801; ;
    /* Rear Left Module */
    constexpr double RL_DEGREE_OFFSET = 1.026637 ;
;
}

// namespace DegreeOffSet
// {
//     /* Merge swerve drive constants into easy access vectors */
//     constexpr radian_t DEGREE_OFFSETS[] = {
//         DefAngModule::MOD_1_DEGREE_OFFSET,
//         DefAngModule::MOD_2_DEGREE_OFFSET,
//         DefAngModule::MOD_3_DEGREE_OFFSET,
//         DefAngModule::MOD_4_DEGREE_OFFSET};
// }

namespace PIDModConstants
{
    constexpr double kPTurning = 0.3;
    constexpr double kITurning = 0.0002;
    constexpr double kDTurning = 0.000005;
    constexpr double kPModuleDriveController = 0.01;
}

namespace ConstantsMod
{
    constexpr double kWheelDiameterMeters = 0.1; 
    constexpr double kDriveMotorGearRatio = 1 / 6.75;
    constexpr double kTurningMotorGearRatio = 1 / 18.0;
    constexpr double kDriveEncoderRot2Meter = kDriveMotorGearRatio * std::numbers::pi * kWheelDiameterMeters;
    constexpr double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * std::numbers::pi;
    constexpr double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    constexpr double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    
} // namespace

namespace OIconstants {
    constexpr  int kDriverYAxis = 1;
    constexpr  int kDriverXAxis = 0;
    constexpr  int kDriverRotAxis = 4;
    constexpr  int kDriverFieldOrientedButtonIdx = 1;
    constexpr  double kDeadband = 0.05;
}//namepsace OIconstants


namespace CenterConstants {
  constexpr meter_t centerx = 0.715_m / 2;
  constexpr meter_t centery = 0.715_m / 2;
}

namespace AutoConstants {
constexpr auto kMaxSpeed = 1.0_mps; //original = 0.5_mps
constexpr auto kMaxAcceleration = 0.5_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.8;
constexpr double kPYController = 0.8;
constexpr double kPThetaController = 0.05;


extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace LimeConstants{
  //NetWorkTable Objects
  static std::shared_ptr<nt::NetworkTable> NetWorkTable_FL;
  static std::shared_ptr<nt::NetworkTable> NetWorkTable_RL;
  static std::shared_ptr<nt::NetworkTable> NetWorkTable_FR;
  static std::shared_ptr<nt::NetworkTable> NetWorkTable_RR;

  //Limes string names
  //FrontLeft Lime name
  static std::string Lime_FlName = "Lime_FrontLeft";
  //RearLeft Lime name
  static std::string Lime_RlName = "Lime_RearLeft"; 
  //FrontRight Lime name
  static std::string Lime_FrName = "Lime_FrontRight"; 
  //RearRight Lime name
  static std::string Lime_RrName = "Lime_RearRight";  

}

namespace ArmConstants{
 constexpr double downwardArmSpeedConstant = 0.2;
    constexpr double upwardArmSpeedConstant = 0.2;
}

namespace IntakeConstants {
  constexpr double intake_SpitSpeakerSpeedConstant = 1.0;
  constexpr double intake_CatchSpeedConstant = 0.5;
  constexpr double intake_SpitAmpSpeedConstant = 0.5;
}

namespace ShooterConstants {
  constexpr double speaker_ShootSpeedConstant = 1.0;//Velocidade do motor do Shooter para lançar no Speaker
  constexpr double source_CatchSpeedConstant = -0.5; //Velocidade do motor para pegar a nota pelo Shooter
  constexpr double amplifier_ShootSpeedConstant = 0.165;//Velocidade do motor do Shooter para lançar no Amplificador
}

namespace ClimberConstants {
  constexpr double upward_ClimberConstant = 0.65;
  constexpr double downward_ClimberConstant = 0.45;
}


namespace encoderPositionConstants{
  //CLIMBER 
  constexpr int climberLeftEncoderMax = 100;
  constexpr int climberLeftEncoderMin = 50;
  constexpr int climberRightEncoderMax = 100;
  constexpr int climberRightEncoderMin = 50;
  //INTAKE ARM
  constexpr int intakeArmEncoderMax = -29.0;
  constexpr int intakeArmEncoderMin = 0.0;
}

namespace encoderSpeedConstants{ //Obs.: Motores REV NEO tem um RPM teórico sem carga de 5676RPM
  //SHOOTER 
  constexpr double shooterLeftMotorEncoderSpeed_Min = 5000.00;
  constexpr double shooterLeftMotorEncoderSpeed_Max = 5676.00;
  constexpr double shooterRightMotorEncoderSpeed_Min = 5000.00;
  constexpr double shooterRightMotorEncoderSpeed_Max = 5676.00;
}