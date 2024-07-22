// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once



#include <frc/drive/MecanumDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

#include <string>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

//#include "AHRS.h" //TIRAR

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>

#include "Constants.h"

#include "LimeLightSubsystem.h"
#include "SwerveModule.h"
#include "ArmSubsystem.h"
#include <thread> 
#include <iostream>


class SwerveSubsystem : public frc2::SubsystemBase {
 public:
  SwerveSubsystem();

  //PID AUTONOMOUS PARAMETER
  double kPThetaController = AutoConstants::kPThetaController;
  double KPXaxisController = AutoConstants::kPXController;
  double KPYaxisController = AutoConstants::kPYController;


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  //The Mainly method for SwerveDrive in TeleOperated, that transform the joystick double inputs into 
  //a module state (Meters Per Second and Radians Per Second)
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative)
             {
                Drive(xSpeed, ySpeed, rot, fieldRelative, 0_m, 0_m);
             }
  //as above, but allow more specifications
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, units::meter_t x_center, units::meter_t y_center);

  
  //Reset the NavX Heading
  void ZeroHeading();

  //Reset all the 8 encoders
  void ResetEncoders();

  //Get The NavX Heading
  double GetHeading();
 
  //2d NavX Rotation method 
  frc::Rotation2d GetRotation2d();

  //2d Odometry method
  frc::Pose2d GetPose2d();

  //Reset the Odometry for Autonomous
  void ResetOdometry(frc::Pose2d Pose);

  //get the pidgeon angle in radians
  double GetRadians();

  //Stop all the 4  modules (8 Motors)
  void StopModules();

  void SetModulesState(wpi::array<frc::SwerveModuleState, 4> desiredStates);


 private:
  //Declaring each module as a SwerveModule object
  SwerveModule FrontLeft;
  SwerveModule RearLeft;
  SwerveModule FrontRight;
  SwerveModule RearRight;

  //Declaring the 4 LimeLights as objects from LimeLightSubsystem
  LimeLightSubsystem Lime_Fl;
  LimeLightSubsystem Lime_Rl;
  LimeLightSubsystem Lime_Fr;
  LimeLightSubsystem Lime_Rr;
  // Tx_Values for each module
   double Tx_FL;
   double Tx_RL;
   double Tx_FR;
   double Tx_RR;
  // Ty_Values for each module
   double Ty_FL;
   double Ty_RL;
   double Ty_FR;
   double Ty_RR;
  //Ta_Values for each module
   double Ta_FL;
   double Ta_RL;
   double Ta_FR;
   double Ta_RR;
  //TSkew_Values for each module
   double TSkew_FL;
   double TSkew_RL;
   double TSkew_FR;
   double TSkew_RR;

  //PIGEON PHOENIX 6
  ctre::phoenix6::hardware::Pigeon2 Pigeon{20};
 
  //PIGEON CONFIGURATION PHOENIX 6
  ctre::phoenix6::configs::Pigeon2Configuration pigeon_Config;

  // Odometry class for tracking robot pose
  // 4 defines the number of modules
  frc::SwerveDriveOdometry<4> Odometry;

  //for the drive method
  double rotation{0.0};
  double x{0.0};
  double y{0.0};
  double heta{0.0};
};