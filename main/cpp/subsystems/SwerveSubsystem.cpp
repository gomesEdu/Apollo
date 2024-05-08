// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveSubsystem.h"


SwerveSubsystem::SwerveSubsystem()
    : FrontLeft{DriveConstants::kFrontLeftDriveMotor, DriveConstants::kFrontLeftTurningMotor,
                BooleansConsts::kFrontLeftDriveEncoderReversed,
                BooleansConsts::kFrontLeftTurningEncoderReversed,
                CanSensorConstants::kFrontLeftTurningSensor,
                DefAngModule::FL_DEGREE_OFFSET,
                BooleansConsts::kFrontLeftAbsoluteEncoderReversed},

      RearLeft{DriveConstants::kBackLeftDriveMotor, DriveConstants::kBackLeftTurningMotor,
               BooleansConsts::kBackLeftDriveEncoderReversed,
               BooleansConsts::kBackLeftTurningEncoderReversed,
               CanSensorConstants::kBackLeftTurningSensor,
               DefAngModule::RL_DEGREE_OFFSET,
               BooleansConsts::kBackLeftAbsoluteEncoderReversed},

      FrontRight{DriveConstants::kFrontRightDriveMotor, DriveConstants::kFrontRightTurningMotor,
                 BooleansConsts::kFrontRightDriveEncoderReversed,
                 BooleansConsts::kFrontRightTurningEncoderReversed,
                 CanSensorConstants::kFrontRightTurningSensor,
                 DefAngModule::FR_DEGREE_OFFSET,
                 BooleansConsts::kFrontRightAbsoluteEncoderReversed},

      RearRight{DriveConstants::kBackRightDriveMotor, DriveConstants::kBackRightTurningMotor,
                BooleansConsts::kBackRightDriveEncoderReversed,
                BooleansConsts::kBackRightTurningEncoderReversed,
                CanSensorConstants::kBackRightTurningSensor,
                DefAngModule::RR_DEGREE_OFFSET,
                BooleansConsts::kBackRightAbsoluteEncoderReversed},
      
      /*Odometry{DriveConstants::kDriveKinematics,
                 GetRotation2d(),
                 {FrontLeft.GetPosition(), FrontRight.GetPosition(),
                  RearLeft.GetPosition(), RearRight.GetPosition()},
                 frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}},
                 */

                 Odometry{DriveConstants::kDriveKinematics,
                 GetRotation2d(),
                 {FrontLeft.GetPosition(), FrontRight.GetPosition(),
                  RearLeft.GetPosition(), RearRight.GetPosition()},
                 frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}},
                 

       Lime_Fl{LimeConstants::Lime_FlName,LimeConstants::NetWorkTable_FL,
               Tx_FL,Ty_FL,Ta_FL,TSkew_FL},

       Lime_Rl{LimeConstants::Lime_RlName,LimeConstants::NetWorkTable_RL,
               Tx_RL,Ty_RL,Ta_RL,TSkew_RL},

       Lime_Fr{LimeConstants::Lime_FrName,LimeConstants::NetWorkTable_FR,
               Tx_FR,Ty_FR,Ta_FR,TSkew_FR},

       Lime_Rr{LimeConstants::Lime_RrName,LimeConstants::NetWorkTable_RR,
               Tx_RR,Ty_RR,Ta_RR,TSkew_RR}

                  

      //initializating navX in  SwerveSubsystem ctor
    //   ahrs(new AHRS(frc::SPI::Port::kMXP))
      
                        
{    

//A thread of execution is a sequence of instructions that can be executed concurrently 
//with other such sequences in multithreading environments, while sharing a same address space.
//We've to create a thread to delay in 1 sec, and in sequence we execute  the ZeroHeading action! 
//Finalizating the trhead call
    std::thread([this]{
        try { 
            std::this_thread::sleep_for(std::chrono::seconds(1)); //First execution
            ZeroHeading(); //Second execution
        } catch(const std::exception& e){

        }
    }).detach();
}

void SwerveSubsystem::ZeroHeading(){
    // ahrs->Reset();
    Pidgeon.Reset();
}

double SwerveSubsystem::GetHeading(){

    
    // return std::remainder(ahrs->GetAngle(),360);
    return std::remainder(Pidgeon.GetAngle(),360);
    //One possible solution to a future problem of Heading
    // return ahrs->GetAngle();
    //return Pidgeon.GetRoll();
    //return Pidgeon.GetAngle();
}

frc::Rotation2d SwerveSubsystem::GetRotation2d(){
//    return ahrs->GetRotation2d().Degrees();
return Pidgeon.GetRotation2d().Degrees();
}

frc::Pose2d SwerveSubsystem::GetPose2d(){

     return Odometry.GetPose();
 }

void SwerveSubsystem::ResetOdometry(frc::Pose2d pose){

Odometry.ResetPosition(GetRotation2d(),
      {FrontLeft.GetPosition(), FrontRight.GetPosition(),
       RearLeft.GetPosition(), RearRight.GetPosition()},
      pose);
}

// This method will be called once per scheduler run
void SwerveSubsystem::Periodic() {


    Odometry.Update(GetRotation2d(), 
    {FrontLeft.GetPosition() , RearLeft.GetPosition(),
    FrontRight.GetPosition(), RearRight.GetPosition()});

//-------------------------------------NAVX Shuffle------------------------------------------------//

    frc::SmartDashboard::PutNumber("Robot Heading",GetHeading());

    
//--------------------------------------------------------------------------------------------------//

    // frc::SmartDashboard::PutNumber("ArmAbsolutePositon",arm_Subsystem.GetAbsolute());

//---------------------------------Swerve Drive Shuffle--------------------------------------------//

//AbsoluteEncoderModule Reading
    frc::SmartDashboard::PutNumber("FrontRightAbsolutePosition", FrontRight.GetAbsoluteEncoderRad());
    frc::SmartDashboard::PutNumber("FrontLeftAbsolutePosition", FrontLeft.GetAbsoluteEncoderRad());
    frc::SmartDashboard::PutNumber("RearLeftAbsolutePosition", RearLeft.GetAbsoluteEncoderRad());
    frc::SmartDashboard::PutNumber("RearRightAbsolutePosition", RearRight.GetAbsoluteEncoderRad());

//GetPositon module methods
    // frc::SmartDashboard::PutNumber("FrontRightTurningPosition", FrontRight.GetTurningPosition());
    frc::SmartDashboard::PutNumber("FrontRightDrivePosition", FrontRight.GetDrivePosition());

    // frc::SmartDashboard::PutNumber("FrontLeftTurningPosition", FrontLeft.GetTurningPosition());
    frc::SmartDashboard::PutNumber("FrontLeftDrivePosition", FrontLeft.GetDrivePosition());

    // frc::SmartDashboard::PutNumber("RearRightTurningPosition", RearRight.GetTurningPosition());
    frc::SmartDashboard::PutNumber("RearRightDrivePosition", RearRight.GetDrivePosition());
    
    // frc::SmartDashboard::PutNumber("RearLeftTurningPosition", RearLeft.GetTurningPosition());
    frc::SmartDashboard::PutNumber("RearLeftDrivePosition", RearLeft.GetDrivePosition());

    // frc::SmartDashboard::PutNumber("RearLeftDrivePosition", RearLeft.GetDrivePosition());

    frc::SmartDashboard::PutNumber("Robot X odometry", static_cast<double>(GetPose2d().Translation().X()));
    frc::SmartDashboard::PutNumber("Robot Y odometry", static_cast<double>(GetPose2d().Translation().Y()));
    frc::SmartDashboard::PutNumber("Robot theta odometry", static_cast<double>(GetPose2d().Rotation().Degrees()));


    frc::SmartDashboard::SetDefaultNumber("KpAutoTheta", kPThetaController);
    kPThetaController = frc::SmartDashboard::GetNumber("KpAutoTheta", kPThetaController);

    frc::SmartDashboard::SetDefaultNumber("KpAutoXaxis", KPXaxisController);
    KPXaxisController = frc::SmartDashboard::GetNumber("KpAutoXaxis", KPXaxisController);

        frc::SmartDashboard::SetDefaultNumber("KpAutoYaxis", KPYaxisController);
    KPYaxisController = frc::SmartDashboard::GetNumber("KpAutoYaxis", KPYaxisController);

  


//--------------------------------------------------------------------------------------------------//

//Odometry Shuffle
// frc::SmartDashboard::("Robot Location", GetPose2d().);


//---------------------------------LimeLight Shuffle------------------------------------------------//

    //Lime_FL TX,TY,TA,TSKEW shuffle values
    // frc::SmartDashboard::PutNumber("LimeLight-FL_Tx", Lime_Fl.GetXhorizontal());
    // frc::SmartDashboard::PutNumber("LimeLight-FL_Ty", Lime_Fl.GetYvertical());
    // frc::SmartDashboard::PutNumber("LimeLight-FL_Ta", Lime_Fl.GetAarea());
    // frc::SmartDashboard::PutNumber("LimeLight-FL_TSkew", Lime_Fl.GetError());

    //Lime_RL TX,TY,TA,TSKEW shuffle values
    // frc::SmartDashboard::PutNumber("LimeLight-RL_Tx", Lime_Rl.GetXhorizontal());
    // frc::SmartDashboard::PutNumber("LimeLight-RL_Ty", Lime_Rl.GetYvertical());
    // frc::SmartDashboard::PutNumber("LimeLight-RL_Ta", Lime_Rl.GetAarea());
    // frc::SmartDashboard::PutNumber("LimeLight-RL_TSkew", Lime_Rl.GetError());

    //Lime_FR TX,TY,TA,TSKEW shuffle values
    // frc::SmartDashboard::PutNumber("LimeLight-FR_Tx", Lime_Fr.GetXhorizontal());
    // frc::SmartDashboard::PutNumber("LimeLight-FR_Ty", Lime_Fr.GetYvertical());
    // frc::SmartDashboard::PutNumber("LimeLight-FR_Ta", Lime_Fr.GetAarea());
    // frc::SmartDashboard::PutNumber("LimeLight-FR_TSkew", Lime_Fr.GetError()); 

    //Lime_RR TX,TY,TA,TSKEW shuffle values
    // frc::SmartDashboard::PutNumber("LimeLight-RR_Tx", Lime_Rr.GetXhorizontal());
    // frc::SmartDashboard::PutNumber("LimeLight-FR_Ty", Lime_Rr.GetYvertical());
    // frc::SmartDashboard::PutNumber("LimeLight-FR_Ta", Lime_Rr.GetAarea());
    // frc::SmartDashboard::PutNumber("LimeLight-FR_TSkew", Lime_Rr.GetError());

//--------------------------------------------------------------------------------------------------//                              

  
}

void SwerveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
                           bool fieldRelative, units::meter_t x_center, units::meter_t y_center)
{


frc::Rotation2d Rot2d;

wpi::array<frc::SwerveModuleState, 4> states = DriveConstants::kDriveKinematics.ToSwerveModuleStates(
    fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetRotation2d() ):
    frc::ChassisSpeeds{xSpeed, ySpeed, rot},
    frc::Translation2d(x_center, y_center));

    SetModulesState(states);

}

void SwerveSubsystem::StopModules(){
    FrontLeft.Stop();
    RearLeft.Stop();
    FrontRight.Stop();
    RearRight.Stop();
}

void SwerveSubsystem::ResetEncoders(){
    FrontLeft.ResetEncoders();
    FrontRight.ResetEncoders();
    RearLeft.ResetEncoders();
    RearRight.ResetEncoders();
}

void SwerveSubsystem::SetModulesState( wpi::array<frc::SwerveModuleState, 4> desiredStates){
DriveConstants::kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, DriveConstants::kMpsPhysicalMaxSpeedMetersPerSecond);
     FrontRight.SetDesiredState(desiredStates[0]);
     RearRight.SetDesiredState(desiredStates[1]);
     FrontLeft.SetDesiredState(desiredStates[2]);
     RearLeft.SetDesiredState(desiredStates[3]);
}
