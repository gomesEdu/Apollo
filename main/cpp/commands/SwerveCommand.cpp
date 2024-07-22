// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SwerveCommand.h"
#include "Constants.h"

SwerveCommand::SwerveCommand(SwerveSubsystem*swerve_SubsystemParameter,
                             ControlSubsystem*Control_SubsystemParameter,
                             bool fielOriented_param):
swerve_Subsystem{swerve_SubsystemParameter},
control{Control_SubsystemParameter},
FieldOriented{fielOriented_param}
  
  {
//Adds the specified Subsystem requirement to the command. The scheduler will prevent
// two commands that require the same subsystem from being scheduled simultaneously.
// Note that the scheduler determines the requirements of a command when it is scheduled,
// so this method should normally be called from the command's constructor.
  AddRequirements(swerve_SubsystemParameter);

  //New Changes Here - Put The AddRequirements for the control Subsystem
  AddRequirements(Control_SubsystemParameter);


  }

// Called when the command is initially scheduled.
void SwerveCommand::Initialize() {
  
  //std::printf("Prog INIT");
}

// Called repeatedly when this Command is scheduled to run
void SwerveCommand::Execute() {

  

  double xAxis = control->GetDriveX();
  double yAxis = control->GetDriveY();
  double rAxis = control->GetRotX();
  bool  Getfield = control->GetFieldOriented();

  //Apllying DeadBand
  xAxis = std::abs(xAxis) > OIconstants::kDeadband ? xAxis : 0.0;
  yAxis = std::abs(yAxis) > OIconstants::kDeadband ? yAxis : 0.0;   
  rAxis = std::abs(rAxis) > OIconstants::kDeadband ? rAxis : 0.0;

  // Make the drive smoother
   xAxis = xAxisLimiter.Calculate(xAxis) * DriveConstants::kTeleDriveMaxSpeedMetersPerSecond;
   yAxis = yAxisLimiter.Calculate(yAxis) * DriveConstants::kTeleDriveMaxSpeedMetersPerSecond;
   rAxis = rAxisLimiter.Calculate(rAxis)
                                 * DriveConstants::kTeleDriveMaxAngularSpeedRadiansPerSecond;

  swerve_Subsystem->Drive(static_cast<units::meters_per_second_t>(yAxis),
                          static_cast<units::meters_per_second_t>(xAxis),
                          static_cast<units::radians_per_second_t>(rAxis),
                          Getfield);

    

std::printf("Prog EXECUTE");
frc::SmartDashboard::PutBoolean("FieldOrientedG", Getfield);
}

// Called once the command ends or is interrupted.
void SwerveCommand::End(bool interrupted) {

  swerve_Subsystem->StopModules();
  //std::printf("Prog END");
}

// Returns true when the command should end.
bool SwerveCommand::IsFinished() {

  //std::printf("Prog FINISHED");
  return false;
}
