#include "subsystems/ArmSubsystem.h"
#include<iostream>
#include"Constants.h"
#include "RobotContainer.h"

ArmSubsystem::ArmSubsystem()

{
armMotor.SetSmartCurrentLimit(0,35);
armMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
armMotor.BurnFlash();
armMotor.SetInverted(false);

// set PID coefficients
    armPidController.SetP(kP);
    armPidController.SetI(kI);
    armPidController.SetD(kD);
    armPidController.SetIZone(kIz);
    armPidController.SetFF(kFF);
    armPidController.SetOutputRange(kMinOutput, kMaxOutput);
  armEncoder.SetPosition(30);
  //armPidController.SetReference(15, rev::CANSparkMax::ControlType::kPosition);
  armMotor.Set(0.0);

// Set Initial Encoder Position - Edu (10/05/2024)
// ***Note that the way the armMotor is attached to our robot, it starts at zero position when the
//arm is at the initial position. However, when it spins to // reach ground position (to catch the note),
//its position decreases to negative values, which is problematic for PID positioning. Therefore, the 
//solution is to set the encoder position to a positive initial value, somewhere near 30. Then, the 
//ground position is somewhere near 0.

}

void ArmSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("ARM POS", armEncoder.GetPosition());
}

  void ArmSubsystem::moveUp(){//Move o Braço do Intake em direção ao shooter, para cima
    armMotor.Set(ArmConstants::upwardArmSpeedConstant);
  }
  void ArmSubsystem::moveDown(){//Move o Braço do Intake em direção ao chão, para baixo
    armMotor.Set(-ArmConstants::downwardArmSpeedConstant);
  }
  void ArmSubsystem::stop(){//Desliga o Braço do Intake
    armMotor.Set(0.00);
  }

  void ArmSubsystem::holdShooterPosition(){//Mantém o braço freado numa posição
    armMotor.Set(0.01);
  }

  void ArmSubsystem::holdGroundPosition(){//Mantém o braço freado numa posição
    armMotor.Set(-0.01);
  }
  bool ArmSubsystem::reachedAmplifier(){}

  bool ArmSubsystem::reachedGround(){
   //if(armEncoder.GetPosition()<=ArmConstants::armAtGroundPosition){
    if(armEncoder.GetPosition()<=2){
      return true;
   }
   else{
      return false;
    }
  }

  bool ArmSubsystem::reachedShooter(){
   if(armEncoder.GetPosition()>=ArmConstants::armAtShooterPosition){
      return true;
   }
   else{
      return false;
    }
  }

  
  void ArmSubsystem::moveToShooterPID(){
    armPidController.SetReference(ArmConstants::armToShooterMaxPIDPosition, rev::CANSparkMax::ControlType::kPosition);
  }

  void ArmSubsystem::moveToGroundnPID(){
    armPidController.SetReference(ArmConstants::armToGroundMaxPIDPosition, rev::CANSparkMax::ControlType::kPosition);
  }

  void ArmSubsystem::moveToAmpPID(){
    armPidController.SetReference(ArmConstants::armAtAmpPosition, rev::CANSparkMax::ControlType::kPosition);
  }

  bool ArmSubsystem::reachedGroundPIDLimit(){
   if(armEncoder.GetPosition()<=ArmConstants::armToGroundMaxPIDPosition){
      return true;
   }
   else{
      return false;
    }
  }

  bool ArmSubsystem::reachedShooterPIDLimit(){
   if(armEncoder.GetPosition()>=ArmConstants::armToShooterMaxPIDPosition){
      return true;
   }
   else{
      return false;
    }
  }







