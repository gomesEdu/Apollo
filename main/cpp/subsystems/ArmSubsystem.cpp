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
}

void ArmSubsystem::Periodic() {}

  void ArmSubsystem::moveUp(){//Move o Braço do Intake em direção ao shooter, para cima
    armMotor.Set(0.30);
  }
  void ArmSubsystem::moveDown(){//Move o Braço do Intake em direção ao chão, para baixo
    armMotor.Set(-0.30);
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
    int motorTurns = armEncoder.GetPosition();
    std::cout<<"\n\nTURNS = "<<motorTurns<<" UHUL\n";
   if(motorTurns<=-29){
      return true;
   }
   else{
      return false;
    }
  }

  bool ArmSubsystem::reachedShooter(){
    int motorTurns = armEncoder.GetPosition();
    std::cout<<"\n\nTURNS = "<<motorTurns<<" UHUL\n";
   if(motorTurns>=-3){
      return true;
   }
   else{
      return false;
    }
  }







