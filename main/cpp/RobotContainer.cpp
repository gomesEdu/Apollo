#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/SwerveCommand.h"
#include <frc2/command/SwerveControllerCommand.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include "Constants.h"

#include "functional"
#include "algorithm"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>

//new Commands (April/May - 2024)
#include "commands/IntakeCommand/ArmToGroundCommand.h"
#include "commands/IntakeCommand/ArmToShooterCommand.h"
#include "commands/IntakeCommand/ArmToGroundPIDCommand.h"
#include "commands/IntakeCommand/ArmToAmpPIDCommand.h"
#include "commands/IntakeCommand/ArmToShooterPIDCommand.h"
#include "commands/IntakeCommand/IntakeCatchCommand.h"
#include "commands/IntakeCommand/IntakeCatchCommand.h"
#include "commands/IntakeCommand/IntakeSpitAmpCommand.h"
#include "commands/IntakeCommand/IntakeSpitSpeakerCommand.h"
#include "commands/IntakeCommand/IntakeStopCommand.h"
#include "commands/ShooterCommand/ShootAmpCommand.h"
#include "commands/ShooterCommand/ShooterStopCommand.h"
#include "commands/ShooterCommand/ShootSpeakerCommand.h"
#include "commands/ShooterCommand/SourceCatchCommand.h"
#include "commands/RGBCommands/LedRgbSetColorCommand.h"

RobotContainer::RobotContainer() {

swerve_Subsystem.SetDefaultCommand(std::move(Command));
//m_intakeSubsystem.SetDefaultCommand(std::move(autoIntakeCatchCommand));

  ConfigureBindings();
  SelectAuto();
}


void RobotContainer::ConfigureBindings() {

  //Comandos do CLIMBER
  /*
  m_DriverController.RightBumper().OnTrue(climberRightUpCommand(m_climberSubsystem).ToPtr());
  m_DriverController.RightTrigger().OnTrue(climberRightDownCommand(m_climberSubsystem).ToPtr());
  m_DriverController.LeftBumper().OnTrue(climberLeftUpCommand(m_climberSubsystem).ToPtr());
  m_DriverController.LeftTrigger().OnTrue(climberLeftDownCommand(m_climberSubsystem).ToPtr());
  */
  m_DriverController.RightBumper().OnTrue(frc2::ParallelCommandGroup(
                              climberRightUpCommand(m_climberSubsystem),
                              climberLeftUpCommand(m_climberSubsystem)
                              ).ToPtr());
  m_DriverController.RightTrigger().OnTrue(frc2::ParallelCommandGroup(
                              climberRightDownCommand(m_climberSubsystem),
                              climberLeftDownCommand(m_climberSubsystem)
                              ).ToPtr());

  //Comandos do SHOOTER
  //m_DriverController.B().WhileTrue(ShootSpeakerCommand(m_shooterSubsystem).ToPtr());
  //m_TopController.A().WhileTrue(SourceCatchCommand(m_shooterSubsystem).ToPtr());
  //m_TopController.LeftBumper().WhileTrue(ShootAmpCommand(m_shooterSubsystem).ToPtr());

  //Comandos do INTAKE
  //m_TopController.LeftTrigger().WhileTrue(IntakeSpitAmpCommand(m_intakeSubsystem,m_shooterSubsystem).ToPtr());
  //m_DriverController.Y().WhileTrue(IntakeSpitSpeakerCommand(m_intakeSubsystem, m_shooterSubsystem).ToPtr());
  //m_TopController.X().WhileTrue(IntakeCatchCommand(m_intakeSubsystem).ToPtr());
  
  //Comandos do BRAÇO do INTAKE 
  //m_TopController.RightBumper().OnTrue(ArmToGroundCommand(m_armSubSystem).ToPtr());
  //m_TopController.RightBumper().OnTrue(ArmToShooterCommand(m_armSubSystem).ToPtr());
  //m_TopController.RightBumper().OnTrue(ArmToShooterPIDCommand(m_armSubSystem).ToPtr());
 //m_TopController.RightTrigger().OnTrue(ArmToGroundPIDCommand(m_armSubSystem).ToPtr());
 //
 //m_TopController.LeftBumper().OnTrue(ArmToAmpPIDCommand(m_armSubSystem).ToPtr());


  //*******************Comandos SEQUENCIAIS***********************************

  //----------------Pega nota do chão automaticamente-------------------------
  
  m_DriverController.X().OnTrue(frc2::SequentialCommandGroup(
                                          ArmToGroundPIDCommand(m_armSubSystem),
                                          LedRgbSetColorCommand(m_ledRgbSubsystem, ledRgbConstants::RED),
                                          IntakeCatchCommand(m_intakeSubsystem),
                                          ArmToShooterPIDCommand(m_armSubSystem),
                                          LedRgbSetColorCommand(m_ledRgbSubsystem, ledRgbConstants::GREEN)
                                        ).ToPtr());
                                        

  //---------------Atira a nota apenas se a velocidade do shooter for máxima--------
   m_DriverController.B().OnTrue(frc2::SequentialCommandGroup(
                                    frc2::ParallelCommandGroup(
                                      ShootSpeakerCommand(m_shooterSubsystem),
                                      IntakeSpitSpeakerCommand(m_intakeSubsystem, m_shooterSubsystem)
                                    ),
                                    LedRgbSetColorCommand(m_ledRgbSubsystem, ledRgbConstants::MAGENTA)
                                  ).ToPtr());

  //---------------Pega a nota pelo Shooter na SOURCE--------
   m_DriverController.Y().OnTrue(frc2::SequentialCommandGroup(
                                    LedRgbSetColorCommand(m_ledRgbSubsystem, ledRgbConstants::ORANGE),
                                    frc2::ParallelCommandGroup(
                                      SourceCatchCommand(m_shooterSubsystem,m_intakeSubsystem),
                                      IntakeCatchCommand(m_intakeSubsystem)
                                    ),
                                    LedRgbSetColorCommand(m_ledRgbSubsystem, ledRgbConstants::GREEN)
                                  ).ToPtr());
}

void RobotContainer::AutonomousInit()
{
  swerve_Subsystem.ResetEncoders();
  swerve_Subsystem.ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d{0_deg}));
}

void RobotContainer::TeleopInit()
{
  swerve_Subsystem.ResetEncoders();
  swerve_Subsystem.ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d{0_deg}));
  m_ledRgbSubsystem.SetColor(10);
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  switch(autoValue){
    case 1: //Teste
        frc::SmartDashboard::PutNumber("Selected AUTO", 1);
        return frc2::SequentialCommandGroup(
                          ArmToGroundCommand(m_armSubSystem),
                          IntakeCatchCommand(m_intakeSubsystem),
                          ArmToShooterCommand(m_armSubSystem),
                            frc2::ParallelCommandGroup(
                              ShootSpeakerCommand(m_shooterSubsystem),
                              IntakeSpitSpeakerCommand(m_intakeSubsystem, m_shooterSubsystem)
                            ),
                            ShooterStopCommand(m_shooterSubsystem)
                          )
                          .ToPtr();
        break;
        case 2:
        frc::SmartDashboard::PutNumber("Selected AUTO", 2);
        return frc2::SequentialCommandGroup(
                          ArmToGroundCommand(m_armSubSystem),
                          IntakeCatchCommand(m_intakeSubsystem),
                          ArmToShooterCommand(m_armSubSystem)
                          )
                          .ToPtr();
        break;
  }
  
}

void RobotContainer::SelectAuto()
{
  frc::SmartDashboard::SetDefaultNumber("Autonomous Select", autoValue);
  autoValue = frc::SmartDashboard::GetNumber("Autonomous Select", autoValue);
  
}


//DECREPTED CODE - Doesnt work - maybe timer problems? - TESTES INICIAIS DO AUTONOMO (JACAREI 2024)
/*
case 2:
    //BLUE SIDE 
    //INICIA NA ESQUINA DO SPEAKER LONGE DO AMP
    //shoota a nota, anda para trás saindo da zona de pontuação
    {
      std::cout<<"AUTO 2"<<std::endl;//me
      return frc2::SequentialCommandGroup(
        frc2::ParallelCommandGroup(autoShooterCommand, autoIntakeSpitCommand),//SHOOTA A NOTA ***0.00s a 2.50s
        autoSwerveStopCommand,//AGUARDA PARADO ***2.5s a 2,6s
        autoSwerveMoveBackwardCommand,//Move-se 1m para trás *** 2,60s a 3,60s
        autoSwerveCounterClockRotationCommand,//Gira horário 30° ***3,60s a 4,10s
        autoSwerveMoveBackwardCommand,//Move-se 1m para trás *** 4,10s a 5,10s
        autoSwerveMoveBackwardCommand,//Move-se 1m para trás *** 5,1s a 6,10s
        autoSwerveStopCommand//AGUARDA PARADO *** 6,10s a 7,10s
        ).ToPtr();
        break;
    }
    case 3: //**BLUE and RED**
    //INICIA NO MEIO DO SPEAKER
    //shoota a nota, anda para trás e pega a nota, anda para frente e shoota a segunda nota
    {
      std::cout<<"AUTO 3"<<std::endl;//me
      return frc2::SequentialCommandGroup(
        frc2::ParallelCommandGroup(autoShooterCommand, autoIntakeSpitCommand),//SHOOTA A NOTA ***0.00s a 2.50s
        autoSwerveStopCommand,//AGUARDA PARADO ***2.5s a 2,6s
        frc2::ParallelCommandGroup(autoArmDownCommand,autoIntakeCatchCommand),//NÃO DESLIGA SOZINHO****2,60s a 3,60s
        autoSwerveMoveBackwardCommand,//Move-se 1m para trás *** 3,60s a 4,60s
        autoSwerveStopCommand,//AGUARDA PARADO *** 4,60s a 4,70s
        autoSwerveMoveBackwardCommand//Move-se 1m para trás *** 4,70s a 5,70s        
        ).ToPtr();
        break;
    }
    case 4:  
    //BLUE SIDE 
    //INICIA NA ESQUINA DO SPEAKER PERTO DO AMP
    //shoota a nota, anda para trás e pega a nota, anda para frente e shoota a segunda nota
    {
      std::cout<<"AUTO 4"<<std::endl;//me
      return frc2::SequentialCommandGroup(
        frc2::ParallelCommandGroup(autoShooterCommand, autoIntakeSpitCommand),//SHOOTA A NOTA ***0.00s a 2.50s
        autoSwerveStopCommand,//AGUARDA PARADO ***2.5s a 2,6s
        autoSwerveMoveBackwardCommand,//Move-se 1m para trás *** 2,60s a 3,60s
        autoSwerveCounterClockRotationCommand,//Gira anti-horário 30° ***3,60s a 4,10s
        autoSwerveMoveBackwardCommand,//Move-se 1m para trás *** 4,10s a 5,10s
        autoSwerveMoveBackwardCommand,//Move-se 1m para trás *** 5,1s a 6,10s
        autoSwerveStopCommand//AGUARDA PARADO *** 6,10s a 7,10s
        ).ToPtr();
        break;
    }
    case 5:  
    //RED SIDE 
    //INICIA NA ESQUINA DO SPEAKER LONGE DO AMP
    //shoota a nota, anda para trás saindo da zona de pontuação
    {
      std::cout<<"AUTO 5"<<std::endl;//me
      return frc2::SequentialCommandGroup(
        frc2::ParallelCommandGroup(autoShooterCommand, autoIntakeSpitCommand),//SHOOTA A NOTA ***0.00s a 2.50s
        autoSwerveMoveBackwardCommand,//Move-se 1m para trás *** 2,60s a 3,60s
        autoSwerveClockwiseRotationCommand,//Gira anti-horário 30° ***3,60s a 4,10s
        autoSwerveMoveBackwardCommand,//Move-se 1m para trás *** 4,10s a 5,10s
        autoSwerveMoveBackwardCommand,//Move-se 1m para trás *** 5,1s a 6,10s
        autoSwerveStopCommand//AGUARDA PARADO *** 6,10s a 7,10s
        ).ToPtr();
        break;
    }
    case 6:  
    //RED SIDE 
    //INICIA NA ESQUINA DO SPEAKER PERTO DO AMP
    //shoota a nota, anda para trás saindo da zona de pontuação
    {
      return frc2::SequentialCommandGroup(
        frc2::ParallelCommandGroup(autoShooterCommand, autoIntakeSpitCommand),//SHOOTA A NOTA ***0.00s a 2.50s
        autoSwerveStopCommand,//AGUARDA PARADO ***2.5s a 2,6s
        autoSwerveMoveBackwardCommand,//Move-se 1m para trás *** 2,60s a 3,60s
        autoSwerveClockwiseRotationCommand,//Gira horário 30° ***3,60s a 4,10s
        autoSwerveMoveBackwardCommand,//Move-se 1m para trás *** 4,10s a 5,10s
        autoSwerveMoveBackwardCommand,//Move-se 1m para trás *** 5,1s a 6,10s
        autoSwerveStopCommand//AGUARDA PARADO *** 6,10s a 7,10s
        ).ToPtr();
        break;
    }
    case 7:  
    //RED SIDE 
    //INICIA NA ESQUINA DO SPEAKER PERTO DO AMP
    //shoota a nota, anda para trás, pega nota do chão, guarda o intake e finaliza com a nota guardada
    {
      return frc2::SequentialCommandGroup(
        frc2::ParallelCommandGroup(autoShooterCommand, autoIntakeSpitCommand),//SHOOTA A NOTA ***0.00s a 2.50s
        autoSwerveStopCommand,//AGUARDA PARADO ***2.5s a 2,6s
        autoSwerveMoveBackwardCommand,//Move-se 1m para trás *** 2,60s a 3,60s
        autoSwerveCounterClockRotationCommand,//Gira horário 30° ***3,60s a 4,10s
        frc2::ParallelCommandGroup(autoArmDownCommand,autoIntakeCatchCommand),//NÃO DESLIGA SOZINHO****2,60s a 3,60s
        autoSwerveMoveBackwardCommand,//Move-se 1m para trás *** 3,60s a 4,60s
        autoSwerveStopCommand,//AGUARDA PARADO *** 4,60s a 4,70s
        autoSwerveMoveBackwardCommand,//Move-se 1m para trás *** 4,70s a 5,70s
        frc2::ParallelCommandGroup(autoArmUpCommand,autoIntakeStopCommand)// *** 5,70s a 8,20s
        ).ToPtr();
        break;
    }
    case 8:  
    //BLUE SIDE 
    //INICIA NA ESQUINA DO SPEAKER PERTO DO AMP
    //shoota a nota, anda para trás, pega nota do chão, guarda o intake e finaliza com a nota guardada
    {
      return frc2::SequentialCommandGroup(
        frc2::ParallelCommandGroup(autoShooterCommand, autoIntakeSpitCommand),//SHOOTA A NOTA ***0.00s a 2.50s
        autoSwerveStopCommand,//AGUARDA PARADO ***2.5s a 2,6s
        autoSwerveMoveBackwardCommand,//Move-se 1m para trás *** 2,60s a 3,60s
        autoSwerveCounterClockRotationCommand,//Gira anti horário 30° ***3,60s a 4,10s
        frc2::ParallelCommandGroup(autoArmDownCommand,autoIntakeCatchCommand),//NÃO DESLIGA SOZINHO****2,60s a 3,60s
        autoSwerveMoveBackwardCommand,//Move-se 1m para trás *** 3,60s a 4,60s
        autoSwerveStopCommand,//AGUARDA PARADO *** 4,60s a 4,70s
        autoSwerveMoveBackwardCommand,//Move-se 1m para trás *** 4,70s a 5,70s
        frc2::ParallelCommandGroup(autoArmUpCommand,autoIntakeStopCommand)// *** 5,70s a 8,20s
        ).ToPtr();
        break;
    }
    case 9:  
    //**BLUE and RED** 
    //INICIA encostado em qualquer lado do speaker
    //shoota a nota e espera o fim do auto
    {
      return frc2::SequentialCommandGroup(
        frc2::ParallelCommandGroup(autoShooterCommand, autoIntakeSpitCommand),//SHOOTA A NOTA ***0.00s a 2.50s
        autoSwerveStopCommand//AGUARDA PARADO ***2.5s a 2,6s
        ).ToPtr();
        break;
    }
  }
*/