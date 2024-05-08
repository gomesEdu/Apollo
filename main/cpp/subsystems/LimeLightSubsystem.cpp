// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LimeLightSubsystem.h"

LimeLightSubsystem::LimeLightSubsystem(std::string LimeName_Parameter,std::shared_ptr<nt::NetworkTable> NetWorkTable_parameter, double Tx_Parameter
  , double Ty_Parameter, double Ta_Parameter, double TSkew_Parameter) : LimeLightName(LimeName_Parameter),
                                                                        NetWorkTable(NetWorkTable_parameter),
                                                                        Tx(Tx_Parameter),
                                                                        Ty(Ty_Parameter),
                                                                        Ta(Ta_Parameter),
                                                                        TSkew(TSkew_Parameter)
{

NetWorkTable = nt::NetworkTableInstance::GetDefault().GetTable(LimeLightName);

}
                                                                        

// This method will be called once per scheduler run
void LimeLightSubsystem::Periodic() {}

double LimeLightSubsystem::GetXhorizontal(){

    Tx = NetWorkTable->GetNumber("Tx", 0.0);

    return Tx;
}

double LimeLightSubsystem::GetYvertical(){

    Ty = NetWorkTable->GetNumber("Ty", 0.0);

    return Tx;
}

double LimeLightSubsystem::GetAarea(){

    Ta = NetWorkTable->GetNumber("Ta", 0.0);

    return Ta;
}

double LimeLightSubsystem::GetError(){

    TSkew = NetWorkTable->GetNumber("TSkew", 0.0);

    return TSkew;
}


