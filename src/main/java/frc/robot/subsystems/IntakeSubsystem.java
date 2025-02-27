// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase {
  private final WPI_VictorSPX motor1 = new WPI_VictorSPX(Constants.IntakeConstants.moto1pin);
  private final WPI_VictorSPX motor2 = new WPI_VictorSPX(Constants.IntakeConstants.moto2pin);

  private final DigitalInput limit =  new DigitalInput(Constants.IntakeConstants.LIMIT_SWITCH_CHANNEL);

  public void intake(double speed){
    motor1.set(VictorSPXControlMode.PercentOutput, speed);
    motor2.set(VictorSPXControlMode.PercentOutput, -speed);
  }

  public boolean getLimit(){
    return limit.get();
  }
}

