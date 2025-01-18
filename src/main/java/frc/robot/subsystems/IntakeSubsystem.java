

    // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase {
  private final TalonSRX motor2 = new WPI_TalonSRX(Constants.IntakeConstants.moto1pin);
  private final TalonSRX motor1 = new WPI_TalonSRX(Constants.IntakeConstants.moto2pin);

  public void intake(double speed){
    motor1.set(TalonSRXControlMode.PercentOutput, speed);
    motor2.set(TalonSRXControlMode.PercentOutput, -speed);
  }

  @Override
  public void periodic() {
 
  }

  @Override
  public void simulationPeriodic() {
 
  }

}

