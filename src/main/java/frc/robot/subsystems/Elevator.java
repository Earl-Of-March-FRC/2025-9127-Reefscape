// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



public class Elevator extends SubsystemBase {
  private final WPI_TalonSRX elevatorMotor;

  /** Creates a new Elevator. */
  
  public Elevator() {

    elevatorMotor = new WPI_TalonSRX(Constants.ElevatorConstants.MOTOR_PORT);

    // Configure the encoder
    elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.ElevatorConstants.PID_SLOT, 10);
    elevatorMotor.configSelectedFeedbackCoefficient(Constants.ElevatorConstants.COUNTS_TO_INCHES_CONVERSION, Constants.ElevatorConstants.PID_SLOT, 10);
    // Configure PID coefficients
    elevatorMotor.config_kP(Constants.ElevatorConstants.PID_SLOT, Constants.ElevatorConstants.P, 10);
    elevatorMotor.config_kI(Constants.ElevatorConstants.PID_SLOT, Constants.ElevatorConstants.I, 10);
    elevatorMotor.config_kD(Constants.ElevatorConstants.PID_SLOT, Constants.ElevatorConstants.D, 10);
    elevatorMotor.config_kF(Constants.ElevatorConstants.PID_SLOT, Constants.ElevatorConstants.F, 10);

    // Set neutral mode
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setSpeed(double speed) {
    elevatorMotor.set(speed);
  }

  public void setPosition(double position){
    elevatorMotor.set(ControlMode.Position, position);
  }

  public double getPosition(){
    return elevatorMotor.getSelectedSensorPosition();
  }

  public double getEncoderPosition(){
    return elevatorMotor.getSelectedSensorPosition();
  }

  public void setEncoderPosition(double position){
    elevatorMotor.setSelectedSensorPosition(position);
  }

}
