// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;



public class Elevator extends SubsystemBase {

  private final SparkMax elevatorLeader;
  private final SparkMax elevatorFollower;

  private final SparkAbsoluteEncoder encoder;

  private final DigitalInput lowLimitSwitch;
  private final DigitalInput highLimitSwitch;

  /** Creates a new Elevator. */
  
  public Elevator() {
    elevatorLeader = new SparkMax(ElevatorConstants.LEADER_PORT, MotorType.kBrushless);
    elevatorFollower = new SparkMax(ElevatorConstants.FOLLOWER_PORT, MotorType.kBrushless);

    lowLimitSwitch = new DigitalInput(ElevatorConstants.LOW_LIMIT_CHANNEL){
      @Override
      public boolean get(){
        return !super.get();
      }
    };
    
    highLimitSwitch = new DigitalInput(ElevatorConstants.HIGH_LIMIT_CHANNEL){
      @Override
      public boolean get(){
        return !super.get();
      }
    };

    encoder = elevatorLeader.getAbsoluteEncoder();

    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    leaderConfig
        .smartCurrentLimit(40)
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    leaderConfig.encoder
        .positionConversionFactor(ElevatorConstants.COUNTS_TO_METERS_CONVERSION);
    leaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pidf(ElevatorConstants.P_UP, ElevatorConstants.I_UP, ElevatorConstants.D_UP, ElevatorConstants.F_UP, ElevatorConstants.PID_SLOT_UP)
        .pidf(ElevatorConstants.P_DOWN, ElevatorConstants.I_DOWN, ElevatorConstants.D_DOWN, ElevatorConstants.F_DOWN, ElevatorConstants.PID_SLOT_DOWN)
        .outputRange(-1, 1, ElevatorConstants.PID_SLOT_UP)
        .outputRange(-1, 1, ElevatorConstants.PID_SLOT_DOWN);

    followerConfig
        .smartCurrentLimit(40)
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    followerConfig.follow(ElevatorConstants.LEADER_PORT, true);

    elevatorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (lowLimitSwitch.get() || highLimitSwitch.get()) {
      setSpeed(0);
    }

    SmartDashboard.putBoolean("Low Limit", lowLimitSwitch.get());
    SmartDashboard.putBoolean("High Limit", highLimitSwitch.get());
    SmartDashboard.putNumber("Encoder Position", encoder.getPosition());

    // if (lowLimitSwitch.get()) {
    //   encoder.setPosition(0);
    // }

    // if (highLimitSwitch.get()) {
    //   encoder.setPosition(ElevatorConstants.MAX_POSITION);
    // }
  }

  public void setSpeed(double speed) {
    elevatorLeader.set(speed);
  }

  public void setPosition(double position){
    SparkClosedLoopController controller = elevatorLeader.getClosedLoopController();

    //Use the appropriate controller based on direction (up or down)
    if (getPosition() <= position) {
      controller.setReference(position, ControlType.kPosition, ElevatorConstants.PID_SLOT_UP);
    }
    else {
      controller.setReference(position, ControlType.kPosition, ElevatorConstants.PID_SLOT_DOWN);
    }
  }

  public double getPosition(){
    return encoder.getPosition();
  }

  // public void setEncoderPosition(double position){
  //   encoder.setPosition(position);
  // }

}
