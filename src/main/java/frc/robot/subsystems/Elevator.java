// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;



public class Elevator extends SubsystemBase {

  private final SparkMax elevatorLeader;
  private final SparkMax elevatorFollower;

  private final RelativeEncoder encoder;

  private final DigitalInput lowLimitSwitch;
  private final DigitalInput highLimitSwitch;
  private final SparkClosedLoopController controller;

  private final double[] setpoints = {
    ElevatorConstants.INTAKE_POSITION,
    ElevatorConstants.L1_POSITION,
    ElevatorConstants.L2_POSITION,
    ElevatorConstants.L3_POSITION,
    ElevatorConstants.L4_POSITION,
  };

  private final String[] setpointNames = {
    "Intake",
    "L1",
    "L2",
    "L3",
    "L4"
  };

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

    encoder = elevatorLeader.getEncoder();
    encoder.setPosition(0);

    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    leaderConfig
        .smartCurrentLimit(40)
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    leaderConfig.encoder
        .positionConversionFactor(ElevatorConstants.COUNTS_TO_INCHES_CONVERSION);
       // .inverted(true);
    leaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(ElevatorConstants.P_UP, ElevatorConstants.I_UP, ElevatorConstants.D_UP, ElevatorConstants.F_UP, ElevatorConstants.PID_SLOT_UP)
        .pidf(ElevatorConstants.P_DOWN, ElevatorConstants.I_DOWN, ElevatorConstants.D_DOWN, ElevatorConstants.F_DOWN, ElevatorConstants.PID_SLOT_DOWN)
        .outputRange(-ElevatorConstants.MAX_PID_OUTPUT_UP, ElevatorConstants.MAX_PID_OUTPUT_UP, ElevatorConstants.PID_SLOT_UP)
        .outputRange(-ElevatorConstants.MAX_PID_OUTPUT_DOWN, ElevatorConstants.MAX_PID_OUTPUT_DOWN, ElevatorConstants.PID_SLOT_DOWN);

    followerConfig
        .smartCurrentLimit(40)
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    followerConfig.follow(ElevatorConstants.LEADER_PORT, true);

    elevatorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    controller = elevatorLeader.getClosedLoopController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (lowLimitSwitch.get() || highLimitSwitch.get()) {
      setSpeed(0);
    }

    SmartDashboard.putBoolean("Low Limit", lowLimitSwitch.get());
    SmartDashboard.putBoolean("High Limit", highLimitSwitch.get());
    SmartDashboard.putNumber("Encoder Position", getPosition());
    SmartDashboard.putNumber("Leader output", elevatorLeader.get());
    SmartDashboard.putNumber("Follower output", elevatorFollower.get());
    SmartDashboard.putString("Current Elevator Settpoint", getCurrSetpoint());

    // if (lowLimitSwitch.get()) {
    //   encoder.setPosition(0);
    // }

    // if (highLimitSwitch.get()) {
    //   encoder.setPosition(ElevatorConstants.MAX_POSITION);
    // }

  }

  public void setSpeed(double speed) {
    elevatorLeader.set(speed*ElevatorConstants.MANUAL_SPEED_MULTIPLIER);
  }

  public void setPosition(double position){

    //Use the appropriate controller based on direction (up or down)
    if (getPosition() <= position) {
      controller.setReference(position, ControlType.kPosition, ElevatorConstants.PID_SLOT_UP);
    }
    else {
      controller.setReference(position, ControlType.kPosition, ElevatorConstants.PID_SLOT_DOWN);
    }
  }

  public double getPosition(){
    //Encoder inversion
    return -encoder.getPosition();
  }

  public void setEncoderPosition(double position){
    encoder.setPosition(position);
  }

  public void resetEncoder(){
    encoder.setPosition(0);
  }

  public double getNextSetpoint() {
    for (int i = 0; i < setpoints.length - 1; i++) {

      //edge case 1: elevator is at max position
      if (MathUtil.isNear(setpoints[setpoints.length-1], getPosition(), 1.5)){
        return setpoints[0];
      }

      //edge case 2: elevator is at a setpoint (with tolerance)
      if (MathUtil.isNear(setpoints[i], getPosition(), 1.5)) {
        return setpoints[i+1];
      }

      //elevator is between setpoints, but not close to any of them
      if (getPosition() > setpoints[i] && getPosition() < setpoints[i+1] && !MathUtil.isNear(setpoints[i+1], getPosition(), 1.5)) {
        return setpoints[i+1];
      }
    }

    return getPosition();
  }

  public double getPrevSetpoint() {
    for (int i = 1; i < setpoints.length; i++) {

      //edge case 1: elevator is at min position
      if (MathUtil.isNear(setpoints[0], getPosition(), 1.5)){
        return setpoints[setpoints.length-1];
      }

      //edge case 2: elevator is at a setpoint (with tolerance)
      if (MathUtil.isNear(setpoints[i], getPosition(), 1.5)) {
        return setpoints[i-1];
      }

      //elevator is between setpoints, but not close to any of them
      if (getPosition() < setpoints[i] && getPosition() > setpoints[i-1] && !MathUtil.isNear(setpoints[i-1], getPosition(), 1.5)) {
        return setpoints[i-1];
      }
    }

    return getPosition();
  }

  public String getCurrSetpoint(){
    for (int i = 0; i < setpoints.length - 1; i++) {

      //edge case 1: elevator is at max position
      if (MathUtil.isNear(setpoints[setpoints.length-1], getPosition(), 1.5)){
        return setpointNames[setpointNames.length-1];
      }

      //edge case 2: elevator is at a setpoint
      if (MathUtil.isNear(setpoints[i], getPosition(), 1.5)) {
        return setpointNames[i];
      }

      //elevator is between setpoints
      if (getPosition() > setpoints[i] && getPosition() < setpoints[i+1]) {
        if (Math.abs(getPosition() - setpoints[i]) < Math.abs(getPosition() - setpoints[i+1])) {
          return setpointNames[i];
        }
        else {
          return setpointNames[i+1];
        }
      }
    }

    return "N/A";
  }
}
