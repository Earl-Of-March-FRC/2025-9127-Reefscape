// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private MecanumDrive mecanumDrive;

  private SparkMax topLeft;
  private SparkMax bottomLeft;
  private SparkMax topRight;
  private SparkMax bottomRight;

  private RelativeEncoder topLeftEncoder;
  private RelativeEncoder bottomLeftEncoder;
  private RelativeEncoder topRightEncoder;
  private RelativeEncoder bottomRightEncoder;

  private SparkRelativeEncoderSim topLeftEncoderSim;
  private SparkRelativeEncoderSim bottomLeftEncoderSim;
  private SparkRelativeEncoderSim topRightEncoderSim;
  private SparkRelativeEncoderSim bottomRightEncoderSim;

  private AHRS gyro;

  private MecanumDriveOdometry driveOdometry;
  private Pose2d drivePose;
  private Field2d field;

  // Field oriented drive on by default
  private boolean fieldOriented = false;

  /** Creates a new MecanumDrive. */
  public Drivetrain() {
    field = new Field2d();
    SmartDashboard.putData("Field", field);

    topLeft = new SparkMax(Constants.DrivetrainConstants.TOP_LEFT_ID, MotorType.kBrushless);
    bottomLeft = new SparkMax(Constants.DrivetrainConstants.BOTTOM_LEFT_ID, MotorType.kBrushless);
    topRight = new SparkMax(Constants.DrivetrainConstants.TOP_RIGHT_ID, MotorType.kBrushless);
    bottomRight = new SparkMax(Constants.DrivetrainConstants.BOTTOM_RIGHT_ID, MotorType.kBrushless);

    topLeftEncoder = topLeft.getEncoder();
    bottomLeftEncoder = bottomLeft.getEncoder();
    topRightEncoder = topRight.getEncoder();
    bottomRightEncoder = bottomRight.getEncoder();

    topLeftEncoder.setPosition(0);
    bottomLeftEncoder.setPosition(0);
    topRightEncoder.setPosition(0);
    bottomRightEncoder.setPosition(0);

    // new Spark Max config syntax, each SparkMaxConfig object represents a
    // configuration of controller, encoder and PID
    // that is applied to the controller using the .configure() method

    SparkMaxConfig configTopLeft = new SparkMaxConfig();
    SparkMaxConfig configBottomLeft = new SparkMaxConfig();
    SparkMaxConfig configTopRight = new SparkMaxConfig();
    SparkMaxConfig configBottomRight = new SparkMaxConfig();

    configTopLeft
        .smartCurrentLimit(40)
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    configTopLeft.encoder
        .positionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION)
        .velocityConversionFactor(Constants.DrivetrainConstants.RPM_TO_MPS_CONVERSION);
    configTopLeft.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0, 0.0, 0.0);

    configBottomLeft
        .smartCurrentLimit(40)
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    configBottomLeft.encoder
        .positionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION)
        .velocityConversionFactor(Constants.DrivetrainConstants.RPM_TO_MPS_CONVERSION);
    configBottomLeft.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0, 0.0, 0.0);

    configTopRight
        .smartCurrentLimit(40)
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    configTopRight.encoder
        .positionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION)
        .velocityConversionFactor(Constants.DrivetrainConstants.RPM_TO_MPS_CONVERSION);
    configTopRight.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0, 0.0, 0.0);

    configBottomRight
        .smartCurrentLimit(40)
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    configBottomRight.encoder
        .positionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION)
        .velocityConversionFactor(Constants.DrivetrainConstants.RPM_TO_MPS_CONVERSION);
    configBottomRight.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0, 0.0, 0.0);

    topLeft.configure(configTopLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomLeft.configure(configBottomLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topRight.configure(configTopRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomRight.configure(configBottomRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Simulation
    topLeftEncoderSim = new SparkRelativeEncoderSim(topLeft);
    topLeftEncoderSim.setPositionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION);
    bottomLeftEncoderSim = new SparkRelativeEncoderSim(bottomLeft);
    bottomLeftEncoderSim.setPositionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION);
    topRightEncoderSim = new SparkRelativeEncoderSim(topRight);
    topRightEncoderSim.setPositionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION);
    bottomRightEncoderSim = new SparkRelativeEncoderSim(bottomRight);
    bottomRightEncoderSim.setPositionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION);

    mecanumDrive = new MecanumDrive(topLeft, bottomLeft, topRight, bottomRight);

    gyro = new AHRS(NavXComType.kMXP_SPI);
    // angle adjustement relative to the front of the bot, + the angle of the bot
    // relative to the field
    gyro.setAngleAdjustment(Constants.DrivetrainConstants.GYRO_ANGLE_OFFSET);

    driveOdometry = new MecanumDriveOdometry(
        new MecanumDriveKinematics(
            Constants.DrivetrainConstants.TOP_LEFT_POS,
            Constants.DrivetrainConstants.TOP_RIGHT_POS,
            Constants.DrivetrainConstants.BOTTOM_LEFT_POS,
            Constants.DrivetrainConstants.BOTTOM_RIGHT_POS),
        gyro.getRotation2d(),
        new MecanumDriveWheelPositions(
            topLeftEncoder.getPosition(),
            topRightEncoder.getPosition(),
            bottomLeftEncoder.getPosition(),
            bottomRightEncoder.getPosition()));
  }

  // X and Y have been swapped as params due to Mechanum Drive class conceptions
  // Uses a square root curve rather than linear
  public void drive(double xSpeed, double ySpeed, double zRotation) {
    if (fieldOriented) {
      mecanumDrive.driveCartesian(
          Math.signum(ySpeed) * Constants.DrivetrainConstants.SPEED_MULTIPLIER *
              Math.sqrt(
                  Math.abs(
                      MathUtil.applyDeadband(ySpeed, Constants.DrivetrainConstants.DRIVE_DEADBAND)
                      )),
          Math.signum(xSpeed) * Constants.DrivetrainConstants.SPEED_MULTIPLIER *
              Math.sqrt(
                  Math.abs(
                      MathUtil.applyDeadband(xSpeed, Constants.DrivetrainConstants.DRIVE_DEADBAND)
                      )),
          MathUtil.applyDeadband(zRotation, Constants.DrivetrainConstants.TURN_DEADBAND) * Constants.DrivetrainConstants.SPEED_MULTIPLIER,

          // The unary minus arises from the swapping of x and y
          gyro.getRotation2d().unaryMinus());
    } 
    else {
      mecanumDrive.driveCartesian(
          MathUtil.applyDeadband(
              Math.signum(ySpeed) * Math.sqrt(Math.abs(ySpeed * Constants.DrivetrainConstants.SPEED_MULTIPLIER)),
              Constants.DrivetrainConstants.DRIVE_DEADBAND),
          MathUtil.applyDeadband(
              Math.signum(xSpeed) * Math.sqrt(Math.abs(xSpeed * Constants.DrivetrainConstants.SPEED_MULTIPLIER)),
              Constants.DrivetrainConstants.DRIVE_DEADBAND),
            MathUtil.applyDeadband(zRotation, Constants.DrivetrainConstants.TURN_DEADBAND) * Constants.DrivetrainConstants.SPEED_MULTIPLIER);
    }
  }

  public Pose2d getDrivePose() {
    return drivePose;
  }

  public void resetGyro() {
    gyro.reset();
  }

  // Set the angle of the bot relative to the field, where 0 points forward, and
  // angles are mesured "CCW" (NOT CONFIRMED)
  public void setBotAngleAdjustment(double botAngleAdjustment) {
    gyro.setAngleAdjustment(Constants.DrivetrainConstants.GYRO_ANGLE_OFFSET + botAngleAdjustment);
  }

  public void changeDriveMode() {
    fieldOriented = !fieldOriented;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    drivePose = driveOdometry.update(gyro.getRotation2d(),
        new MecanumDriveWheelPositions(
            topLeftEncoder.getPosition(),
            topRightEncoder.getPosition(),
            bottomLeftEncoder.getPosition(),
            bottomRightEncoder.getPosition()));

    field.setRobotPose(drivePose);
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void simulationPeriodic() {
    drivePose = driveOdometry.update(gyro.getRotation2d(),
        new MecanumDriveWheelPositions(
            topLeftEncoderSim.getPosition(),
            topRightEncoderSim.getPosition(),
            bottomLeftEncoderSim.getPosition(),
            bottomRightEncoderSim.getPosition()));

    topLeftEncoderSim.iterate(topLeft.get() * Constants.DrivetrainConstants.SIM_MAX_VELOCITY, 0.02);
    bottomLeftEncoderSim.iterate(bottomLeft.get() * Constants.DrivetrainConstants.SIM_MAX_VELOCITY, 0.02);
    topRightEncoderSim.iterate(topRight.get() * Constants.DrivetrainConstants.SIM_MAX_VELOCITY, 0.02);
    bottomRightEncoderSim.iterate(bottomRight.get() * Constants.DrivetrainConstants.SIM_MAX_VELOCITY, 0.02);

    field.setRobotPose(drivePose);
    SmartDashboard.putData("Field", field);
  }
}