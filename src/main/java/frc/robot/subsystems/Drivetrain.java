// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.ControlType;
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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DrivetrainConstants;

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

  private MecanumDrivePoseEstimator poseEstimator;
  private MecanumDriveKinematics driveKinematics;
  private Pose2d drivePose;
  private Field2d field;

  private Supplier<Optional<Pose2d>> limelightRobotPoseSupplier;

  // Field oriented drive on by default
  private boolean fieldOriented = false;

  private boolean slowMode = false;

  private RobotConfig robotConfig;

  /** Creates a new MecanumDrive. */
  public Drivetrain(Supplier<Optional<Pose2d>> limelightRobotPose) {
    this.limelightRobotPoseSupplier = limelightRobotPose;

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
        .pid(0.0, 0.0, 0.0)
        .velocityFF(DrivetrainConstants.VELOCITY_Kf);

    configBottomLeft
        .smartCurrentLimit(40)
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    configBottomLeft.encoder
        .positionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION)
        .velocityConversionFactor(Constants.DrivetrainConstants.RPM_TO_MPS_CONVERSION);
    configBottomLeft.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0, 0.0, 0.0)
        .velocityFF(DrivetrainConstants.VELOCITY_Kf);

    configTopRight
        .smartCurrentLimit(40)
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    configTopRight.encoder
        .positionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION)
        .velocityConversionFactor(Constants.DrivetrainConstants.RPM_TO_MPS_CONVERSION);
    configTopRight.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0, 0.0, 0.0)
        .velocityFF(DrivetrainConstants.VELOCITY_Kf);

    configBottomRight
        .smartCurrentLimit(40)
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    configBottomRight.encoder
        .positionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION)
        .velocityConversionFactor(Constants.DrivetrainConstants.RPM_TO_MPS_CONVERSION);
    configBottomRight.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0, 0.0, 0.0)
        .velocityFF(DrivetrainConstants.VELOCITY_Kf);

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

    poseEstimator = new MecanumDrivePoseEstimator(
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
            bottomRightEncoder.getPosition()),
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    driveKinematics = new MecanumDriveKinematics(
      Constants.DrivetrainConstants.TOP_LEFT_POS,
      Constants.DrivetrainConstants.TOP_RIGHT_POS,
      Constants.DrivetrainConstants.BOTTOM_LEFT_POS,
      Constants.DrivetrainConstants.BOTTOM_RIGHT_POS
    );

    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (IOException | ParseException e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
      ()->getDrivePose(),
      pose->resetDrivePose(pose),
      ()->getChassisSpeeds(),
      (speeds,feedforwards)->drive(speeds),
      new PPHolonomicDriveController(
        new PIDConstants(Constants.DrivetrainConstants.TRANSLATE_P, Constants.DrivetrainConstants.TRANSLATE_I, Constants.DrivetrainConstants.TRANSLATE_D),
        new PIDConstants(Constants.DrivetrainConstants.ROTATE_P, Constants.DrivetrainConstants.ROTATE_I, Constants.DrivetrainConstants.ROTATE_D)),
      robotConfig, 
      ()->{
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()){
          return alliance.get()==DriverStation.Alliance.Red;
        }
        return false;
      },
    this);
  }

  // X and Y have been swapped as params due to Mechanum Drive class conceptions
  // Uses a square root curve rather than linear
  //Drive orientation:
  //X+ = drive right
  //Y+ = drive forward
  public void drive(double xSpeed, double ySpeed, double zRotation) {
    if (fieldOriented) {
      mecanumDrive.driveCartesian(
          Math.signum(ySpeed) * (slowMode? Constants.DrivetrainConstants.SLOW_SPEED_MULTIPLIER : Constants.DrivetrainConstants.SPEED_MULTIPLIER) *
              Math.sqrt(
                  Math.abs(
                      MathUtil.applyDeadband(ySpeed, Constants.DrivetrainConstants.DRIVE_DEADBAND)
                      )),
          Math.signum(xSpeed) * (slowMode? Constants.DrivetrainConstants.SLOW_SPEED_MULTIPLIER : Constants.DrivetrainConstants.SPEED_MULTIPLIER) *
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
        Math.signum(ySpeed) * (slowMode? Constants.DrivetrainConstants.SLOW_SPEED_MULTIPLIER : Constants.DrivetrainConstants.SPEED_MULTIPLIER) *
          Math.sqrt(
              Math.abs(
                  MathUtil.applyDeadband(ySpeed, Constants.DrivetrainConstants.DRIVE_DEADBAND)
                  )),
        Math.signum(xSpeed) * (slowMode? Constants.DrivetrainConstants.SLOW_SPEED_MULTIPLIER : Constants.DrivetrainConstants.SPEED_MULTIPLIER) *
            Math.sqrt(
                Math.abs(
                    MathUtil.applyDeadband(xSpeed, Constants.DrivetrainConstants.DRIVE_DEADBAND)
                    )),
        MathUtil.applyDeadband(zRotation, Constants.DrivetrainConstants.TURN_DEADBAND) * Constants.DrivetrainConstants.SPEED_MULTIPLIER
      );
    }
  }

  //For PID control
  public void driveRobotOriented(double xSpeed, double ySpeed, double zRotation) {
    mecanumDrive.driveCartesian(
        ySpeed*Constants.DrivetrainConstants.PID_SPEED_MULTIPLIER,
        xSpeed*Constants.DrivetrainConstants.PID_SPEED_MULTIPLIER,
        zRotation*Constants.DrivetrainConstants.PID_SPEED_MULTIPLIER
      );
  }

  //For PID control
  public void driveFieldOriented(double xSpeed, double ySpeed, double zRotation) {
    mecanumDrive.driveCartesian(
        ySpeed*Constants.DrivetrainConstants.PID_SPEED_MULTIPLIER,
        xSpeed*Constants.DrivetrainConstants.PID_SPEED_MULTIPLIER,
        zRotation*Constants.DrivetrainConstants.PID_SPEED_MULTIPLIER,
        gyro.getRotation2d().unaryMinus()
      );
  }

  // Robot-relative drive using chassis speeds
  public void drive(ChassisSpeeds speeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(DrivetrainConstants.MAX_SPEED_MPS);
    
    topLeft.getClosedLoopController().setReference(wheelSpeeds.frontLeftMetersPerSecond , ControlType.kVelocity);
    bottomLeft.getClosedLoopController().setReference(wheelSpeeds.rearLeftMetersPerSecond, ControlType.kVelocity);
    topRight.getClosedLoopController().setReference(wheelSpeeds.frontRightMetersPerSecond, ControlType.kVelocity);
    bottomRight.getClosedLoopController().setReference(wheelSpeeds.rearRightMetersPerSecond, ControlType.kVelocity);
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

  public double getBotAngleAdjustment() {
    return gyro.getAngleAdjustment();
  }

  public void toggleSlowMode() {
    slowMode = !slowMode;
  }

  public void changeDriveMode() {
    fieldOriented = !fieldOriented;
  }

  public void resetDrivePose(Pose2d pose) {
    poseEstimator.resetPosition(gyro.getRotation2d(), getWheelPositions(), pose); 
  }
    
public MecanumDriveWheelPositions getWheelPositions() {
  return new MecanumDriveWheelPositions(
    topLeftEncoder.getPosition(),
    topRightEncoder.getPosition(),
    bottomLeftEncoder.getPosition(),
    bottomRightEncoder.getPosition()
  );
}
    
  // Getting robot-relative chassis speeds
  public ChassisSpeeds getChassisSpeeds() {
    return driveKinematics.toChassisSpeeds(
      new MecanumDriveWheelSpeeds(
        topLeftEncoder.getVelocity(), 
        topRightEncoder.getVelocity(),
        bottomLeftEncoder.getVelocity(),
        bottomRightEncoder.getVelocity()
      )
    );
  }

  @Override
  public void periodic() {
    Optional<Pose2d> limelightPose = limelightRobotPoseSupplier.get();

    if(limelightPose.isPresent()){
      // In your periodic function:
      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      if (limelightMeasurement.tagCount >= 1) {  // Only trust measurement if we see multiple tags
          poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
          poseEstimator.addVisionMeasurement(
              limelightMeasurement.pose,
              limelightMeasurement.timestampSeconds
        );
      }
    }
    // This method will be called once per scheduler run
    poseEstimator.update(gyro.getRotation2d(),
        new MecanumDriveWheelPositions(
            topLeftEncoder.getPosition(),
            topRightEncoder.getPosition(),
            bottomLeftEncoder.getPosition(),
            bottomRightEncoder.getPosition()));

    drivePose = poseEstimator.getEstimatedPosition();

    field.setRobotPose(drivePose);


    SmartDashboard.putData("Field", field);

    SmartDashboard.putBoolean("Field Oriented", fieldOriented);
  }

  @Override
  public void simulationPeriodic() {
    drivePose = poseEstimator.update(gyro.getRotation2d(),
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
    // SmartDashboard.putData("Field", field);
  }
}