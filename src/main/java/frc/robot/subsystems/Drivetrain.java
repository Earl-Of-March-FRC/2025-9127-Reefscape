// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.lang.annotation.Target;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoConstants;
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

  private SparkClosedLoopController topLeftController;
  private SparkClosedLoopController topRightController;
  private SparkClosedLoopController bottomLeftController;
  private SparkClosedLoopController bottomRightController;

  private SparkRelativeEncoderSim topLeftEncoderSim;
  private SparkRelativeEncoderSim bottomLeftEncoderSim;
  private SparkRelativeEncoderSim topRightEncoderSim;
  private SparkRelativeEncoderSim bottomRightEncoderSim;

  private AHRS gyro;

  private MecanumDrivePoseEstimator poseEstimator;
  private MecanumDriveKinematics driveKinematics;
  private Field2d field;
  private AprilTagFieldLayout fieldLayout;

  private Pose2d drivePose;

  private Supplier<Optional<Pose2d>> limelightRobotPoseSupplier;

  // Field oriented drive on by default
  private boolean fieldOriented = false;
  private boolean slowMode = false;
  private RobotConfig robotConfig;


  /** Creates a new MecanumDrive. */
  public Drivetrain(Supplier<Optional<Pose2d>> limelightRobotPose) {
    this.limelightRobotPoseSupplier = limelightRobotPose;

    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

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
        .pid(DrivetrainConstants.VELOCITY_KP, DrivetrainConstants.VELOCITY_KI, DrivetrainConstants.VELOCITY_KD)
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
        .pid(DrivetrainConstants.VELOCITY_KP, DrivetrainConstants.VELOCITY_KI, DrivetrainConstants.VELOCITY_KD)
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
        .pid(DrivetrainConstants.VELOCITY_KP, DrivetrainConstants.VELOCITY_KI, DrivetrainConstants.VELOCITY_KD)
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
        .pid(DrivetrainConstants.VELOCITY_KP, DrivetrainConstants.VELOCITY_KI, DrivetrainConstants.VELOCITY_KD)
        .velocityFF(DrivetrainConstants.VELOCITY_Kf);

    topLeft.configure(configTopLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomLeft.configure(configBottomLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topRight.configure(configTopRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomRight.configure(configBottomRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    topLeftController = topLeft.getClosedLoopController();
    topRightController = topRight.getClosedLoopController();
    bottomLeftController = bottomLeft.getClosedLoopController();
    bottomRightController =bottomRight.getClosedLoopController();

    // Simulation
    topLeftEncoderSim = new SparkRelativeEncoderSim(topLeft);
    topLeftEncoderSim.setPositionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION);
    bottomLeftEncoderSim = new SparkRelativeEncoderSim(bottomLeft);
    bottomLeftEncoderSim.setPositionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION);
    topRightEncoderSim = new SparkRelativeEncoderSim(topRight);
    topRightEncoderSim.setPositionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION);
    bottomRightEncoderSim = new SparkRelativeEncoderSim(bottomRight);
    bottomRightEncoderSim.setPositionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION);

    mecanumDrive = new MecanumDrive(topLeft, bottomLeft, topRight, bottomRight){
      @Override
      public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
          if (!isSafetyEnabled()) {
            setSafetyEnabled(true);
          }
          super.driveCartesian(xSpeed, ySpeed, zRotation);
      }

      @Override
      public void driveCartesian(double xSpeed, double ySpeed, double zRotation, Rotation2d gyroAngle) {
          if (!isSafetyEnabled()) {
            setSafetyEnabled(true);
          }
          super.driveCartesian(xSpeed, ySpeed, zRotation, gyroAngle);
      }
    };

    gyro = new AHRS(NavXComType.kMXP_SPI);
    // angle adjustement relative to the front of the bot, + the angle of the bot
    // relative to the field
    gyro.setAngleAdjustment(Constants.DrivetrainConstants.GYRO_ANGLE_OFFSET + LimelightHelpers.getBotPose2d("limelight").getRotation().getDegrees());



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

  public void setVelocity(double velMPS){

    if (mecanumDrive.isSafetyEnabled()) {
      mecanumDrive.setSafetyEnabled(false); 
    }

    topLeft.getClosedLoopController().setReference(velMPS, ControlType.kVelocity);
    bottomLeft.getClosedLoopController().setReference(velMPS, ControlType.kVelocity);
    topRight.getClosedLoopController().setReference(velMPS, ControlType.kVelocity);
    bottomRight.getClosedLoopController().setReference(velMPS, ControlType.kVelocity);

    SmartDashboard.putNumber("Velocity setpoint", velMPS);
    SmartDashboard.putNumber("Top Left Velocity", topLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Top Right Velocity", topRightEncoder.getVelocity());
    SmartDashboard.putNumber("Bottom Left Velocity", bottomLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Bottom Right Velocity", bottomRightEncoder.getVelocity());
  }

  // Robot-relative drive using chassis speeds
  // public void drive(ChassisSpeeds speeds) {

  //   if (mecanumDrive.isSafetyEnabled()) {
  //     mecanumDrive.setSafetyEnabled(false); 
  //   }
    
  //   speeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vxMetersPerSecond, -speeds.omegaRadiansPerSecond);

  //   MecanumDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(speeds);
  //   wheelSpeeds.desaturate(DrivetrainConstants.MAX_SPEED_MPS);

    
  //   if (mecanumDrive.isSafetyEnabled()) {
  //     mecanumDrive.setSafetyEnabled(false); 
  //   }

  //   topLeft.getClosedLoopController().setReference(wheelSpeeds.frontLeftMetersPerSecond , ControlType.kVelocity);
  //   bottomLeft.getClosedLoopController().setReference(wheelSpeeds.rearLeftMetersPerSecond, ControlType.kVelocity);
  //   topRight.getClosedLoopController().setReference(wheelSpeeds.frontRightMetersPerSecond, ControlType.kVelocity);
  //   bottomRight.getClosedLoopController().setReference(wheelSpeeds.rearRightMetersPerSecond, ControlType.kVelocity);
  // }

  public void drive(ChassisSpeeds speeds) {
    mecanumDrive.driveCartesian(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond);
  }
  
  public Pose2d getDrivePose() {
    return poseEstimator.getEstimatedPosition();
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

  //Id ranges from 1-6, regardeless of alliance (this is due to allianceIdOffset).
  /* Field layout:
   *                  Blue              Red
   *               |  3/\4              4/\3   |
   * Driver Station| 2|  |5            5|  |2  | Driver station
   *               |  1\/6              6\/1   | 
   */
  /*Current assumptions:
  Robot heading is 0 when robot is facing towards the red wall
  April tag heading is mesured CCW
  */
  //TODO handle edge cases (ex: optionals don't exist)
  public Command moveToTagCommand(int tagID) {
    Pose2d startingPose = getDrivePose();
    Pose2d targetPose = new Pose2d();

    //Blue by default
    int allianceIdOffset = 16;
    if (DriverStation.getAlliance().isPresent()) {
      allianceIdOffset = DriverStation.getAlliance().get() == Alliance.Blue ? 16 : 5;
    }

    //Get the pose of the tag if it is present
    Optional<Pose3d> targetPose3D = fieldLayout.getTagPose(allianceIdOffset + tagID);
    if (targetPose3D.isPresent()) {
      targetPose = targetPose3D.get().toPose2d();
    }
    else {
      System.out.println("Tag not found");
      return null;
    }

    //Transform April tag Pose2D into the desired robot Pose2D
    targetPose = new Pose2d(
      targetPose.getTranslation()
        .plus(new Translation2d(
          Math.cos(targetPose.getRotation().getRadians()),
          Math.sin(targetPose.getRotation().getRadians()))
        ).times(0.5),
      targetPose.getRotation().plus(Rotation2d.fromDegrees(180))
    );

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startingPose, targetPose);
    PathPlannerPath path = new PathPlannerPath(waypoints, DrivetrainConstants.kPathfindingConstraints, null,
        new GoalEndState(0, targetPose.getRotation()));
    path.preventFlipping = true;

    
    //TODO: decide which method is better for compiling the path, ensure that obstacles are avoided
    return AutoBuilder.pathfindToPose(targetPose, DrivetrainConstants.kPathfindingConstraints, 0);
    //return AutoBuilder.followPath(path);
  }

  public Command moveToLeftStationCommand() {
    return AutoBuilder.pathfindToPose(AutoConstants.LEFT_STATION_POSE, DrivetrainConstants.kPathfindingConstraints, 0);
  }

  public Command moveToRightStationCommand() {
    return AutoBuilder.pathfindToPose(AutoConstants.RIGHT_STATION_POSE, DrivetrainConstants.kPathfindingConstraints, 0);
  }

  public Command moveToNearestStationCommandCommand() {
    //go to the station that is closest to the robot, this does not take into account obstacles
    if (getDrivePose().getTranslation().minus(AutoConstants.LEFT_STATION_POSE.getTranslation()).getNorm() >
        getDrivePose().getTranslation().minus(AutoConstants.RIGHT_STATION_POSE.getTranslation()).getNorm()) {
      return moveToRightStationCommand();
    } else {
      return moveToLeftStationCommand();
    }
  }

  @Override
  public void periodic() {
    Optional<Pose2d> limelightPose = limelightRobotPoseSupplier.get();

    SmartDashboard.putNumber("Top Left Velocity", topLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Top Right Velocity", topRightEncoder.getVelocity());
    SmartDashboard.putNumber("Bottom Left Velocity", bottomLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Bottom Right Velocity", bottomRightEncoder.getVelocity());

    SmartDashboard.putNumber("Top Left Output", topLeft.getAppliedOutput());
    SmartDashboard.putNumber("Top Right Output", topRight.getAppliedOutput());
    SmartDashboard.putNumber("Bottom Left Output", bottomLeft.getAppliedOutput());
    SmartDashboard.putNumber("Bottom Right Output", bottomRight.getAppliedOutput());

    
    // In your periodic function:
    LimelightHelpers.PoseEstimate limelightMeasurement1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("left");
    if (limelightMeasurement1.tagCount >= 1) {  // Only trust measurement if we see multiple tags
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        poseEstimator.addVisionMeasurement(
            limelightMeasurement1.pose,
            limelightMeasurement1.timestampSeconds
      );
    }

    LimelightHelpers.PoseEstimate limelightMeasurement2 = LimelightHelpers.getBotPoseEstimate_wpiBlue("right");
    if (limelightMeasurement2.tagCount >= 1) {  // Only trust measurement if we see multiple tags
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        poseEstimator.addVisionMeasurement(
            limelightMeasurement2.pose,
            limelightMeasurement2.timestampSeconds
      );
    }

    // This method will be called once per scheduler run
    drivePose = poseEstimator.update(gyro.getRotation2d(),
        //Rotation2d.fromDegrees(drivePose.getRotation().getDegrees() + gyro.getRate() * 0.02),
        new MecanumDriveWheelPositions(
            topLeftEncoder.getPosition(),
            topRightEncoder.getPosition(),
            bottomLeftEncoder.getPosition(),
            bottomRightEncoder.getPosition()));


    field.setRobotPose(drivePose);


    SmartDashboard.putData("Field", field);

    SmartDashboard.putBoolean("Field Oriented", fieldOriented);
  }

  @Override
  public void simulationPeriodic() {
    Pose2d drivePose = poseEstimator.update(gyro.getRotation2d(),
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