// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    //Simulation
    topLeftEncoderSim = new SparkRelativeEncoderSim(topLeft);
    topLeftEncoderSim.setPositionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION);
    bottomLeftEncoderSim = new SparkRelativeEncoderSim(bottomLeft);
    bottomLeftEncoderSim.setPositionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION);
    topRightEncoderSim = new SparkRelativeEncoderSim(topRight);
    topRightEncoderSim.setPositionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION);
    bottomRightEncoderSim = new SparkRelativeEncoderSim(bottomRight);
    bottomRightEncoderSim.setPositionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION);

    topLeftEncoder.setPosition(0);
    bottomLeftEncoder.setPosition(0);
    topRightEncoder.setPosition(0);
    bottomRightEncoder.setPosition(0);

    SparkMaxConfig configTopLeft = new SparkMaxConfig();
    SparkMaxConfig configBottomLeft = new SparkMaxConfig();
    SparkMaxConfig configTopRight = new SparkMaxConfig();
    SparkMaxConfig configBottomRight = new SparkMaxConfig();

    configTopLeft
    .inverted(false)
    .idleMode(IdleMode.kBrake);
    configTopLeft.encoder
    .positionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION);
    //.velocityConversionFactor(1000);
    configTopLeft.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.0, 0.0, 0.0);
    
    configBottomLeft
    .inverted(true)
    .idleMode(IdleMode.kBrake);
    configBottomLeft.encoder
    .positionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION);
    //.velocityConversionFactor(1000);
    configBottomLeft.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.0, 0.0, 0.0);
    
    configTopRight
    .inverted(false)
    .idleMode(IdleMode.kBrake);
    configTopRight.encoder
    .positionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION);
    //.velocityConversionFactor(1000);
    configTopRight.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.0, 0.0, 0.0);
    
    configBottomRight
    .inverted(true)
     .idleMode(IdleMode.kBrake);
    configBottomRight.encoder
    .positionConversionFactor(Constants.DrivetrainConstants.COUNTS_TO_METERS_CONVERSION);
    //.velocityConversionFactor(1000);
    configBottomRight.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.0, 0.0, 0.0);
    
    topLeft.configure(configTopLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomLeft.configure(configBottomLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topRight.configure(configTopRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomRight.configure(configBottomRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    mecanumDrive = new MecanumDrive(topLeft, bottomLeft, topRight, bottomRight);
    
    gyro = new AHRS(NavXComType.kMXP_SPI);
    gyro.reset();
    
    driveOdometry = new MecanumDriveOdometry(
      new MecanumDriveKinematics(
        Constants.DrivetrainConstants.TOP_LEFT_POS,
        Constants.DrivetrainConstants.TOP_RIGHT_POS,
        Constants.DrivetrainConstants.BOTTOM_LEFT_POS,
        Constants.DrivetrainConstants.BOTTOM_RIGHT_POS
      ),
      gyro.getRotation2d(),
      new MecanumDriveWheelPositions(
        topLeftEncoder.getPosition(), 
        topRightEncoder.getPosition(),
        bottomLeftEncoder.getPosition(),
        bottomRightEncoder.getPosition()
      )
    );
  }

  public void driveWithVision(ChassisSpeeds speeds) {
    mecanumDrive.driveCartesian(
      speeds.vxMetersPerSecond*Constants.DrivetrainConstants.SPEED_MULTIPLIER, 
      speeds.vyMetersPerSecond*Constants.DrivetrainConstants.SPEED_MULTIPLIER, 
      speeds.omegaRadiansPerSecond*Constants.DrivetrainConstants.SPEED_MULTIPLIER
      );
  }

  public void drive(double xSpeed, double ySpeed, double zRotation, boolean fieldOriented) {
    // if (fieldOriented) {
    //   mecanumDrive.driveCartesian(xSpeed*Constants.DrivetrainConstants.SPEED_MULTIPLIER, ySpeed*Constants.DrivetrainConstants.SPEED_MULTIPLIER, zRotation, gyro.getRotation2D());
    // }
    // else{
      mecanumDrive.driveCartesian(xSpeed*Constants.DrivetrainConstants.SPEED_MULTIPLIER, ySpeed*Constants.DrivetrainConstants.SPEED_MULTIPLIER, zRotation*Constants.DrivetrainConstants.SPEED_MULTIPLIER);
    //}
  }
  
  public void stop(){
    mecanumDrive.driveCartesian(0, 0, 0);
  }


  public Pose2d getDrivePose() {
      return drivePose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    drivePose = 
    driveOdometry.update(gyro.getRotation2d(), 
      new MecanumDriveWheelPositions(
        topLeftEncoder.getPosition(), 
        topRightEncoder.getPosition(),
        bottomLeftEncoder.getPosition(),
        bottomRightEncoder.getPosition()
      )
    );

    field.setRobotPose(drivePose);
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void simulationPeriodic() {
    drivePose = 
    driveOdometry.update(gyro.getRotation2d(), 
      new MecanumDriveWheelPositions(
        topLeftEncoderSim.getPosition(), 
        topRightEncoderSim.getPosition(),
        bottomLeftEncoderSim.getPosition(),
        bottomRightEncoderSim.getPosition()
      )
    );

    topLeftEncoderSim.iterate(topLeft.get()*Constants.DrivetrainConstants.SIM_MAX_VELOCITY, 0.02);
    bottomLeftEncoderSim.iterate(bottomLeft.get()*Constants.DrivetrainConstants.SIM_MAX_VELOCITY, 0.02);
    topRightEncoderSim.iterate(topRight.get()*Constants.DrivetrainConstants.SIM_MAX_VELOCITY, 0.02);
    bottomRightEncoderSim.iterate(bottomRight.get()*Constants.DrivetrainConstants.SIM_MAX_VELOCITY, 0.02);

    field.setRobotPose(drivePose);
    SmartDashboard.putData("Field", field);
  }
}