// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private MecanumDrive mecanumDrive;
  private AHRS gyro;

  private SparkMax topLeft;
  private SparkMax bottomLeft;
  private SparkMax topRight;
  private SparkMax bottomRight;

  private RelativeEncoder topLeftEncoder;
  private RelativeEncoder bottomLeftEncoder;
  private RelativeEncoder topRightEncoder;
  private RelativeEncoder bottomRightEncoder;

  private MecanumDriveOdometry drivePose;

  /** Creates a new MecanumDrive. */
  public Drivetrain() {
    topLeft = new SparkMax(0, MotorType.kBrushless);
    bottomLeft = new SparkMax(1, MotorType.kBrushless);
    topRight = new SparkMax(2, MotorType.kBrushless);
    bottomRight = new SparkMax(3, MotorType.kBrushless);

    topLeftEncoder = topLeft.getEncoder();
    bottomLeftEncoder = bottomLeft.getEncoder();
    topRightEncoder = topRight.getEncoder();
    bottomRightEncoder = bottomRight.getEncoder();

    SparkMaxConfig configTopLeft = new SparkMaxConfig();
    SparkMaxConfig configBottomLeft = new SparkMaxConfig();
    SparkMaxConfig configTopRight = new SparkMaxConfig();
    SparkMaxConfig configBottomRight = new SparkMaxConfig();

    //Encoder config factors are arbitrary placeholders and should be changed
    configTopLeft
    .inverted(false)
    .idleMode(IdleMode.kBrake);
    configTopLeft.encoder
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);
    configTopLeft.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.0, 0.0, 0.0);
    
    configBottomLeft
    .inverted(true)
    .idleMode(IdleMode.kBrake);
    configBottomLeft.encoder
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);
    configBottomLeft.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.0, 0.0, 0.0);
    
    configTopRight
    .inverted(true)
    .idleMode(IdleMode.kBrake);
    configTopRight.encoder
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);
    configTopRight.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.0, 0.0, 0.0);
    
    configBottomRight
    .inverted(false)
    .idleMode(IdleMode.kBrake);
    configBottomRight.encoder
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);
    configBottomRight.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.0, 0.0, 0.0);
    
    topLeft.configure(configTopLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomLeft.configure(configBottomLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topRight.configure(configTopRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomRight.configure(configBottomRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    gyro = new AHRS(NavXComType.kMXP_SPI);
    
    mecanumDrive = new MecanumDrive(topLeft, bottomLeft, topRight, bottomRight);
  }

  public void drive(double xSpeed, double ySpeed, double zRotation, boolean fieldOriented) {
    if (fieldOriented) {
      mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation, gyro.getRotation2d());
    }
    else{
      mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation);
    }


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
