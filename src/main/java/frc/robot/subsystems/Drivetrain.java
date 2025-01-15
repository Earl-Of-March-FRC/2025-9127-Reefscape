// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private MecanumDrive mecanumDrive;
  private WPI_TalonSRX topLeft;
  private WPI_TalonSRX bottomLeft;
  private WPI_TalonSRX topRight;
  private WPI_TalonSRX bottomRight;


  /** Creates a new MecanumDrive. */
  public Drivetrain() {
    topLeft = new WPI_TalonSRX(0);
    bottomLeft = new WPI_TalonSRX(1);
    topRight = new WPI_TalonSRX(2);
    bottomRight = new WPI_TalonSRX(3);

    //check inversions
    topRight.setInverted(true);
    bottomRight.setInverted(true);
    
    
    mecanumDrive = new MecanumDrive(topLeft, bottomLeft, topRight, bottomRight);
  }

  public void drive(double xSpeed, double ySpeed, double zRotation) {
    mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
