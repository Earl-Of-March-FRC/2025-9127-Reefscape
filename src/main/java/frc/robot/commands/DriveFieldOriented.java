// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveFieldOriented extends Command {
  private Drivetrain drivetrain;
  private DoubleSupplier xSpeed;
  private DoubleSupplier ySpeed;
  private DoubleSupplier zRotation;

  /** Creates a new Drive. */
  public DriveFieldOriented(Drivetrain drivetrain, boolean fieldOriented, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zRotation) {
    this.drivetrain = drivetrain;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), zRotation.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
