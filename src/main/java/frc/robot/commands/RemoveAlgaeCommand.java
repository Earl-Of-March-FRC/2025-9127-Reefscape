// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeRemove;

public class RemoveAlgaeCommand extends Command {
  private AlgaeRemove algaeRemove;
  private DoubleSupplier speed;

  /** Creates a new Intake. */
  public RemoveAlgaeCommand(AlgaeRemove algaeRemove, DoubleSupplier speed) {
    this.algaeRemove = algaeRemove;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeRemove);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeRemove.remove(speed.getAsDouble());
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
