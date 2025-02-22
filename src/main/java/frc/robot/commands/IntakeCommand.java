// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {
  private IntakeSubsystem intakeWheels;
  private DoubleSupplier speed;
  Timer timer = new Timer();    
  private boolean timerStarted = false;

  /** Creates a new Intake. */
  public IntakeCommand(IntakeSubsystem intakeWheels, DoubleSupplier speed) {
    this.intakeWheels = intakeWheels;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeWheels);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean limitSwitchTriggered = intakeWheels.getLimit();

    if (!limitSwitchTriggered) { // Beam broken
        if (!timerStarted) { 
            timer.reset();
            timer.start();
            timerStarted = true;
        }

        if (timer.get() >= Constants.IntakeConstants.intakeStopDelay) { // variable to change wait time
            intakeWheels.intake(0);
        } else {
            intakeWheels.intake(speed.getAsDouble());
        }
    } else { // If beam is restored reset timer
        intakeWheels.intake(speed.getAsDouble()); 
        timer.reset();
        timer.stop();
        timerStarted = false;
    }

    SmartDashboard.putBoolean("Limit switch: ", limitSwitchTriggered);
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
