// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.LedMode;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LedCommand extends Command {

  LimelightSubsystem limelight = new LimelightSubsystem();
  IntakeSubsystem intake = new IntakeSubsystem();
  Elevator elevator = new Elevator();

  /** Creates a new LedCommand. */
  public LedCommand() {
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (elevator.getCurrSetpoint().equals("Intake") && intake.getLimit()==true){
      limelight.setLedMode(LedMode.BLINK);
    }
    else if(intake.getLimit()==false){
      limelight.setLedMode(LedMode.ON);
    }
    else{
      limelight.setLedMode(LedMode.OFF);
    }
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
