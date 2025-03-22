// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.LedMode;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LedCommand extends Command {

  private final LimelightSubsystem limelight;
  private final IntakeSubsystem intake;
  private final Elevator elevator;

  /** Creates a new LedCommand. */
  public LedCommand(LimelightSubsystem limelight, IntakeSubsystem intake, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.intake = intake;
    this.elevator = elevator;

    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (MathUtil.isNear(ElevatorConstants.INTAKE_POSITION, elevator.getPosition(), 1) && intake.getLimit()==true){
      limelight.setLedMode(LedMode.ON);
    }
    else if(intake.getLimit()==false){
      limelight.setLedMode(LedMode.OFF);
    }
    else{
      limelight.setLedMode(LedMode.BLINK);
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
