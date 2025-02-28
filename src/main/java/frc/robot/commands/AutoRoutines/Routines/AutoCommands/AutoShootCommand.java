package frc.robot.commands.AutoRoutines.Routines.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoShootCommand extends Command {
  private final IntakeSubsystem intake;
  
  public AutoShootCommand(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    System.out.println("AutoShootCommand Started");
  }

  @Override
  public void execute() {
    intake.intake(Constants.IntakeConstants.SHOOT_MULTIPLIER);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("AutoShootCommand Ended");
    intake.intake(0);
  }

  @Override
  public boolean isFinished() {
    return false; 
  }
}
