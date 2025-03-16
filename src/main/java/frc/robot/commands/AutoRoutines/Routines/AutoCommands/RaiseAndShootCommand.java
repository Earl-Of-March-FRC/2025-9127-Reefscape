package frc.robot.commands.AutoRoutines.Routines.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeSubsystem;

public class RaiseAndShootCommand extends Command {
  private final IntakeSubsystem intake;
  private final Elevator elevator;
  private final double setpoint;
  Timer timer = new Timer();
  
  public RaiseAndShootCommand(IntakeSubsystem intake, Elevator elevator, double setpoint) {
    this.intake = intake;
    this.elevator = elevator;
    this.setpoint = setpoint;
    addRequirements(intake, elevator);
  }

  @Override
  public void initialize() {
    System.out.println("RaiseAndShootCommand Started");
  }

  @Override
  public void execute() {
    elevator.setPosition(-setpoint);

    if (MathUtil.isNear(setpoint, elevator.getPosition(), 2)){
      intake.intake(0.5);

      if (!timer.isRunning()) {
        timer.start();
      }
    }//Constants.IntakeConstants.SHOOT_MULTIPLIER);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("AutoShootCommand Ended");
    intake.intake(0);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1.5); 
  }
}
