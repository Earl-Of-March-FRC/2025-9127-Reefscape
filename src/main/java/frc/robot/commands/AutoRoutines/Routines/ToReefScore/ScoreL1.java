
package frc.robot.commands.AutoRoutines.Routines.ToReefScore;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoRoutines.Routines.AutoCommands.AutoShootCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class ScoreL1 extends SequentialCommandGroup {
  public ScoreL1(IntakeSubsystem intake, String pathName) {
    addCommands(
        new PathPlannerAuto(pathName), // Drive to the scoring location
        new AutoShootCommand(intake).withTimeout(2) // Shoot for 2 seconds
    );
  }
}
