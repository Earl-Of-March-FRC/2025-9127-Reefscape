
package frc.robot.commands.AutoRoutines.Routines.ToReefScore;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoRoutines.Routines.AutoCommands.AutoShootCommand;
import frc.robot.commands.ElevatorPID;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeSubsystem;


public class Score extends SequentialCommandGroup {
  public Score(IntakeSubsystem intake, String pathName, Elevator elevator, double setpoint) {
    addCommands(
        new PathPlannerAuto(pathName), // Drive to the scoring location
        new ElevatorPID(elevator, setpoint),
        new AutoShootCommand(intake).withTimeout(2) // Shoot for 2 seconds
    );
  }
}
