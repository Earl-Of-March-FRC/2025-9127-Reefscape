
package frc.robot.commands.AutoRoutines.Routines.ToReefScore;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoRoutines.Routines.AutoCommands.RaiseAndShootCommand;
import frc.robot.commands.ElevatorPID;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeSubsystem;


public class ScoreandStation extends SequentialCommandGroup {
  public ScoreandStation(IntakeSubsystem intake, String pathName, Elevator elevator, double setpoint, String secondPath) {
    addCommands(
        new PathPlannerAuto(pathName), // Drive to the scoring location
        new RaiseAndShootCommand(intake, elevator, setpoint), // Shoot for 2 seconds
        new PathPlannerAuto(secondPath) // Drive to the scoring location
    );
  }
}
