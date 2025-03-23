// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines.Routines.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ElevatorPID;
import frc.robot.commands.ReverseCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RaiseAndShootCommand extends ParallelCommandGroup {
  /** Creates a new RaiseAndShootCommand. */
  public RaiseAndShootCommand(IntakeSubsystem intake, Elevator elevator, double setpoint) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Raise elevator
      new ElevatorPID(elevator, setpoint),
      Commands.sequence(
        //Once we are at the setpoint, shoot for 2 seconds
        Commands.waitSeconds(5),
        Commands.deadline(
          Commands.waitSeconds(2),
          new ShootCommand(intake, () -> 0.5)
        )
      )
    );
  }
}
