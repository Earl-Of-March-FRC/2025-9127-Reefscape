// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines.Routines.ToReefScore;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoRoutines.Routines.AutoCommands.RaiseAndShootCommand;
import frc.robot.commands.AutoRoutines.Routines.TimedRoutines.ExitZoneCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TimedScore extends SequentialCommandGroup {
  /** Creates a new TimedScore. */
  public TimedScore(Drivetrain drive, IntakeSubsystem intake, Elevator elevator, double setpoint, double speed, double time) {
    addCommands(
        new ExitZoneCommand(drive, speed, time),
        new RaiseAndShootCommand(intake, elevator, setpoint)
    );
  }
}
