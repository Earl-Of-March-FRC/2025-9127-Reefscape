// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoRoutines.Routines.TimedRoutines.ExitZoneCommand;
import frc.robot.commands.AutoRoutines.Routines.ToReefScore.ScoreL1;
import frc.robot.commands.DriveFieldOriented;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.commands.DriveFieldOriented;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ReverseCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;


public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final XboxController driveController = new XboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final IntakeSubsystem intakeSub = new IntakeSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();


    intakeSub.setDefaultCommand(
        new ShootCommand(
            intakeSub,
            () -> operatorController.getLeftTriggerAxis()
        )
    );


    // Configure the trigger bindings
    drivetrain.setDefaultCommand(new DriveFieldOriented(
      drivetrain,
      () -> (driveController.getLeftX()), //X translation
      () -> -(driveController.getLeftY()), //Y translation
      () -> (driveController.getRightX()) //Z rotation
      ));
    configureBindings();

    //Go to Reef commands
    autoChooser.addOption("GO TO REEF FROM RIGHT", new PathPlannerAuto("To reef from right"));
    autoChooser.addOption("GO TO REEF FROM LEFT", new PathPlannerAuto("To reef from left"));
    autoChooser.addOption("GO TO REEF FROM CENTRE", new PathPlannerAuto("To reef from centre"));

    //Score on L1 Commands
    autoChooser.addOption("SCORE L1 FROM RIGHT", new ScoreL1(intakeSub, "To reef from right"));
    autoChooser.addOption("SCORE L1 FROM LEFT", new ScoreL1(intakeSub, "To reef from left"));
    autoChooser.addOption("SCORE L1 FROM CENTRE", new ScoreL1(intakeSub, "To reef from centre"));

    //Exit Zone timed
    autoChooser.addOption("EXIT ZONE TIMED", new ExitZoneCommand(drivetrain, 1, 2));
    
    SmartDashboard.putData("Autonomous Routine", autoChooser);
  }

  private void configureBindings() {
    // Configure your button bindings here

    //Reset the gyro angle to 0 when A is pressed on the driver controller
    new Trigger(driveController::getAButtonPressed).onTrue(Commands.runOnce(() -> drivetrain.resetGyro(), drivetrain));
    
    //Toggle the drive mode (field or robot oriented) when B is pressed on the driver controller
    new Trigger(driveController::getBButtonPressed).onTrue(Commands.runOnce(() -> drivetrain.changeDriveMode(), drivetrain));

    //automatically intake with beam break sensor using button a
    operatorController.a().whileTrue(new IntakeCommand(intakeSub));

    //reverse direction for intake with right trigger
    new Trigger(() -> operatorController.getRightTriggerAxis() > 0.1)
        .whileTrue(new ReverseCommand(
            intakeSub,
            () -> operatorController.getRightTriggerAxis()
        ));
    }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}