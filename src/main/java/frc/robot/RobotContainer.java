// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ReverseCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.IntakeSubsystem;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem intakeSub = new IntakeSubsystem();
  private final CommandXboxController xboxController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
public RobotContainer() {
    intakeSub.setDefaultCommand(
        new ShootCommand(
            intakeSub,
            () -> xboxController.getLeftTriggerAxis()
        )
    );


    // Configure the trigger bindings
    configureBindings();
  }


  private void configureBindings() {
    // Configure controller bindings here...
    xboxController.a().whileTrue(new IntakeCommand(intakeSub));
    new Trigger(() -> xboxController.getRightTriggerAxis() > 0.1)
        .whileTrue(new ReverseCommand(
            intakeSub,
            () -> xboxController.getRightTriggerAxis()
        ));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
