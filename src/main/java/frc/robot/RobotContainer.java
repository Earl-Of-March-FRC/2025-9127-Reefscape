// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveFieldOriented;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final XboxController xboxController = new XboxController(0);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    drivetrain.setDefaultCommand(new DriveFieldOriented(
      drivetrain, false,//xboxController.getLeftBumperButtonPressed(),
      () -> (xboxController.getLeftX()), 
      () -> -(xboxController.getLeftY()), 
      () -> (xboxController.getRightX())
      ));

    configureBindings();
  }


  private void configureBindings() {
    // Configure your button bindings here
    
    // new Trigger(xboxController::getLeftBumperButtonPressed).onTrue(new Command() {
    //   @Override
    //   public void execute() {
    //       fieldOriented = !fieldOriented;
    //   }
    // });
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}